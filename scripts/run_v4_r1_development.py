"""Execute only the frozen V4 Development bank for V4-R1.

The script has no code path that loads the V4 Holdout manifest.  It evaluates
the four immutable comparators or preregistered CART-OFMPC candidates and keeps
compact per-sample evidence plus controller-mechanism aggregates.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import time
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import mujoco
import numpy as np
import yaml

from uav_sway.control.base import ReferenceState
from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.control.state_reader import StateReader
from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind
from uav_sway.evaluation.task_space_metrics import first_continuous_acquisition
from uav_sway.models.model_config import load_model_config
from uav_sway.v3.controllers import V3CascadedTaskPID, V3FullStateLQR, V3TaskWeightedLQR
from uav_sway.v3.dr_tsrmpc import V3DRTSRMPC
from uav_sway.v3.metrics import load_r0_linear_matrices
from uav_sway.v3.observation import V3Reference, V3StateReader, reference_for_target
from uav_sway.v4.cart_ofmpc import CARTOFMPC


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v4/r0"
R1 = ROOT / "reproducibility/v4/r1"
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
DT = 0.001
OUTER_DT = 0.05
LOG_DT = 0.005


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("\n", encoding="utf-8", newline="\n")
        return
    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n", extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    return (
        float(math.atan2(rotation[2, 1], rotation[2, 2])),
        float(math.asin(np.clip(-rotation[2, 0], -1.0, 1.0))),
        float(math.atan2(rotation[1, 0], rotation[0, 0])),
    )


def wind_value(spec: dict, time_s: float, stochastic: np.ndarray | None, index: int) -> float:
    kind = spec["kind"]
    if kind == "calm":
        return 0.0
    if kind == "constant":
        return float(spec["speed_m_s"]) if time_s >= float(spec.get("onset_time_s", 0.0)) else 0.0
    if kind == "ramp":
        onset = float(spec["onset_time_s"])
        fraction = np.clip((time_s - onset) / float(spec["duration_s"]), 0.0, 1.0)
        return float(spec.get("start_speed_m_s", 0.0)) + fraction * (float(spec["speed_m_s"]) - float(spec.get("start_speed_m_s", 0.0)))
    if kind == "stochastic" and stochastic is not None:
        return float(stochastic[min(index, len(stochastic) - 1)])
    raise ValueError(f"unsupported V4 Development wind kind: {kind}")


def stochastic_series(spec: dict, duration_s: float) -> np.ndarray | None:
    if spec["kind"] != "stochastic":
        return None
    times = np.arange(0.0, duration_s + 0.5 * LOG_DT, LOG_DT)
    rng = np.random.Generator(np.random.PCG64(int(spec["seed"])))
    alpha = float(np.exp(-LOG_DT / 1.0))
    sigma = float(spec.get("sigma_m_s", 0.8))
    clip = float(spec.get("clip_m_s", 3.0))
    result = np.zeros_like(times)
    for index in range(1, len(times)):
        result[index] = np.clip(alpha * result[index - 1] + np.sqrt(1.0 - alpha * alpha) * sigma * float(rng.standard_normal()), -clip, clip)
    return result


def load_metric_and_model() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    a, b = load_r0_linear_matrices(ROOT)
    metric = read_json(ROOT / "reproducibility/v3/r1/task_metric_alignment_audit.json")
    c_task = np.vstack([metric[name] for name in ("C_pos", "C_vel", "C_dir", "C_omega_perp")])
    return a, b, c_task


def comparator_specs() -> list[tuple[str, dict]]:
    files = (
        ("corrected_pid", ROOT / "reproducibility/v3/r1r1/pid_freeze.json"),
        ("full_lqr", ROOT / "reproducibility/v3/r1/full_lqr_freeze.json"),
        ("task_lqr", ROOT / "reproducibility/v3/r1/task_lqr_freeze.json"),
        ("legacy_self", ROOT / "reproducibility/v3/r2/self_freeze.json"),
    )
    return [(kind, read_json(path)["parameters"]) for kind, path in files]


def controller(kind: str, parameters: dict):
    if kind == "corrected_pid":
        return V3CascadedTaskPID(
            np.asarray(parameters["uav_kp"]), np.asarray(parameters["uav_kd"]), np.asarray(parameters["uav_ki"]),
            np.asarray(parameters["tip_kp"]), np.asarray(parameters["tip_kd"]), np.asarray(parameters["correction_limit_m"]),
            float(parameters["correction_slew_m_per_update"]), float(parameters["integral_limit"]), str(parameters["tip_velocity_mode"]),
        )
    if kind == "full_lqr":
        return V3FullStateLQR(np.asarray(parameters["K"]))
    if kind == "task_lqr":
        return V3TaskWeightedLQR(np.asarray(parameters["K"]))
    a, b, c_task = load_metric_and_model()
    if kind == "legacy_self":
        return V3DRTSRMPC(a, b, c_task, np.asarray(parameters["K"]), parameters)
    if kind == "cart_ofmpc":
        gain = np.asarray(read_json(ROOT / "reproducibility/v3/r1/task_lqr_freeze.json")["parameters"]["K"])
        return CARTOFMPC(a, b, c_task, gain, parameters)
    raise KeyError(kind)


def run_case(kind: str, parameters: dict, sample: dict) -> dict:
    if not bool(sample.get("execution_allowed", False)):
        raise RuntimeError(f"Development runner refuses sample without execution permission: {sample['sample_id']}")
    duration = float(sample["duration_s"])
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0; data.ctrl[:] = 0.0; data.eq_active[:] = 0
    mujoco.mj_forward(model, data)
    model_config = load_model_config(ROOT / "configs/model_5link.yaml")
    s3 = yaml.safe_load((ROOT / "configs/s3_pid.yaml").read_text(encoding="utf-8"))
    aero = load_aerodynamic_config(ROOT / "configs/aerodynamics.yaml")
    quad_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    equilibrium_tip = np.asarray(data.site_xpos[tip_id], dtype=float).copy()
    equilibrium_relative = equilibrium_tip - np.asarray(data.xpos[quad_id], dtype=float)
    state_reader = V3StateReader(model)
    legacy_reader = StateReader(model, model_config.n_links, float(equilibrium_relative[0]))
    total_mass = float(np.sum(model.body_mass))
    inner = GeometricInnerLoop(total_mass, np.asarray(model.body_inertia[quad_id]), s3["attitude_natural_frequency_rad_s"], s3["attitude_damping_ratio"], s3["position_gains_y"][0], s3["position_gains_y"][1], s3["position_gains_z"][0], s3["position_gains_z"][1])
    outer = controller(kind, parameters); outer.reset()
    stochastic = stochastic_series(sample["wind"], duration)
    target_delta = np.asarray(sample["target"]["delta_tip_m"], dtype=float)
    issue_time = sample.get("target_issue_time_s")
    physics_steps = int(round(duration / DT)); wind_stride = int(round(LOG_DT / DT)); outer_stride = int(round(OUTER_DT / DT))
    command = np.zeros(3)
    reference = reference_for_target(equilibrium_tip, equilibrium_relative, 0.0)
    reference_state = ReferenceState(float(reference.uav_position_world[0]), 0.0, 0.0, float(reference.uav_position_world[1]), float(reference.uav_position_world[2]), 0.0)
    actuator_ids = {name: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)) for name in ("thrust_motor", "mx_motor", "my_motor", "mz_motor")}
    rows: list[dict] = []; updates: list[dict] = []; safety_reasons: set[str] = set()
    max_abs_command = np.zeros(3); max_step = np.zeros(3); previous_logged = np.zeros(3)
    max_thrust = 0.0; max_torque = 0.0; force = {"total_x": 0.0}
    for step in range(physics_steps + 1):
        time_s = step * DT
        wind_index = step // wind_stride
        if step % wind_stride == 0:
            wind = wind_value(sample["wind"], time_s, stochastic, wind_index)
            force = clear_and_apply_wind(model, data, model_config, aero, wind)
        if step % outer_stride == 0:
            target = equilibrium_tip + target_delta if issue_time is not None and time_s >= float(issue_time) else equilibrium_tip
            reference = reference_for_target(target, equilibrium_relative, time_s)
            reference_state = ReferenceState(float(reference.uav_position_world[0]), 0.0, 0.0, float(reference.uav_position_world[1]), float(reference.uav_position_world[2]), 0.0)
            observation = state_reader.read(model, data, reference)
            started = time.perf_counter_ns(); command = outer.command(observation, reference, OUTER_DT); external_solve_ms = (time.perf_counter_ns() - started) / 1.0e6
            diag = outer.diagnostics
            update = {"solve_time_ms": float(getattr(diag, "solve_time_ms", external_solve_ms)), "saturated": bool(np.any(diag.saturated)), "slew_limited": bool(np.any(diag.slew_limited))}
            for name in (
                "residual_clip_fraction", "trust", "residual_unrepresented_norm", "residual_debt_norm",
                "steady_equality_residual", "steady_task_residual", "steady_feasible", "steady_constraint_active",
                "reachable_constraint_active", "limiter_mismatch", "qp_iterations",
            ):
                if hasattr(diag, name): update[name] = getattr(diag, name)
            if hasattr(diag, "requested_steady_input"):
                update["requested_steady_input_max_abs"] = float(np.max(np.abs(diag.requested_steady_input)))
                update["feasible_steady_input_max_abs"] = float(np.max(np.abs(diag.feasible_steady_input)))
                update["backbone_norm"] = float(np.linalg.norm(diag.backbone_command))
                update["qp_correction_norm"] = float(np.linalg.norm(diag.qp_correction))
                update["qp_status"] = str(diag.qp_status)
            elif hasattr(diag, "qp_status"):
                update["qp_status"] = str(diag.qp_status)
                update["limiter_mismatch"] = float(diag.limiter_mismatch)
            updates.append(update)
        if step % wind_stride == 0:
            control_state = legacy_reader.read(model, data)
            inner_output = inner.compute(control_state, reference_state, float(command[0]), (float(command[1]), float(command[2])))
            thrust_raw = float(inner_output["thrust_raw_N"]); torque_raw = np.asarray(inner_output["torque_raw_Nm"])
            thrust = float(np.clip(thrust_raw, *model.actuator_ctrlrange[actuator_ids["thrust_motor"]]))
            torque = np.asarray([np.clip(torque_raw[index], *model.actuator_ctrlrange[actuator_ids[name]]) for index, name in enumerate(("mx_motor", "my_motor", "mz_motor"))])
            data.ctrl[:] = 0.0; data.ctrl[actuator_ids["thrust_motor"]] = thrust
            for value, name in zip(torque, ("mx_motor", "my_motor", "mz_motor")): data.ctrl[actuator_ids[name]] = value
            task = state_reader.task_reader.read(model, data); target = reference.tip_position_world
            position_error = task.tip_position_world - target; tip_speed = float(np.linalg.norm(task.tip_velocity_world))
            orientation_deg = float(np.rad2deg(np.arccos(np.clip(task.cutter_axis_world @ np.asarray([1.0, 0.0, 0.0]), -1.0, 1.0))))
            angular_speed = float(np.linalg.norm(task.cutter_angular_velocity_world)); roll, pitch, _ = _rpy(control_state.rotation)
            current = np.asarray(command); torque_limit = max(max(abs(model.actuator_ctrlrange[actuator_ids[name], 0]), abs(model.actuator_ctrlrange[actuator_ids[name], 1])) for name in ("mx_motor", "my_motor", "mz_motor"))
            finite = bool(np.isfinite(np.r_[position_error, task.tip_velocity_world, task.cutter_angular_velocity_world, current, thrust, torque]).all())
            limits_ok = bool(np.all(np.abs(current) <= 2.0 + 1.0e-9) and np.all(np.abs(current - previous_logged) <= 0.25 + 1.0e-9))
            safety = finite and control_state.position[2] > 0.05 and task.tip_position_world[2] > 0.05 and np.max(np.abs(control_state.joint_angles)) < np.deg2rad(100.0) and abs(roll) < np.deg2rad(25.0) and abs(pitch) < np.deg2rad(25.0) and limits_ok and thrust_raw <= model.actuator_ctrlrange[actuator_ids["thrust_motor"], 1] + 1.0e-9 and np.max(np.abs(torque_raw)) <= torque_limit + 1.0e-9
            if not finite: safety_reasons.add("non_finite")
            if control_state.position[2] <= 0.05: safety_reasons.add("uav_height")
            if task.tip_position_world[2] <= 0.05: safety_reasons.add("tip_height")
            if np.max(np.abs(control_state.joint_angles)) >= np.deg2rad(100.0): safety_reasons.add("joint_angle")
            if abs(roll) >= np.deg2rad(25.0) or abs(pitch) >= np.deg2rad(25.0): safety_reasons.add("attitude")
            if not limits_ok: safety_reasons.add("command_limit")
            max_abs_command = np.maximum(max_abs_command, np.abs(current)); max_step = np.maximum(max_step, np.abs(current - previous_logged))
            max_thrust = max(max_thrust, abs(thrust_raw)); max_torque = max(max_torque, float(np.max(np.abs(torque_raw))))
            rows.append({"time": time_s, "position": float(np.linalg.norm(position_error)), "speed": tip_speed, "orientation": orientation_deg, "angular": angular_speed, "safe": bool(safety), "command": current.copy()})
            previous_logged = current.copy()
        if step < physics_steps: mujoco.mj_step(model, data)
    times = np.asarray([row["time"] for row in rows]); position = np.asarray([row["position"] for row in rows]); speed = np.asarray([row["speed"] for row in rows]); orientation = np.asarray([row["orientation"] for row in rows]); angular = np.asarray([row["angular"] for row in rows])
    start = 0.0 if issue_time is None else float(issue_time)
    acquired, timestamp = first_continuous_acquisition(times, (position <= 0.05) & (speed <= 0.10) & (orientation <= 5.0) & (angular <= 0.10), hold_time_s=1.0, start_time_s=start)
    commands = np.asarray([row["command"] for row in rows]); ramp_mask = (times >= 2.0) & (times <= 8.0); steady_mask = times >= 8.0
    result = {
        "sample_id": sample["sample_id"], "cohort": sample["cohort"], "controller": kind, "candidate_id": parameters["candidate_id"],
        "safe": bool(all(row["safe"] for row in rows)), "task_success": bool(acquired), "acquisition_time_s": float(timestamp - start) if acquired and timestamp is not None else None,
        "position_rmse_3d_m": float(np.sqrt(np.mean(position ** 2))), "orientation_rmse_deg": float(np.sqrt(np.mean(orientation ** 2))),
        "ramp_peak_position_error_m": float(np.max(position[ramp_mask])) if np.any(ramp_mask) else 0.0, "ramp_steady_state_position_error_m": float(np.mean(position[steady_mask])) if np.any(steady_mask) else 0.0,
        "total_acceleration_effort": float(np.trapezoid(np.sum(commands ** 2, axis=1), times)), "max_abs_command_m_s2": float(np.max(max_abs_command)), "max_command_step_m_s2": float(np.max(max_step)),
        "max_thrust_N": max_thrust, "max_abs_torque_Nm": max_torque, "max_cutter_angular_speed_rad_s": float(np.max(angular)), "safety_failure_reasons": sorted(safety_reasons),
        "solve_time_p95_ms": float(np.percentile([row["solve_time_ms"] for row in updates], 95)), "amplitude_saturation_fraction": float(np.mean([row["saturated"] for row in updates])), "slew_activity_fraction": float(np.mean([row["slew_limited"] for row in updates])),
    }
    if kind == "cart_ofmpc":
        numeric = ("residual_clip_fraction", "trust", "residual_unrepresented_norm", "residual_debt_norm", "steady_equality_residual", "steady_task_residual", "requested_steady_input_max_abs", "feasible_steady_input_max_abs", "backbone_norm", "qp_correction_norm", "limiter_mismatch")
        for name in numeric:
            values = [float(row[name]) for row in updates]
            result[name + "_mean"] = float(np.mean(values)); result[name + "_p95"] = float(np.percentile(values, 95)); result[name + "_max"] = float(np.max(values))
        result["steady_infeasibility_fraction"] = float(np.mean([not bool(row["steady_feasible"]) for row in updates]))
        result["steady_constraint_activity_fraction"] = float(np.mean([bool(row["steady_constraint_active"]) for row in updates]))
        result["reachable_constraint_activity_fraction"] = float(np.mean([bool(row["reachable_constraint_active"]) for row in updates]))
        result["solver_success_rate"] = float(np.mean([row.get("qp_status") in {"solved", "solved inaccurate"} for row in updates]))
        result["qp_iterations_max"] = int(max(row["qp_iterations"] for row in updates))
    return result


def aggregate(candidate_id: str, parameters: dict, rows: list[dict]) -> dict:
    acquisition = [row["acquisition_time_s"] for row in rows if row["task_success"] and row["acquisition_time_s"] is not None]
    result = {
        "candidate_id": candidate_id, "sample_count": len(rows), "safe_sample_count": int(sum(row["safe"] for row in rows)), "task_success_count": int(sum(row["task_success"] for row in rows)),
        "safety_rate": float(np.mean([row["safe"] for row in rows])), "success_rate": float(np.mean([row["task_success"] for row in rows])), "acquisition_median_s": float(np.median(acquisition)) if acquisition else None,
        "position_rmse_3d_m": float(np.mean([row["position_rmse_3d_m"] for row in rows])), "position_p90_3d_m": float(np.percentile([row["position_rmse_3d_m"] for row in rows], 90)), "position_p95_3d_m": float(np.percentile([row["position_rmse_3d_m"] for row in rows], 95)),
        "orientation_rmse_deg": float(np.mean([row["orientation_rmse_deg"] for row in rows])), "ramp_peak_position_error_m": float(max(row["ramp_peak_position_error_m"] for row in rows)), "ramp_steady_state_position_error_m": float(max(row["ramp_steady_state_position_error_m"] for row in rows)),
        "total_acceleration_effort": float(np.mean([row["total_acceleration_effort"] for row in rows])), "solve_time_p95_ms": float(max(row["solve_time_p95_ms"] for row in rows)), "parameters": parameters,
    }
    if rows and rows[0]["controller"] == "cart_ofmpc":
        for name in ("residual_clip_fraction_mean", "steady_infeasibility_fraction", "trust_mean", "residual_unrepresented_norm_mean", "residual_debt_norm_mean", "requested_steady_input_max_abs_mean", "feasible_steady_input_max_abs_mean", "amplitude_saturation_fraction", "slew_activity_fraction", "backbone_norm_mean", "qp_correction_norm_mean", "limiter_mismatch_max", "solver_success_rate"):
            result[name] = float(np.mean([row[name] for row in rows]))
    return result


def cohort_summaries(parameters: dict, rows: list[dict]) -> dict:
    return {cohort: aggregate(parameters["candidate_id"], parameters, [row for row in rows if row["cohort"] == cohort]) for cohort in sorted({row["cohort"] for row in rows})}


def cache_file(phase: str, candidate_id: str, sample_id: str) -> Path:
    return R1 / "_cache" / phase / candidate_id / f"{sample_id}.json"


def execute_job(kind: str, parameters: dict, sample: dict, phase: str) -> dict:
    path = cache_file(phase, parameters["candidate_id"], sample["sample_id"])
    if path.exists(): return read_json(path)
    row = run_case(kind, parameters, sample); write_json(path, row); return row


def run_bank(kind: str, parameters: dict, samples: list[dict], phase: str, workers: int) -> tuple[dict, list[dict]]:
    rows: dict[str, dict] = {}
    if workers <= 1:
        for index, sample in enumerate(samples, 1):
            rows[sample["sample_id"]] = execute_job(kind, parameters, sample, phase)
            if index % 10 == 0 or index == len(samples): print(f"{phase}:{parameters['candidate_id']} {index}/{len(samples)}", flush=True)
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            futures = {pool.submit(execute_job, kind, parameters, sample, phase): sample["sample_id"] for sample in samples}
            for index, future in enumerate(as_completed(futures), 1):
                rows[futures[future]] = future.result()
                if index % 20 == 0 or index == len(samples): print(f"{phase}:{parameters['candidate_id']} {index}/{len(samples)}", flush=True)
    ordered = [rows[sample["sample_id"]] for sample in samples]
    summary = aggregate(parameters["candidate_id"], parameters, ordered)
    summary["cohorts"] = cohort_summaries(parameters, ordered)
    return summary, ordered


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--mode", choices=("baseline", "stage-a", "stage-b", "stage-c"), required=True); parser.add_argument("--workers", type=int, default=min(24, os.cpu_count() or 1)); args = parser.parse_args()
    manifest = read_json(R0 / "development_manifest.json")
    if manifest["name"] != "V4_FROZEN_DEVELOPMENT_BANK" or len(manifest["samples"]) != 94: raise RuntimeError("unexpected frozen Development manifest")
    samples = manifest["samples"]
    if args.mode == "baseline":
        all_rows: list[dict] = []; summaries: dict[str, dict] = {}
        for kind, parameters in comparator_specs():
            summary, rows = run_bank(kind, parameters, samples, "baseline", args.workers); summaries[parameters["candidate_id"]] = summary; all_rows.extend(rows)
        write_csv(R1 / "baseline_development_results.csv", all_rows); write_json(R1 / "baseline_summary.json", {"authoritative_runs": len(all_rows), "controllers": summaries, "cohort_counts": Counter(row["cohort"] for row in samples), "holdout_executed": False})
        return 0
    protocol = read_json(R1 / "development_protocol.json")
    stage_key = args.mode.replace("-", "_")
    candidate_rows = protocol["search"][stage_key]["candidates"] if stage_key != "stage_c" else protocol["search"]["stage_c"]["candidate_source"]
    selected_samples = samples
    if stage_key == "stage_a":
        ids = set(protocol["search"]["stage_a"]["sample_ids"]); selected_samples = [sample for sample in samples if sample["sample_id"] in ids]
    if stage_key == "stage_c":
        history = read_json(R1 / "search_history.json"); ids = history["stage_c_selected_candidate_ids"]; by_id = {row["candidate_id"]: row for row in protocol["search"]["stage_b"]["candidates"]}; candidate_rows = [by_id[value] for value in ids]
    summaries: list[dict] = []; all_rows = []
    for parameters in candidate_rows:
        summary, rows = run_bank("cart_ofmpc", parameters, selected_samples, stage_key, args.workers); summaries.append(summary); all_rows.extend(rows)
    write_json(R1 / f"{stage_key}_summary.json", {"stage": stage_key, "candidate_count": len(candidate_rows), "sample_count_each": len(selected_samples), "summaries": summaries, "holdout_executed": False})
    write_csv(R1 / f"{stage_key}_results.csv", all_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
