"""Instrument-only V4-R0 postmortem of the frozen V3 controllers.

This script never changes controller parameters and never reads a V4 Holdout.
It replays frozen V3 evidence as V4 prior/failure-discovery data and emits only
compact per-case diagnostics.
"""

from __future__ import annotations

import csv
import json
import math
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
from uav_sway.v3.controllers import V3FullStateLQR
from uav_sway.v3.dr_tsrmpc import V3DRTSRMPC
from uav_sway.v3.metrics import load_r0_linear_matrices
from uav_sway.v3.observation import V3StateReader, reference_for_target


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v3/r0"
R1 = ROOT / "reproducibility/v3/r1"
R2 = ROOT / "reproducibility/v3/r2"
R4 = ROOT / "reproducibility/v3/r4"
OUT = ROOT / "reproducibility/v4/r0/strong_wind_diagnostic.csv"
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
DT = 0.001
OUTER_DT = 0.05
LOG_DT = 0.005
DURATION = 12.0


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = list(rows[0])
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def controller(kind: str):
    if kind == "self_a_034":
        parameters = read_json(R2 / "self_freeze.json")["parameters"]
        a, b = load_r0_linear_matrices(ROOT)
        metric = read_json(R1 / "task_metric_alignment_audit.json")
        c_task = np.vstack([metric[name] for name in ("C_pos", "C_vel", "C_dir", "C_omega_perp")])
        return V3DRTSRMPC(a, b, c_task, np.asarray(parameters["K"]), parameters)
    if kind == "full_lqr_048":
        parameters = read_json(R1 / "full_lqr_freeze.json")["parameters"]
        return V3FullStateLQR(np.asarray(parameters["K"]))
    raise KeyError(kind)


def wind_value(spec: dict, time_s: float) -> float:
    kind = spec["kind"]
    if kind == "calm":
        return 0.0
    if kind == "constant":
        return float(spec["speed_m_s"]) if time_s >= float(spec.get("onset_time_s", 0.0)) else 0.0
    if kind == "ramp":
        start = float(spec.get("ramp_start_s", 2.0))
        duration = float(spec.get("ramp_duration_s", 6.0))
        return float(spec["speed_m_s"]) * float(np.clip((time_s - start) / duration, 0.0, 1.0))
    raise KeyError(kind)


def percentile(values: list[float], q: float) -> float:
    return float(np.percentile(np.asarray(values, dtype=float), q)) if values else 0.0


def run_case(kind: str, case: dict) -> dict:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    data.eq_active[:] = 0
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
    inner = GeometricInnerLoop(
        total_mass,
        np.asarray(model.body_inertia[quad_id], dtype=float),
        s3["attitude_natural_frequency_rad_s"],
        s3["attitude_damping_ratio"],
        s3["position_gains_y"][0],
        s3["position_gains_y"][1],
        s3["position_gains_z"][0],
        s3["position_gains_z"][1],
    )
    outer = controller(kind)
    outer.reset()
    target_delta = np.asarray(case["target_delta_tip_m"], dtype=float)
    target_issue = case.get("target_issue_time_s")
    reference = reference_for_target(equilibrium_tip, equilibrium_relative, 0.0)
    reference_state = ReferenceState(
        float(reference.uav_position_world[0]), 0.0, 0.0,
        float(reference.uav_position_world[1]), float(reference.uav_position_world[2]), 0.0,
    )
    actuator_ids = {
        name: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name))
        for name in ("thrust_motor", "mx_motor", "my_motor", "mz_motor")
    }

    command = np.zeros(3)
    previous_outer_command = np.zeros(3)
    position_errors: list[float] = []
    tip_speeds: list[float] = []
    orientation_errors: list[float] = []
    angular_speeds: list[float] = []
    command_sq: list[float] = []
    joint_angles: list[float] = []
    force_values: list[float] = []
    outer_rows: list[dict] = []
    safe_all = True
    force = {"total_x": 0.0}
    physics_steps = int(round(DURATION / DT))
    wind_stride = int(round(LOG_DT / DT))
    outer_stride = int(round(OUTER_DT / DT))

    for step in range(physics_steps + 1):
        time_s = step * DT
        if step % wind_stride == 0:
            force = clear_and_apply_wind(model, data, model_config, aero, wind_value(case["wind"], time_s))
        if step % outer_stride == 0:
            active_target = target_issue is not None and time_s >= float(target_issue)
            target = equilibrium_tip + target_delta if active_target else equilibrium_tip
            reference = reference_for_target(target, equilibrium_relative, time_s)
            reference_state = ReferenceState(
                float(reference.uav_position_world[0]), 0.0, 0.0,
                float(reference.uav_position_world[1]), float(reference.uav_position_world[2]), 0.0,
            )
            observation = state_reader.read(model, data, reference)
            previous_outer_command = command.copy()
            command = outer.command(observation, reference, OUTER_DT)
            diag = outer.diagnostics
            row = {
                "time_s": time_s,
                "state_norm": float(np.linalg.norm(observation.full_state_error)),
                "raw_command_norm": float(np.linalg.norm(diag.raw_command)),
                "command_norm": float(np.linalg.norm(command)),
                "physical_saturation": bool(np.any(np.abs(diag.raw_command) >= 2.0 - 1.0e-5)),
                "slew_active": bool(np.any(np.abs(command - previous_outer_command) >= 0.25 - 1.0e-5)),
                "wind_x": wind_value(case["wind"], time_s),
            }
            if kind == "self_a_034":
                steady = outer.steady.solve(diag.d_projected)
                particular = outer.steady.pinv @ diag.d_projected
                reduced_hessian = outer.steady.null.T @ outer.steady.weight @ outer.steady.null
                reduced_linear = outer.steady.null.T @ outer.steady.weight @ particular
                unscaled_solution = particular - outer.steady.null @ np.linalg.solve(reduced_hessian, reduced_linear)
                unscaled_steady_input = unscaled_solution[20:]
                steady_scale = min(1.0, 1.9 / max(float(np.max(np.abs(unscaled_steady_input))), 1.0e-12))
                backbone = -outer.gain @ (observation.full_state_error - steady.state)
                correction = diag.raw_command - steady.command - backbone
                row.update({
                    "d_raw_norm": float(diag.d_raw_norm),
                    "d_hat_norm": float(diag.d_hat_norm),
                    "d_projected_norm": float(diag.d_projected_norm),
                    "d_rejected_norm": float(diag.d_rejected_norm),
                    "raw_clip_components": int(np.count_nonzero(np.abs(diag.d_raw) > outer.estimator.component_limit)),
                    "steady_input_norm": float(np.linalg.norm(diag.steady_input)),
                    "steady_input_max_abs": float(np.max(np.abs(diag.steady_input))),
                    "steady_input_infeasible": bool(np.any(np.abs(diag.steady_input) > 2.0)),
                    "unscaled_steady_input_max_abs": float(np.max(np.abs(unscaled_steady_input))),
                    "steady_scale": float(steady_scale),
                    "steady_scaling_active": bool(steady_scale < 1.0 - 1.0e-12),
                    "steady_state_residual": float(diag.steady_state_residual),
                    "steady_task_residual": float(diag.steady_task_residual),
                    "backbone_norm": float(np.linalg.norm(backbone)),
                    "qp_correction_norm": float(np.linalg.norm(correction)),
                    "limiter_mismatch": float(diag.limiter_mismatch),
                    "qp_iterations": int(diag.qp_iterations),
                    "qp_solved": str(diag.qp_status) in {"solved", "solved inaccurate"},
                })
                for state_index, value in enumerate(diag.d_raw):
                    row[f"raw_clip_{state_index:02d}"] = bool(abs(float(value)) > outer.estimator.component_limit)
            outer_rows.append(row)

        if step % wind_stride == 0:
            control_state = legacy_reader.read(model, data)
            inner_output = inner.compute(
                control_state, reference_state, float(command[0]), (float(command[1]), float(command[2]))
            )
            thrust_raw = float(inner_output["thrust_raw_N"])
            torque_raw = np.asarray(inner_output["torque_raw_Nm"], dtype=float)
            thrust = float(np.clip(thrust_raw, *model.actuator_ctrlrange[actuator_ids["thrust_motor"]]))
            torque = np.asarray([
                np.clip(torque_raw[index], *model.actuator_ctrlrange[actuator_ids[name]])
                for index, name in enumerate(("mx_motor", "my_motor", "mz_motor"))
            ])
            data.ctrl[:] = 0.0
            data.ctrl[actuator_ids["thrust_motor"]] = thrust
            data.ctrl[actuator_ids["mx_motor"]] = torque[0]
            data.ctrl[actuator_ids["my_motor"]] = torque[1]
            data.ctrl[actuator_ids["mz_motor"]] = torque[2]
            task = state_reader.task_reader.read(model, data)
            position_error = float(np.linalg.norm(task.tip_position_world - reference.tip_position_world))
            tip_speed = float(np.linalg.norm(task.tip_velocity_world))
            axis_dot = float(np.clip(task.cutter_axis_world @ np.asarray([1.0, 0.0, 0.0]), -1.0, 1.0))
            orientation = float(np.rad2deg(np.arccos(axis_dot)))
            angular_speed = float(np.linalg.norm(task.cutter_angular_velocity_world))
            roll = float(math.atan2(control_state.rotation[2, 1], control_state.rotation[2, 2]))
            pitch = float(math.asin(np.clip(-control_state.rotation[2, 0], -1.0, 1.0)))
            finite = bool(np.isfinite(np.r_[position_error, tip_speed, orientation, angular_speed, command]).all())
            safe_all = safe_all and finite and abs(roll) < np.deg2rad(25.0) and abs(pitch) < np.deg2rad(25.0)
            position_errors.append(position_error)
            tip_speeds.append(tip_speed)
            orientation_errors.append(orientation)
            angular_speeds.append(angular_speed)
            command_sq.append(float(command @ command))
            joint_angles.append(float(np.max(np.abs(control_state.joint_angles))))
            force_values.append(float(force["total_x"]))
        if step < physics_steps:
            mujoco.mj_step(model, data)

    times = np.arange(len(position_errors), dtype=float) * LOG_DT
    issue_start = 0.0 if target_issue is None else float(target_issue)
    acquired, timestamp = first_continuous_acquisition(
        times,
        (np.asarray(position_errors) <= 0.05)
        & (np.asarray(tip_speeds) <= 0.10)
        & (np.asarray(orientation_errors) <= 5.0)
        & (np.asarray(angular_speeds) <= 0.10),
        hold_time_s=1.0,
        start_time_s=issue_start,
    )
    result = {
        "diagnostic_id": case["diagnostic_id"],
        "source_sample_id": case.get("source_sample_id", "V4_R0_TIMING_DIAGNOSTIC"),
        "controller": kind,
        "regime": case["regime"],
        "wind_kind": case["wind"]["kind"],
        "wind_speed_m_s": case["wind"].get("speed_m_s", 0.0),
        "wind_onset_time_s": case["wind"].get("onset_time_s", 0.0),
        "target_issue_time_s": "" if target_issue is None else target_issue,
        "safe": safe_all,
        "task_success": bool(acquired),
        "acquisition_time_s": "" if not acquired or timestamp is None else float(timestamp - issue_start),
        "position_rmse_3d_m": float(np.sqrt(np.mean(np.square(position_errors)))),
        "position_peak_m": float(np.max(position_errors)),
        "position_steady_mean_m": float(np.mean(np.asarray(position_errors)[times >= 8.0])),
        "orientation_rmse_deg": float(np.sqrt(np.mean(np.square(orientation_errors)))),
        "orientation_peak_deg": float(np.max(orientation_errors)),
        "cutter_angular_speed_peak_rad_s": float(np.max(angular_speeds)),
        "joint_angle_peak_deg": float(np.rad2deg(np.max(joint_angles))),
        "command_effort": float(np.trapezoid(command_sq, times)),
        "command_norm_peak": float(max(row["command_norm"] for row in outer_rows)),
        "raw_command_norm_peak": float(max(row["raw_command_norm"] for row in outer_rows)),
        "physical_saturation_fraction": float(np.mean([row["physical_saturation"] for row in outer_rows])),
        "slew_active_fraction": float(np.mean([row["slew_active"] for row in outer_rows])),
        "wind_force_peak_N": float(np.max(np.abs(force_values))),
    }
    self_fields = [
        "d_raw_norm", "d_hat_norm", "d_projected_norm", "d_rejected_norm",
        "steady_input_norm", "steady_input_max_abs", "unscaled_steady_input_max_abs", "steady_scale",
        "steady_state_residual", "steady_task_residual",
        "backbone_norm", "qp_correction_norm", "limiter_mismatch", "qp_iterations",
    ]
    for field in self_fields:
        values = [float(row[field]) for row in outer_rows if field in row]
        result[f"{field}_mean"] = float(np.mean(values)) if values else 0.0
        result[f"{field}_max"] = float(np.max(values)) if values else 0.0
    result.update({
        "raw_clip_update_fraction": float(np.mean([row.get("raw_clip_components", 0) > 0 for row in outer_rows])),
        "raw_clip_component_fraction": float(np.sum([row.get("raw_clip_components", 0) for row in outer_rows]) / (20 * len(outer_rows))),
        "steady_input_infeasible_fraction": float(np.mean([row.get("steady_input_infeasible", False) for row in outer_rows])),
        "steady_scaling_active_fraction": float(np.mean([row.get("steady_scaling_active", False) for row in outer_rows])),
        "qp_solved_fraction": float(np.mean([row.get("qp_solved", True) for row in outer_rows])),
    })
    for state_index in range(20):
        result[f"raw_clip_state_{state_index:02d}_fraction"] = float(
            np.mean([row.get(f"raw_clip_{state_index:02d}", False) for row in outer_rows])
        )
    pre_target = [row for row in outer_rows if target_issue is not None and row["time_s"] < float(target_issue)]
    near_target = [row for row in outer_rows if target_issue is not None and abs(row["time_s"] - float(target_issue)) <= 0.10]
    after_target = [row for row in outer_rows if target_issue is not None and float(target_issue) <= row["time_s"] < float(target_issue) + 1.0]
    result["pre_target_d_hat_p95"] = percentile([row.get("d_hat_norm", 0.0) for row in pre_target], 95)
    result["reference_step_d_raw_peak"] = max([row.get("d_raw_norm", 0.0) for row in near_target], default=0.0)
    result["post_target_1s_d_raw_mean"] = float(np.mean([row.get("d_raw_norm", 0.0) for row in after_target])) if after_target else 0.0
    result["post_target_1s_state_norm_peak"] = max([row["state_norm"] for row in after_target], default=0.0)
    return result


def build_cases() -> list[dict]:
    manifest = read_json(R4 / "holdout_execution_manifest.json")
    source = [sample for sample in manifest["samples"] if sample["sample_id"].startswith("holdout_constant_3p5_")]
    cases = [
        {
            "diagnostic_id": f"replay_{sample['sample_id']}",
            "source_sample_id": sample["sample_id"],
            "regime": "V3_CONSTANT_3P5_REPLAY",
            "target_delta_tip_m": sample["target"]["delta_tip_m"],
            "target_issue_time_s": 3.0,
            "wind": {"kind": "constant", "speed_m_s": 3.5, "onset_time_s": 0.0},
        }
        for sample in source
    ]
    representative = source[3]
    delta = representative["target"]["delta_tip_m"]
    cases.extend([
        {"diagnostic_id": "matched_calm", "regime": "MATCHED_CALM", "target_delta_tip_m": delta,
         "target_issue_time_s": 3.0, "wind": {"kind": "calm"}},
        {"diagnostic_id": "matched_constant_2p0", "regime": "MATCHED_CONSTANT_2P0", "target_delta_tip_m": delta,
         "target_issue_time_s": 3.0, "wind": {"kind": "constant", "speed_m_s": 2.0, "onset_time_s": 0.0}},
        {"diagnostic_id": "matched_ramp_equilibrium", "source_sample_id": "holdout_ramp_wind_equilibrium_hold",
         "regime": "V3_RAMP_0_TO_3P5_REPLAY", "target_delta_tip_m": [0.0, 0.0, 0.0],
         "target_issue_time_s": None, "wind": {"kind": "ramp", "speed_m_s": 3.5}},
        {"diagnostic_id": "timing_simultaneous_t3", "regime": "TIMING_SIMULTANEOUS", "target_delta_tip_m": delta,
         "target_issue_time_s": 3.0, "wind": {"kind": "constant", "speed_m_s": 3.5, "onset_time_s": 3.0}},
        {"diagnostic_id": "timing_post_target_t4", "regime": "TIMING_POST_TARGET", "target_delta_tip_m": delta,
         "target_issue_time_s": 1.0, "wind": {"kind": "constant", "speed_m_s": 3.5, "onset_time_s": 4.0}},
        {"diagnostic_id": "strong_wind_equilibrium_no_step", "regime": "STRONG_WIND_NO_REFERENCE_STEP",
         "target_delta_tip_m": [0.0, 0.0, 0.0], "target_issue_time_s": None,
         "wind": {"kind": "constant", "speed_m_s": 3.5, "onset_time_s": 0.0}},
    ])
    return cases


def main() -> int:
    cases = build_cases()
    rows = []
    for index, case in enumerate(cases, 1):
        for kind in ("full_lqr_048", "self_a_034"):
            rows.append(run_case(kind, case))
        print(f"v4-r0-postmortem {index}/{len(cases)}", flush=True)
    write_csv(OUT, rows)
    print(json.dumps({"result": "V4_R0_INSTRUMENT_ONLY_DIAGNOSTICS_COMPLETE", "cases": len(cases), "runs": len(rows)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
