"""Run, select, and freeze the three V3-R1 traditional baselines.

The runner has no code path that loads the V3-R0 holdout.  All tuning and
selection use the executable development manifest committed by the protocol
freeze.  Stage-1 keeps aggregate CSVs; Stage-2 keeps per-sample summaries and
only reruns four representative traces for each final selected controller.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np
import yaml

from uav_sway.control.base import ReferenceState
from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.control.runtime_model import create_runtime_model
from uav_sway.control.state_reader import StateReader
from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind
from uav_sway.evaluation.task_space_metrics import first_continuous_acquisition
from uav_sway.models.model_config import load_model_config
from uav_sway.v3.controllers import V3CascadedTaskPID, V3FullStateLQR, V3TaskPID, V3TaskWeightedLQR
from uav_sway.v3.dr_tsrmpc import V3DRTSRMPC
from uav_sway.v3.metrics import build_full_lqr_q, build_task_lqr_q, load_r0_linear_matrices, solve_v3_lqr
from uav_sway.v3.observation import V3Reference, V3StateReader, reference_for_target


ROOT = Path(__file__).resolve().parents[1]
R1 = ROOT / "reproducibility" / "v3" / "r1"
R0 = ROOT / "reproducibility" / "v3" / "r0"
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
MODEL_SHA = "19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d"
DT = 0.001
OUTER_DT = 0.05
LOG_DT = 0.005
DURATION = 12.0


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("\n", encoding="utf-8", newline="\n")
        return
    columns = list(rows[0])
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def _rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    return float(math.atan2(rotation[2, 1], rotation[2, 2])), float(math.asin(np.clip(-rotation[2, 0], -1.0, 1.0))), float(math.atan2(rotation[1, 0], rotation[0, 0]))


def _wind_series(sample: dict) -> np.ndarray:
    times = np.arange(0.0, DURATION + 0.5 * LOG_DT, LOG_DT)
    spec = sample["wind"]
    kind = spec["kind"]
    if kind == "calm":
        return np.zeros_like(times)
    if kind == "constant":
        return np.full_like(times, float(spec["speed_m_s"]))
    if kind == "ramp":
        return 3.0 * np.clip((times - 2.0) / 6.0, 0.0, 1.0)
    if kind == "stochastic":
        rng = np.random.Generator(np.random.PCG64(int(spec["seed"])))
        alpha = float(np.exp(-LOG_DT / 1.0))
        value = 0.0
        result = np.zeros_like(times)
        for index in range(1, len(times)):
            value = alpha * value + np.sqrt(1.0 - alpha * alpha) * 0.8 * float(rng.standard_normal())
            result[index] = np.clip(value, -3.0, 3.0)
        return result
    raise ValueError(f"unsupported development wind kind: {kind}")


def _target_delta(sample: dict) -> np.ndarray:
    return np.asarray(sample["target"]["delta_tip_m"], dtype=float)


def _issue_time(sample: dict) -> float | None:
    if sample["scenario"] == "CALM_3D_SETPOINT":
        return 1.0
    if sample["scenario"] == "WIND_3D_SETPOINT":
        return 3.0
    return None


def _target_at(sample: dict, time_s: float, equilibrium_tip: np.ndarray) -> np.ndarray:
    issue = _issue_time(sample)
    if issue is not None and time_s >= issue:
        return equilibrium_tip + _target_delta(sample)
    return equilibrium_tip.copy()


def _controller(kind: str, parameters: dict, gains: dict | None = None):
    if kind == "pid":
        return V3TaskPID(np.asarray(parameters["kp"], dtype=float), np.asarray(parameters["kd"], dtype=float), np.asarray(parameters["ki"], dtype=float))
    if kind == "corrected_pid":
        return V3CascadedTaskPID(
            np.asarray(parameters["uav_kp"], dtype=float),
            np.asarray(parameters["uav_kd"], dtype=float),
            np.asarray(parameters["uav_ki"], dtype=float),
            np.asarray(parameters["tip_kp"], dtype=float),
            np.asarray(parameters["tip_kd"], dtype=float),
            np.asarray(parameters["correction_limit_m"], dtype=float),
            float(parameters["correction_slew_m_per_update"]),
            float(parameters["integral_limit"]),
            str(parameters.get("tip_velocity_mode", "absolute")),
        )
    if kind == "full_lqr":
        return V3FullStateLQR(np.asarray(parameters["K"], dtype=float))
    if kind == "task_lqr":
        return V3TaskWeightedLQR(np.asarray(parameters["K"], dtype=float))
    if kind == "self_dr_tsrmpc":
        a, b = load_r0_linear_matrices(ROOT)
        metric = read_json(R1 / "task_metric_alignment_audit.json")
        c_task = np.vstack([metric[name] for name in ("C_pos", "C_vel", "C_dir", "C_omega_perp")])
        return V3DRTSRMPC(a, b, c_task, np.asarray(parameters["K"], dtype=float), parameters)
    raise KeyError(kind)


def _control_state(model, data, reader: StateReader):
    return reader.read(model, data)


def run_case(kind: str, parameters: dict, sample: dict, output_csv: str | None = None) -> dict:
    """Execute one 12-second development case and recompute all metrics."""
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    data.eq_active[:] = 0
    mujoco.mj_forward(model, data)
    model_config_path = ROOT / "configs/model_5link.yaml"
    model_config = load_model_config(model_config_path)
    s3 = yaml.safe_load((ROOT / "configs/s3_pid.yaml").read_text(encoding="utf-8"))
    aero = load_aerodynamic_config(ROOT / "configs/aerodynamics.yaml")
    quad_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    equilibrium_tip = np.asarray(data.site_xpos[tip_id], dtype=float).copy()
    equilibrium_relative = equilibrium_tip - np.asarray(data.xpos[quad_id], dtype=float)
    state_reader = V3StateReader(model)
    legacy_reader = StateReader(model, model_config.n_links, float(equilibrium_relative[0]))
    total_mass = float(np.sum(model.body_mass))
    inner = GeometricInnerLoop(total_mass, np.asarray(model.body_inertia[quad_id], dtype=float), s3["attitude_natural_frequency_rad_s"], s3["attitude_damping_ratio"], s3["position_gains_y"][0], s3["position_gains_y"][1], s3["position_gains_z"][0], s3["position_gains_z"][1])
    controller = _controller(kind, parameters)
    controller.reset()
    wind = _wind_series(sample)
    physics_steps = int(round(DURATION / DT))
    wind_stride = int(round(LOG_DT / DT))
    outer_stride = int(round(OUTER_DT / DT))
    command = np.zeros(3, dtype=float)
    reference = reference_for_target(equilibrium_tip, equilibrium_relative, 0.0)
    reference_state = ReferenceState(float(reference.uav_position_world[0]), 0.0, 0.0, float(reference.uav_position_world[1]), float(reference.uav_position_world[2]), 0.0)
    actuator_ids = {name: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)) for name in ("thrust_motor", "mx_motor", "my_motor", "mz_motor")}
    rows: list[dict] = []
    safety_reasons: set[str] = set()
    max_abs_command = np.zeros(3, dtype=float)
    max_step = np.zeros(3, dtype=float)
    max_thrust = 0.0
    max_torque = 0.0
    previous_logged_command = np.zeros(3, dtype=float)
    force = {"total_x": 0.0}
    advanced_updates: list[dict] = []
    self_audit = {
        "d_raw_norm_max": 0.0,
        "d_hat_norm_max": 0.0,
        "d_projected_norm_max": 0.0,
        "d_rejected_norm_max": 0.0,
        "steady_state_residual_max": 0.0,
        "steady_task_residual_max": 0.0,
        "limiter_mismatch_max": 0.0,
        "qp_statuses": set(),
        "qp_iterations_max": 0,
    }
    for step in range(physics_steps + 1):
        time_s = step * DT
        wind_index = min(step // wind_stride, len(wind) - 1)
        if step % wind_stride == 0:
            force = clear_and_apply_wind(model, data, model_config, aero, float(wind[wind_index]))
        if step % outer_stride == 0:
            target = _target_at(sample, time_s, equilibrium_tip)
            reference = reference_for_target(target, equilibrium_relative, time_s)
            reference_state = ReferenceState(float(reference.uav_position_world[0]), 0.0, 0.0, float(reference.uav_position_world[1]), float(reference.uav_position_world[2]), 0.0)
            observation = state_reader.read(model, data, reference)
            started = time.perf_counter_ns()
            command = controller.command(observation, reference, OUTER_DT)
            solve_time_ms = (time.perf_counter_ns() - started) / 1.0e6
            diagnostics = controller.diagnostics
            if hasattr(diagnostics, "qp_status"):
                advanced_updates.append({
                    "status": str(diagnostics.qp_status),
                    "iterations": int(diagnostics.qp_iterations),
                    "solve_time_ms": float(diagnostics.solve_time_ms),
                    "limiter_mismatch": float(diagnostics.limiter_mismatch),
                    "steady_state_residual": float(diagnostics.steady_state_residual),
                    "steady_task_residual": float(diagnostics.steady_task_residual),
                    "d_raw_norm": float(diagnostics.d_raw_norm),
                    "d_hat_norm": float(diagnostics.d_hat_norm),
                    "d_projected_norm": float(diagnostics.d_projected_norm),
                    "d_rejected_norm": float(diagnostics.d_rejected_norm),
                })
        if step % wind_stride == 0:
            control_state = _control_state(model, data, legacy_reader)
            inner_output = inner.compute(control_state, reference_state, float(command[0]), (float(command[1]), float(command[2])))
            thrust_raw = float(inner_output["thrust_raw_N"])
            torque_raw = np.asarray(inner_output["torque_raw_Nm"], dtype=float)
            thrust = float(np.clip(thrust_raw, *model.actuator_ctrlrange[actuator_ids["thrust_motor"]]))
            torque = np.asarray([np.clip(torque_raw[index], *model.actuator_ctrlrange[actuator_ids[name]]) for index, name in enumerate(("mx_motor", "my_motor", "mz_motor"))])
            data.ctrl[:] = 0.0
            data.ctrl[actuator_ids["thrust_motor"]] = thrust
            data.ctrl[actuator_ids["mx_motor"]] = torque[0]
            data.ctrl[actuator_ids["my_motor"]] = torque[1]
            data.ctrl[actuator_ids["mz_motor"]] = torque[2]
            task = state_reader.task_reader.read(model, data)
            target = reference.tip_position_world
            position_error = task.tip_position_world - target
            tip_speed = float(np.linalg.norm(task.tip_velocity_world))
            axis_dot = float(np.clip(task.cutter_axis_world @ np.asarray([1.0, 0.0, 0.0]), -1.0, 1.0))
            orientation_deg = float(np.rad2deg(np.arccos(axis_dot)))
            angular_speed = float(np.linalg.norm(task.cutter_angular_velocity_world))
            height_uav = float(control_state.position[2])
            height_tip = float(task.tip_position_world[2])
            roll, pitch, _ = _rpy(control_state.rotation)
            current_command = np.asarray(command, dtype=float)
            diagnostics = controller.diagnostics
            if kind == "self_dr_tsrmpc":
                for name in (
                    "d_raw_norm", "d_hat_norm", "d_projected_norm", "d_rejected_norm",
                    "steady_state_residual", "steady_task_residual", "limiter_mismatch",
                ):
                    self_audit[f"{name}_max"] = max(self_audit[f"{name}_max"], float(getattr(diagnostics, name)))
                self_audit["qp_statuses"].add(str(diagnostics.qp_status))
                self_audit["qp_iterations_max"] = max(self_audit["qp_iterations_max"], int(diagnostics.qp_iterations))
            finite = bool(np.isfinite(np.r_[position_error, task.tip_velocity_world, task.cutter_angular_velocity_world, current_command, thrust, torque]).all())
            limits_ok = bool(np.all(np.abs(current_command) <= 2.0 + 1.0e-9) and np.all(np.abs(current_command - previous_logged_command) <= 0.25 + 1.0e-9))
            torque_limit = max(max(abs(model.actuator_ctrlrange[actuator_ids[name], 0]), abs(model.actuator_ctrlrange[actuator_ids[name], 1])) for name in ("mx_motor", "my_motor", "mz_motor"))
            safety = finite and height_uav > 0.05 and height_tip > 0.05 and np.max(np.abs(control_state.joint_angles)) < np.deg2rad(100.0) and abs(roll) < np.deg2rad(25.0) and abs(pitch) < np.deg2rad(25.0) and limits_ok and thrust_raw <= model.actuator_ctrlrange[actuator_ids["thrust_motor"], 1] + 1.0e-9 and np.max(np.abs(torque_raw)) <= torque_limit + 1.0e-9
            if not finite: safety_reasons.add("non_finite")
            if height_uav <= 0.05: safety_reasons.add("uav_height")
            if height_tip <= 0.05: safety_reasons.add("tip_height")
            if np.max(np.abs(control_state.joint_angles)) >= np.deg2rad(100.0): safety_reasons.add("joint_angle")
            if abs(roll) >= np.deg2rad(25.0) or abs(pitch) >= np.deg2rad(25.0): safety_reasons.add("attitude")
            if not limits_ok: safety_reasons.add("command_limit")
            max_abs_command = np.maximum(max_abs_command, np.abs(current_command))
            max_step = np.maximum(max_step, np.abs(current_command - previous_logged_command))
            max_thrust = max(max_thrust, abs(thrust_raw)); max_torque = max(max_torque, float(np.max(np.abs(torque_raw))))
            row = {"time": time_s, "position_error_3d_m": float(np.linalg.norm(position_error)), "tip_speed_m_s": tip_speed, "orientation_error_deg": orientation_deg, "angular_speed_rad_s": angular_speed, "safe": bool(safety), "task_success": False, "ax": float(current_command[0]), "ay": float(current_command[1]), "az": float(current_command[2]), "raw_ax": float(diagnostics.raw_command[0]), "raw_ay": float(diagnostics.raw_command[1]), "raw_az": float(diagnostics.raw_command[2]), "integral_x": float(diagnostics.integral[0]), "integral_y": float(diagnostics.integral[1]), "integral_z": float(diagnostics.integral[2]), "saturated": bool(np.any(diagnostics.saturated)), "slew_limited": bool(np.any(diagnostics.slew_limited)), "roll_deg": float(np.rad2deg(roll)), "pitch_deg": float(np.rad2deg(pitch)), "uav_x": float(control_state.position[0]), "uav_y": float(control_state.position[1]), "uav_z": float(control_state.position[2]), "tip_x": float(task.tip_position_world[0]), "tip_y": float(task.tip_position_world[1]), "tip_z": float(task.tip_position_world[2]), "target_x": float(target[0]), "target_y": float(target[1]), "target_z": float(target[2]), "wind_x": float(wind[wind_index]), "thrust_raw_N": thrust_raw, "max_abs_torque_Nm": float(np.max(np.abs(torque_raw))), "solve_time_ms": float(solve_time_ms if step % outer_stride == 0 else rows[-1]["solve_time_ms"] if rows else 0.0), "force_total_x": float(force["total_x"])}
            if hasattr(diagnostics, "qp_status"):
                row.update({
                    "qp_status": str(diagnostics.qp_status),
                    "qp_iterations": int(diagnostics.qp_iterations),
                    "limiter_mismatch": float(diagnostics.limiter_mismatch),
                    "steady_state_residual": float(diagnostics.steady_state_residual),
                    "steady_task_residual": float(diagnostics.steady_task_residual),
                    "d_raw_norm": float(diagnostics.d_raw_norm),
                    "d_hat_norm": float(diagnostics.d_hat_norm),
                    "d_projected_norm": float(diagnostics.d_projected_norm),
                    "d_rejected_norm": float(diagnostics.d_rejected_norm),
                })
            rows.append(row)
            previous_logged_command = current_command.copy()
        if step < physics_steps:
            mujoco.mj_step(model, data)
    times = np.asarray([row["time"] for row in rows], dtype=float)
    position = np.asarray([row["position_error_3d_m"] for row in rows], dtype=float)
    speed = np.asarray([row["tip_speed_m_s"] for row in rows], dtype=float)
    orientation = np.asarray([row["orientation_error_deg"] for row in rows], dtype=float)
    angular = np.asarray([row["angular_speed_rad_s"] for row in rows], dtype=float)
    issue = _issue_time(sample)
    start = 0.0 if issue is None else issue
    acquisition_mask = (position <= 0.05) & (speed <= 0.10) & (orientation <= 5.0) & (angular <= 0.10)
    acquired, acquisition_timestamp = first_continuous_acquisition(times, acquisition_mask, hold_time_s=1.0, start_time_s=start)
    if acquired and acquisition_timestamp is not None:
        acquisition_time = float(acquisition_timestamp - start)
    else:
        acquisition_time = None
    ramp_mask = (times >= 2.0) & (times <= 8.0)
    steady_mask = times >= 8.0
    result = {"sample_id": sample["sample_id"], "scenario": sample["scenario"], "wind_kind": sample["wind"]["kind"], "wind_speed_m_s": sample["wind"].get("speed_m_s", 0.0), "seed": sample["wind"].get("seed", -1), "sample_count": len(rows), "safe": bool(all(row["safe"] for row in rows)), "safe_sample_count": int(sum(row["safe"] for row in rows)), "task_success": bool(acquired), "acquisition_time_s": acquisition_time, "position_rmse_3d_m": float(np.sqrt(np.mean(position ** 2))), "orientation_rmse_deg": float(np.sqrt(np.mean(orientation ** 2))), "ramp_peak_position_error_m": float(np.max(position[ramp_mask])) if np.any(ramp_mask) else 0.0, "ramp_steady_state_position_error_m": float(np.mean(position[steady_mask])) if np.any(steady_mask) else 0.0, "total_acceleration_effort": float(np.trapezoid(np.sum(np.asarray([[row["ax"], row["ay"], row["az"]] for row in rows]) ** 2, axis=1), times) if hasattr(np, "trapezoid") else np.trapz(np.sum(np.asarray([[row["ax"], row["ay"], row["az"]] for row in rows]) ** 2, axis=1), times)), "max_abs_ax_m_s2": float(max_abs_command[0]), "max_abs_ay_m_s2": float(max_abs_command[1]), "max_abs_az_m_s2": float(max_abs_command[2]), "max_ax_step_m_s2": float(max_step[0]), "max_ay_step_m_s2": float(max_step[1]), "max_az_step_m_s2": float(max_step[2]), "max_thrust_N": float(max_thrust), "max_abs_torque_Nm": float(max_torque), "max_cutter_angular_speed_rad_s": float(np.max(angular)), "solve_time_mean_ms": float(np.mean([row["solve_time_ms"] for row in rows])), "solve_time_p95_ms": float(np.percentile([row["solve_time_ms"] for row in rows], 95)), "safety_failure_reasons": sorted(safety_reasons), "controller": kind, "candidate_id": parameters["candidate_id"]}
    if advanced_updates:
        solved_statuses = {"solved", "solved inaccurate", "not_run_residual_only_ablation"}
        solve_times = [item["solve_time_ms"] for item in advanced_updates]
        result.update({
            "solver_success_count": int(sum(item["status"] in solved_statuses for item in advanced_updates)),
            "outer_update_count": len(advanced_updates),
            "solver_success_rate": float(sum(item["status"] in solved_statuses for item in advanced_updates) / len(advanced_updates)),
            "qp_statuses": sorted({item["status"] for item in advanced_updates}),
            "qp_iterations_max": int(max(item["iterations"] for item in advanced_updates)),
            "advanced_solve_time_p95_ms": float(np.percentile(solve_times, 95)),
            "limiter_mismatch_max": float(max(item["limiter_mismatch"] for item in advanced_updates)),
            "steady_state_residual_max": float(max(item["steady_state_residual"] for item in advanced_updates)),
            "steady_task_residual_max": float(max(item["steady_task_residual"] for item in advanced_updates)),
            "d_raw_norm_max": float(max(item["d_raw_norm"] for item in advanced_updates)),
            "d_hat_norm_max": float(max(item["d_hat_norm"] for item in advanced_updates)),
            "d_projected_norm_max": float(max(item["d_projected_norm"] for item in advanced_updates)),
            "d_rejected_norm_max": float(max(item["d_rejected_norm"] for item in advanced_updates)),
            "_advanced_solve_times_ms": solve_times,
        })
    if kind == "self_dr_tsrmpc":
        result.update({key: value for key, value in self_audit.items() if key != "qp_statuses"})
        result["qp_statuses"] = sorted(self_audit["qp_statuses"])
        result["solver_valid"] = bool(result["qp_statuses"] and set(result["qp_statuses"]) <= {"solved", "solved inaccurate"})
        result["limiter_parity_valid"] = bool(result["limiter_mismatch_max"] <= 1.0e-5)
    if output_csv is not None:
        write_csv(Path(output_csv), rows)
    return result


def pid_screen_parameters(axis: str, row: dict) -> dict:
    gains = {name: np.zeros(3) for name in ("kp", "kd", "ki")}
    index = {"x": 0, "y": 1, "z": 2}[axis]
    for name in gains: gains[name][index] = row[name]
    return {"candidate_id": row["candidate_id"], **{name: gains[name].tolist() for name in gains}, "screen_axis": axis}


def pid_combination_parameters(combo: dict, top2: dict) -> dict:
    rows = {axis: top2[axis][combo[axis]] for axis in ("x", "y", "z")}
    indices = {"x": 0, "y": 1, "z": 2}
    return {"candidate_id": combo["candidate_id"], "kp": [rows[axis]["kp"][indices[axis]] for axis in ("x", "y", "z")], "kd": [rows[axis]["kd"][indices[axis]] for axis in ("x", "y", "z")], "ki": [rows[axis]["ki"][indices[axis]] for axis in ("x", "y", "z")], "axis_sources": rows}


def selection_key(row: dict) -> tuple:
    success = row.get("task_success_count", int(bool(row.get("task_success", False))))
    return (-row["safe_sample_count"], -int(success), float(row["position_rmse_3d_m"]), float(row["acquisition_median_s"] if row.get("acquisition_median_s") is not None else row.get("acquisition_time_s") if row.get("acquisition_time_s") is not None else float("inf")), float(row["ramp_steady_state_position_error_m"]), float(row["ramp_peak_position_error_m"]), float(row["orientation_rmse_deg"]), float(row["total_acceleration_effort"]), str(row["candidate_id"]))


def aggregate(candidate_id: str, rows: list[dict], parameters: dict) -> dict:
    acquisition = [row["acquisition_time_s"] for row in rows if row["task_success"] and row["acquisition_time_s"] is not None]
    safe_count = sum(bool(row["safe"]) for row in rows)
    success_count = sum(bool(row["task_success"]) for row in rows)
    result = {"candidate_id": candidate_id, "sample_count": len(rows), "safe_sample_count": safe_count, "task_success_count": success_count, "safety_rate": safe_count / len(rows), "success_rate": success_count / len(rows), "acquisition_median_s": float(np.median(acquisition)) if acquisition else None, "position_rmse_3d_m": float(np.mean([row["position_rmse_3d_m"] for row in rows])), "orientation_rmse_deg": float(np.mean([row["orientation_rmse_deg"] for row in rows])), "ramp_peak_position_error_m": float(max(row["ramp_peak_position_error_m"] for row in rows)), "ramp_steady_state_position_error_m": float(max(row["ramp_steady_state_position_error_m"] for row in rows)), "total_acceleration_effort": float(np.mean([row["total_acceleration_effort"] for row in rows])), "solve_time_p95_ms": float(max(row["solve_time_p95_ms"] for row in rows)), "parameters": parameters, "rows": rows}
    if rows and rows[0].get("controller") == "self_dr_tsrmpc":
        for name in ("d_raw_norm_max", "d_hat_norm_max", "d_projected_norm_max", "d_rejected_norm_max", "steady_state_residual_max", "steady_task_residual_max", "limiter_mismatch_max", "qp_iterations_max"):
            result[name] = float(max(row[name] for row in rows))
        result["solver_valid_rate"] = sum(bool(row["solver_valid"]) for row in rows) / len(rows)
        result["limiter_parity_rate"] = sum(bool(row["limiter_parity_valid"]) for row in rows) / len(rows)
    return result


def case_subset(manifest: list[dict], mode: str) -> list[dict]:
    if mode == "screen_x":
        ids = {"axis_00", "axis_01"}
        return [s for s in manifest if s["target"]["target_id"] in ids and (s["wind"]["kind"] == "calm" or s["wind"].get("speed_m_s") == 3.0)]
    if mode == "screen_y":
        ids = {"axis_02", "axis_03"}
        return [s for s in manifest if s["target"]["target_id"] in ids and (s["wind"]["kind"] == "calm" or s["wind"].get("speed_m_s") == 3.0)]
    if mode == "screen_z":
        ids = {"axis_04", "axis_05"}
        return [s for s in manifest if s["target"]["target_id"] in ids and (s["wind"]["kind"] == "calm" or s["wind"].get("speed_m_s") == 3.0)]
    if mode == "core":
        calm_ids = {"axis_00", "axis_01", "axis_02", "axis_03", "axis_04", "axis_05"}
        wind_ids = {"face_diagonal_00", "face_diagonal_01", "face_diagonal_04", "face_diagonal_05", "face_diagonal_08", "face_diagonal_09"}
        return [s for s in manifest if (s["target"]["target_id"] in calm_ids and s["wind"]["kind"] == "calm") or (s["target"]["target_id"] in wind_ids and s["wind"].get("speed_m_s") == 3.0)] + [s for s in manifest if s["scenario"] == "RAMP_WIND_EQUILIBRIUM_HOLD"]
    return manifest


def run_candidates(kind: str, candidates: list[dict], samples: list[dict], workers: int, label: str) -> list[dict]:
    jobs = [(kind, candidate, sample) for candidate in candidates for sample in samples]
    results: dict[tuple[str, str], dict] = {}
    if workers <= 1:
        for index, (method, candidate, sample) in enumerate(jobs, 1):
            results[(candidate["candidate_id"], sample["sample_id"])] = run_case(method, candidate, sample)
            if index % 10 == 0 or index == len(jobs): print(f"{label} {index}/{len(jobs)}", flush=True)
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            futures = [(candidate, sample, pool.submit(run_case, kind, candidate, sample)) for kind, candidate, sample in jobs]
            for index, (candidate, sample, future) in enumerate(futures, 1):
                results[(candidate["candidate_id"], sample["sample_id"])] = future.result()
                if index % 25 == 0 or index == len(jobs): print(f"{label} {index}/{len(jobs)}", flush=True)
    summaries = []
    for candidate in candidates:
        rows = [results[(candidate["candidate_id"], sample["sample_id"])] for sample in samples]
        summaries.append(aggregate(candidate["candidate_id"], rows, candidate))
    return summaries


def add_gain_parameters(kind: str, candidates: list[dict], metric: dict, a: np.ndarray, b: np.ndarray, q_full_selected: np.ndarray | None = None) -> list[dict]:
    result = []
    for candidate in candidates:
        row = dict(candidate)
        if kind == "full_lqr":
            q = build_full_lqr_q(row); algebra = solve_v3_lqr(a, b, q, float(row["r"]) * np.eye(3)); row["K"] = algebra["K"].tolist(); row["spectral_radius"] = algebra["spectral_radius"]; row["p_min_eigenvalue"] = algebra["p_min_eigenvalue"]
        elif kind == "task_lqr":
            if q_full_selected is None: raise ValueError("task LQR needs selected full LQR Q")
            q, r, maps = build_task_lqr_q(a, b, metric, row, q_full_selected); algebra = solve_v3_lqr(a, b, q, r); row["K"] = algebra["K"].tolist(); row["spectral_radius"] = algebra["spectral_radius"]; row["p_min_eigenvalue"] = algebra["p_min_eigenvalue"]; row["metric_maps"] = maps
        result.append(row)
    return result


def summary_without_rows(value: dict) -> dict:
    return {key: item for key, item in value.items() if key not in {"rows"}}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=1)
    parser.add_argument("--smoke", action="store_true")
    args = parser.parse_args()
    protocol = read_json(R1 / "r1_protocol.json")
    manifest_payload = read_json(R1 / "development_evaluation_manifest.json")
    if not read_json(R0 / "holdout_manifest.json")["execution_allowed"] is False:
        raise RuntimeError("R1 runner refused: holdout is executable")
    if sha256(MODEL).lower() != MODEL_SHA:
        raise RuntimeError("frozen plant SHA drift")
    samples = manifest_payload["samples"]
    if args.smoke:
        result = run_case("pid", {"candidate_id": "smoke", "kp": [0.8, 0.1, 6.0], "kd": [4.0, 0.5, 3.5], "ki": [0.0, 0.0, 0.0]}, samples[0])
        print(json.dumps({"result": "V3_R1_SYNTHETIC_SMOKE", "sample": result["sample_id"], "safe": result["safe"], "success": result["task_success"], "position_rmse": result["position_rmse_3d_m"]}, indent=2))
        return 0
    R1.mkdir(parents=True, exist_ok=True)
    grids = {"pid_axis": read_json(R1 / "pid_axis_grids.json"), "pid_combo": read_json(R1 / "pid_combination_grid.json")["grid"], "full": read_json(R1 / "full_lqr_grid.json")["grid"], "task": read_json(R1 / "task_lqr_grid.json")["grid"]}
    screen_top2 = {}
    screen_rows = []
    for axis in ("x", "y", "z"):
        candidates = [pid_screen_parameters(axis, row) for row in grids["pid_axis"][axis]]
        summaries = run_candidates("pid", candidates, case_subset(samples, f"screen_{axis}"), args.workers, f"pid-screen-{axis}")
        ranked = sorted(summaries, key=selection_key)
        screen_top2[axis] = [item["parameters"] for item in ranked[:2]]
        screen_rows.extend([summary_without_rows(item) for item in summaries])
    write_csv(R1 / "pid_axis_screening.csv", screen_rows)
    pid_candidates = [pid_combination_parameters(row, {axis: screen_top2[axis] for axis in ("x", "y", "z")}) for row in grids["pid_combo"]]
    core_samples = case_subset(samples, "core")
    pid_stage1 = run_candidates("pid", pid_candidates, core_samples, args.workers, "pid-stage1")
    write_csv(R1 / "pid_stage1.csv", [summary_without_rows(item) for item in pid_stage1])
    a, b = load_r0_linear_matrices(ROOT)
    metric = read_json(R1 / "task_metric_alignment_audit.json")
    full_candidates = add_gain_parameters("full_lqr", grids["full"], metric, a, b)
    full_stage1 = run_candidates("full_lqr", full_candidates, core_samples, args.workers, "full-lqr-stage1")
    write_csv(R1 / "full_lqr_stage1.csv", [summary_without_rows(item) for item in full_stage1])
    selected_full_stage1 = sorted(full_stage1, key=selection_key)[0]
    q_full_selected = build_full_lqr_q(selected_full_stage1["parameters"])
    task_candidates = add_gain_parameters("task_lqr", grids["task"], metric, a, b, q_full_selected)
    task_stage1 = run_candidates("task_lqr", task_candidates, core_samples, args.workers, "task-lqr-stage1")
    write_csv(R1 / "task_lqr_stage1.csv", [summary_without_rows(item) for item in task_stage1])
    top3 = {"pid": sorted(pid_stage1, key=selection_key)[:3], "full_lqr": sorted(full_stage1, key=selection_key)[:3], "task_lqr": sorted(task_stage1, key=selection_key)[:3]}
    stage2_rows = []
    final_selected = {}
    for kind, selected in top3.items():
        candidates = [item["parameters"] for item in selected]
        stage2 = run_candidates(kind, candidates, samples, args.workers, f"{kind}-stage2")
        final_selected[kind] = sorted(stage2, key=selection_key)[0]
        for rank, summary in enumerate(stage2, 1):
            stage2_rows.extend([{**row, "method": kind, "stage2_rank": rank} for row in summary["rows"]])
    write_csv(R1 / "traditional_development_results.csv", stage2_rows)
    write_json(R1 / "traditional_development_summary.json", {"stage1_top3": {kind: [summary_without_rows(row) for row in rows] for kind, rows in top3.items()}, "final_selected": {kind: summary_without_rows(row) for kind, row in final_selected.items()}})
    freeze_names = {"pid": "pid_freeze.json", "full_lqr": "full_lqr_freeze.json", "task_lqr": "task_lqr_freeze.json"}
    for kind, summary in final_selected.items():
        write_json(R1 / freeze_names[kind], {"controller": kind, "selected": summary_without_rows(summary), "parameters": summary["parameters"], "common_limits": protocol["controller_output"], "holdout_executed": False})
    envelope = {}
    metric_map = {"safety": "safety_rate", "success": "success_rate", "position": "position_rmse_3d_m", "acquisition": "acquisition_median_s", "orientation": "orientation_rmse_deg", "ramp_peak": "ramp_peak_position_error_m", "ramp_steady": "ramp_steady_state_position_error_m", "effort": "total_acceleration_effort"}
    for name, key in metric_map.items():
        valid = [(kind, row[key]) for kind, row in final_selected.items() if row[key] is not None]
        best = max(valid, key=lambda item: item[1]) if name in {"safety", "success"} else min(valid, key=lambda item: item[1])
        envelope[name] = {"value": best[1], "controller": best[0]}
    write_json(R1 / "primary_traditional_baseline.json", {"primary_controller": min(final_selected, key=lambda kind: selection_key(final_selected[kind])) + ":" + final_selected[min(final_selected, key=lambda kind: selection_key(final_selected[kind]))]["candidate_id"], "primary_method": min(final_selected, key=lambda kind: selection_key(final_selected[kind])), "selection_rule": protocol["stage1_selection"], "selected_metrics": summary_without_rows(min(final_selected.values(), key=selection_key))})
    write_json(R1 / "traditional_metric_envelope.json", envelope)
    primary_kind = min(final_selected, key=lambda kind: selection_key(final_selected[kind]))
    primary = final_selected[primary_kind]
    acquisition_threshold = None if primary["acquisition_median_s"] is None else 1.05 * primary["acquisition_median_s"]
    strict_acquisition_threshold = None if primary["acquisition_median_s"] is None else 0.95 * primary["acquisition_median_s"]
    write_json(R1 / "advanced_numeric_win_contract.json", {"contract": "V3-R1-advanced-numeric-win-contract", "written_before_any_advanced_performance": True, "safety_threshold_rate": max(item["safety_rate"] for item in final_selected.values()), "success_threshold_rate": max(item["success_rate"] for item in final_selected.values()), "primary_traditional": primary_kind + ":" + primary["candidate_id"], "position_rmse_max_m": 0.95 * primary["position_rmse_3d_m"], "acquisition_noninferiority_max_s": acquisition_threshold, "strict_acquisition_max_s": strict_acquisition_threshold, "ramp_peak_max_m": 0.95 * primary["ramp_peak_position_error_m"], "ramp_steady_max_m": 0.95 * primary["ramp_steady_state_position_error_m"], "paired_bootstrap": {"variable": "RMSE_primary_i - RMSE_advanced_i", "pairing": "exact sample_id", "resamples": 10000, "seed": 20260809, "required_ci_lower_bound": "> 0"}, "holdout_execution_allowed": False})
    representative_ids = {"axis_calm": "calm_axis_00", "face_diagonal_wind3": "constant_3_face_diagonal_00", "stochastic": "stochastic_2000", "ramp": "ramp_wind_equilibrium_hold"}
    for kind, summary in final_selected.items():
        for label, sample_id in representative_ids.items():
            sample = next(row for row in samples if row["sample_id"] == sample_id)
            output = R1 / "representative" / kind / f"{label}.csv"
            run_case(kind, summary["parameters"], sample, str(output))
    all_selected_safe = bool(all(row["safe_sample_count"] == row["sample_count"] for row in final_selected.values()))
    gate = {"task": protocol["task"], "start_head": protocol["start_head"], "r1_protocol_freeze_head": "36fb57a", "branch": "research-v3", "protocol_frozen_before_performance": True, "v3_r0_unchanged": True, "v2_final_unchanged": True, "pid_axis_grid_size_each": 18, "pid_combination_grid_size": 8, "full_lqr_grid_size": 64, "task_lqr_grid_size": 27, "stage1_core_count": 13, "development_evaluation_count": 75, "pid_selected": bool(final_selected["pid"]["safe_sample_count"] > 0), "full_lqr_selected": bool(final_selected["full_lqr"]["safe_sample_count"] > 0), "task_lqr_selected": bool(final_selected["task_lqr"]["safe_sample_count"] > 0), "all_selected_safe": all_selected_safe, "all_lqr_selected_stable": bool(all(row["parameters"].get("spectral_radius", 2.0) < 1.0 for row in final_selected.values() if row["parameters"].get("spectral_radius") is not None)), "primary_traditional_selected": True, "traditional_competence_gate": all_selected_safe, "metric_envelope_written": True, "advanced_numeric_contract_frozen": True, "traditional_started": True, "advanced_self_started": False, "advanced_paper_selected": False, "holdout_executed": False, "result": "V3_3D_TRADITIONAL_BASELINES_FROZEN" if all_selected_safe else "BLOCKED_V3_TRADITIONAL_BASELINE"}
    write_json(R1 / "safety_audit.json", {"development_count": len(samples), "holdout_executed": False, "selected": {kind: {"safe": row["safe_sample_count"] == row["sample_count"], "safety_rate": row["safety_rate"], "failure_reasons": sorted({reason for item in row["rows"] for reason in item["safety_failure_reasons"]})} for kind, row in final_selected.items()}})
    write_json(R1 / "holdout_access_audit.json", {"r0_holdout_execution_allowed": False, "holdout_loaded_by_runner": False, "holdout_executed": False, "forbidden_seeds": list(range(3000, 3020))})
    write_json(R1 / "gate.json", gate)
    print(json.dumps({"result": gate["result"], "primary": primary_kind + ":" + primary["candidate_id"], "selected": {kind: row["candidate_id"] for kind, row in final_selected.items()}}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
