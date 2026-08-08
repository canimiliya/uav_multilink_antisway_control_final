"""Closed-loop development runner for S6T1 Task-PID and Task-LQR."""

from __future__ import annotations

import csv
import hashlib
import json
import time as wall_time
from pathlib import Path

import mujoco
import numpy as np
import yaml

from uav_sway.control.base import ReferenceState
from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.control.runtime_model import create_runtime_model
from uav_sway.control.state_reader import StateReader
from uav_sway.control.task_lqr import TaskLQR
from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind
from uav_sway.disturbances.wind_io import read_wind_csv
from uav_sway.evaluation.controlled_schema import controlled_schema_columns
from uav_sway.evaluation.task_space_metrics import compute_task_metrics
from uav_sway.linearization.reduced_state import ReducedStateLayout
from uav_sway.models.model_config import load_model_config
from uav_sway.scenarios.scenario_config import load_scenario_config
from uav_sway.task_space.reference import build_equilibrium_task_pose, task_reference_at
from uav_sway.models.state_io import capture_state
from uav_sway.linearization.task_output import TaskOutputMap


ROOT = Path(__file__).resolve().parents[3]
TASK_COLUMNS = [
    "reference_event", "tip_velocity_world_x", "tip_velocity_world_y", "tip_velocity_world_z",
    "cutter_axis_world_x", "cutter_axis_world_y", "cutter_axis_world_z",
    "task_reference_tip_x", "task_reference_tip_y", "task_reference_tip_z",
    "task_reference_axis_x", "task_reference_axis_y", "task_reference_axis_z",
    "tip_task_x_error_m", "tip_task_z_error_m", "task_position_error_xz_m", "position_error_3d_m",
    "tip_speed_m_s", "orientation_error_rad", "orientation_error_deg",
]
TASK_CONTROLLER_COLUMNS = [
    "task_position_error_x", "task_velocity_error_x", "theta_cutter_signed_rad", "omega_cutter_y",
    "task_pid_integral_x", "task_lqr_feedback_ax", "task_lqr_state_norm",
]


def sha256_file(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def sha256_json(value: dict) -> str:
    def encode(item):
        if isinstance(item, np.ndarray):
            return item.tolist()
        if isinstance(item, np.generic):
            return item.item()
        raise TypeError(type(item).__name__)
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"), default=encode).encode("utf-8")).hexdigest()


def read_reference(path: str | Path) -> dict[str, np.ndarray]:
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("empty reference")
    result = {key: np.asarray([float(row[key]) for row in rows], dtype=float) for key in ("time", "x_ref", "vx_ref", "ax_ref", "y_ref", "z_ref", "yaw_ref")}
    result["event"] = np.asarray([row["event"] for row in rows], dtype=object)
    return result


def reference_at(ref: dict[str, np.ndarray], index: int) -> ReferenceState:
    return ReferenceState(*(float(ref[name][index]) for name in ("x_ref", "vx_ref", "ax_ref", "y_ref", "z_ref", "yaw_ref")))


def _id(model, object_type, name: str) -> int:
    value = int(mujoco.mj_name2id(model, object_type, name))
    if value < 0:
        raise KeyError(name)
    return value


def _rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    return (float(np.arctan2(rotation[2, 1], rotation[2, 2])), float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))), float(np.arctan2(rotation[1, 0], rotation[0, 0])))


def _write_csv(path: Path, rows: list[dict], columns: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            encoded = {}
            for key in columns:
                value = row[key]
                if isinstance(value, (bool, np.bool_)):
                    encoded[key] = "true" if value else "false"
                elif isinstance(value, str):
                    encoded[key] = value
                else:
                    encoded[key] = format(float(value), ".17g")
            writer.writerow(encoded)


def make_task_output_map(root: str | Path = ROOT) -> tuple[mujoco.MjModel, mujoco.MjData, TaskOutputMap, Path]:
    root = Path(root)
    runtime_xml = root / "reproducibility/frozen/model/model_5link_controlled.xml"
    model = mujoco.MjModel.from_xml_path(str(runtime_xml))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    data.eq_active[:] = 0
    mujoco.mj_forward(model, data)
    pose = build_equilibrium_task_pose(model, data, runtime_xml)
    return model, data, TaskOutputMap(model, capture_state(model, data), pose), runtime_xml


def run_task_baseline_scenario(model_config_path: str | Path, controller_kind: str,
                               candidate: dict, scenario: str, wind_csv: str | Path,
                               reference_csv: str | Path, output_csv: str | Path,
                               root: str | Path = ROOT, duration_s: float = 12.0,
                               c_task: np.ndarray | None = None) -> dict:
    root = Path(root)
    model, data, task_map, runtime_xml = make_task_output_map(root)
    s3 = yaml.safe_load((root / "configs/s3_pid.yaml").read_text(encoding="utf-8"))
    model_cfg = load_model_config(model_config_path)
    scenario_cfg = load_scenario_config(root / "configs/scenarios.yaml")
    aero = load_aerodynamic_config(root / "configs/aerodynamics.yaml")
    wind = read_wind_csv(wind_csv)
    ref = read_reference(reference_csv)
    if len(wind["time"]) != len(ref["time"]):
        raise ValueError("wind/reference sample counts differ")
    quad_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    tip_id = _id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip")
    cutter_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "cutter")
    equilibrium_relative_x = float(data.site_xpos[tip_id, 0] - data.xpos[quad_id, 0])
    reader = StateReader(model, model_cfg.n_links, equilibrium_relative_x)
    layout = ReducedStateLayout(model)
    total_mass = float(np.sum(model.body_mass))
    inner = GeometricInnerLoop(total_mass, np.asarray(model.body_inertia[quad_id], dtype=float), s3["attitude_natural_frequency_rad_s"], s3["attitude_damping_ratio"], *s3["position_gains_y"], *s3["position_gains_z"])
    if controller_kind == "task_lqr":
        if c_task is None:
            raise ValueError("Task-LQR requires C_task")
        controller = TaskLQR(np.asarray(candidate["K"], dtype=float), -2.0, 2.0, 0.25)
    else:
        raise ValueError(controller_kind)
    controller.reset()
    actuator_ids = {name: _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ("rotor_motor_0", "rotor_motor_1", "rotor_motor_2", "rotor_motor_3", "thrust_motor", "mx_motor", "my_motor", "mz_motor")}
    physics_dt = float(model.opt.timestep)
    signal_steps = int(round(float(s3["wind_dt_s"]) / physics_dt))
    outer_steps = int(round(float(s3["outer_dt_s"]) / physics_dt))
    physics_steps = int(round(duration_s / physics_dt))
    rows: list[dict] = []
    last_inner = {"thrust_raw_N": total_mass * 9.81, "torque_raw_Nm": np.zeros(3)}
    last_limited = {"thrust": total_mass * 9.81, "torque": np.zeros(3)}
    last_solve_ms = 0.0
    force = {"quadrotor_x": 0.0, "cutter_x": 0.0, "total_x": 0.0, **{f"link_{i}_x": 0.0 for i in range(1, model_cfg.n_links + 1)}}
    outer_calls = inner_calls = log_calls = wind_calls = 0
    for step in range(physics_steps + 1):
        index = min(step // signal_steps, len(wind["time"]) - 1)
        force = clear_and_apply_wind(model, data, model_cfg, aero, float(wind["wind_x"][index])); wind_calls += 1
        reference = reference_at(ref, index)
        if step % outer_steps == 0:
            task_y = task_map.from_mujoco(data, reference)
            started = wall_time.perf_counter_ns()
            controller.command(layout.extract(model, data, reference), reference, 0.05)
            last_solve_ms = (wall_time.perf_counter_ns() - started) / 1.0e6; outer_calls += 1
        if step % signal_steps == 0:
            control_state = reader.read(model, data)
            task_state = task_map.from_mujoco(data, reference)
            task_reference = task_reference_at({key: float(ref[key][index]) for key in ("x_ref", "y_ref", "z_ref")}, task_map.pose)
            last_inner = inner.compute(control_state, reference, controller.diagnostics.ax_cmd_limited)
            thrust_raw = float(last_inner["thrust_raw_N"]); torque_raw = np.asarray(last_inner["torque_raw_Nm"], dtype=float)
            thrust_lim = float(np.clip(thrust_raw, *model.actuator_ctrlrange[actuator_ids["thrust_motor"]]))
            torque_lim = np.asarray([np.clip(torque_raw[i], *model.actuator_ctrlrange[actuator_ids[name]]) for i, name in enumerate(("mx_motor", "my_motor", "mz_motor"))])
            last_limited = {"thrust": thrust_lim, "torque": torque_lim}
            data.ctrl[:] = 0.0; data.ctrl[actuator_ids["thrust_motor"]] = thrust_lim
            for i, name in enumerate(("mx_motor", "my_motor", "mz_motor")): data.ctrl[actuator_ids[name]] = torque_lim[i]
            roll, pitch, yaw = _rpy(control_state.rotation)
            tip_position, tip_velocity, axis_world, angular_velocity = task_map.kinematics_from_mujoco(data)
            position_error = np.asarray([data.site_xpos[tip_id, j] for j in range(3)]) - task_reference.tip_position_world
            dot = float(np.clip(np.dot(axis_world, task_reference.cutter_axis_world), -1.0, 1.0))
            diag = controller.diagnostics
            base = {
                "time": float(wind["time"][index]), "scenario": scenario, "seed": -1 if wind["seed"] is None else int(wind["seed"]), "protocol_mode": "free_flight_controlled", "wind_x": float(wind["wind_x"][index]), "wind_y": 0.0, "wind_z": 0.0,
                "x_ref": reference.x_ref, "vx_ref": reference.vx_ref, "ax_ref": reference.ax_ref, "y_ref": reference.y_ref, "z_ref": reference.z_ref, "yaw_ref": reference.yaw_ref, "reference_event": str(ref["event"][index]),
                "uav_x": control_state.position[0], "uav_y": control_state.position[1], "uav_z": control_state.position[2], "uav_vx": control_state.velocity[0], "uav_vy": control_state.velocity[1], "uav_vz": control_state.velocity[2],
                "uav_qw": data.xquat[quad_id, 0], "uav_qx": data.xquat[quad_id, 1], "uav_qy": data.xquat[quad_id, 2], "uav_qz": data.xquat[quad_id, 3],
                **{f"joint_{i}_angle": control_state.joint_angles[i - 1] for i in range(1, model_cfg.n_links + 1)}, **{f"joint_{i}_velocity": control_state.joint_velocities[i - 1] for i in range(1, model_cfg.n_links + 1)},
                "tip_x": data.site_xpos[tip_id, 0], "tip_y": data.site_xpos[tip_id, 1], "tip_z": data.site_xpos[tip_id, 2], "tip_relative_x": data.site_xpos[tip_id, 0] - control_state.position[0], "tip_equilibrium_relative_x": equilibrium_relative_x, "tip_displacement": control_state.tip_displacement,
                "wind_force_quad_x": force["quadrotor_x"], **{f"wind_force_link_{i}_x": force[f"link_{i}_x"] for i in range(1, model_cfg.n_links + 1)}, "wind_force_cutter_x": force["cutter_x"], "wind_force_total_x": force["total_x"],
                "ax_cmd_raw": diag.ax_cmd_raw, "ax_cmd_limited": diag.ax_cmd_limited, "ax_saturated": diag.ax_saturated, "solve_time_ms": last_solve_ms,
                "controller": controller_kind, "anchor_active": False, "position_error_x": task_state[0], "velocity_error_x": task_state[1], "pid_integral_x": getattr(diag, "pid_integral_x", 0.0),
                "ax_reference_feedforward": diag.ax_reference_feedforward, "ax_pid_feedback": getattr(diag, "ax_pid_feedback", 0.0), "ax_cmd_amplitude_limited": diag.ax_cmd_amplitude_limited, "ax_slew_limited": diag.ax_slew_limited,
                "thrust_cmd_raw_N": thrust_raw, "thrust_cmd_limited_N": thrust_lim, "mx_cmd_raw_Nm": torque_raw[0], "my_cmd_raw_Nm": torque_raw[1], "mz_cmd_raw_Nm": torque_raw[2], "mx_cmd_limited_Nm": torque_lim[0], "my_cmd_limited_Nm": torque_lim[1], "mz_cmd_limited_Nm": torque_lim[2], "inner_loop_saturated": bool(thrust_raw != thrust_lim or np.any(torque_raw != torque_lim)), "roll_rad": roll, "pitch_rad": pitch, "yaw_rad": yaw,
                "task_position_error_x": task_state[0], "task_velocity_error_x": task_state[1], "theta_cutter_signed_rad": task_state[2], "omega_cutter_y": task_state[3], "task_pid_integral_x": getattr(diag, "pid_integral_x", 0.0), "task_lqr_feedback_ax": getattr(diag, "lqr_feedback_ax", 0.0), "task_lqr_state_norm": getattr(diag, "lqr_state_norm", 0.0),
                "tip_velocity_world_x": tip_velocity[0], "tip_velocity_world_y": tip_velocity[1], "tip_velocity_world_z": tip_velocity[2],
                "cutter_axis_world_x": axis_world[0], "cutter_axis_world_y": axis_world[1], "cutter_axis_world_z": axis_world[2], "task_reference_tip_x": task_reference.tip_position_world[0], "task_reference_tip_y": task_reference.tip_position_world[1], "task_reference_tip_z": task_reference.tip_position_world[2], "task_reference_axis_x": task_reference.cutter_axis_world[0], "task_reference_axis_y": task_reference.cutter_axis_world[1], "task_reference_axis_z": task_reference.cutter_axis_world[2],
                "tip_task_x_error_m": position_error[0], "tip_task_z_error_m": position_error[2], "task_position_error_xz_m": np.linalg.norm(position_error[[0, 2]]), "position_error_3d_m": np.linalg.norm(position_error), "tip_speed_m_s": np.linalg.norm(tip_velocity - np.asarray([reference.vx_ref, 0.0, 0.0])), "orientation_error_rad": np.arccos(dot), "orientation_error_deg": np.rad2deg(np.arccos(dot)),
            }
            rows.append(base); inner_calls += 1; log_calls += 1
        if step < physics_steps:
            mujoco.mj_step(model, data)
    output = Path(output_csv)
    columns = controlled_schema_columns(model_cfg.n_links) + TASK_CONTROLLER_COLUMNS + TASK_COLUMNS
    _write_csv(output, rows, columns)
    metrics = compute_task_metrics(output)
    metrics.update({
        "controller": controller_kind, "candidate_id": candidate["candidate_id"], "physics_intervals": physics_steps,
        "formal_log_samples": log_calls, "outer_control_updates": outer_calls, "inner_loop_updates": inner_calls,
        "wind_force_calls": wind_calls, "anchor_active": False, "runtime_model_sha256": sha256_file(runtime_xml),
        "physics_dt": physics_dt, "inner_dt": float(s3["wind_dt_s"]), "outer_dt": float(s3["outer_dt_s"]),
        "sample_count": log_calls, "wind_sha256": sha256_file(wind_csv), "reference_sha256": sha256_file(reference_csv),
        "controller_config_sha256": sha256_json({key: value for key, value in candidate.items() if key != "K"}),
        "controller_config": {key: value for key, value in candidate.items() if key != "K"},
        "rotor_motor_max_abs_cmd": 0.0, "scheduled_physics_hz": 1000, "scheduled_inner_hz": 200, "scheduled_outer_hz": 20,
        "scenario_set": ["approach_stop", "crosswind_hover"], "provenance_complete": True,
        "settling_start_s": float(scenario_cfg[scenario]["settling_start_s"]),
    })
    return metrics
