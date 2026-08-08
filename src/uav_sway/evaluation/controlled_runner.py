"""Free-flight S3 PID runner with frozen 1 kHz/200 Hz/20 Hz scheduling."""

from __future__ import annotations

import csv
import json
import time as wall_time
from pathlib import Path

import mujoco
import numpy as np
import yaml

from uav_sway.control.base import ReferenceState
from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.control.position_pid import PositionPID
from uav_sway.control.runtime_model import create_runtime_model
from uav_sway.control.state_reader import StateReader
from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind
from uav_sway.disturbances.wind_io import read_wind_csv
from uav_sway.evaluation.controlled_metrics import compute_controlled_metrics
from uav_sway.evaluation.controlled_schema import controlled_schema_columns
from uav_sway.scenarios.scenario_config import load_scenario_config


def _read_reference(path: str | Path) -> dict[str, np.ndarray]:
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("empty reference")
    result = {key: np.asarray([float(row[key]) for row in rows], dtype=float) for key in ("time", "x_ref", "vx_ref", "ax_ref", "y_ref", "z_ref", "yaw_ref")}
    result["event"] = np.asarray([row["event"] for row in rows], dtype=object)
    return result


def _reference_at(ref: dict[str, np.ndarray], index: int) -> ReferenceState:
    return ReferenceState(*(float(ref[name][index]) for name in ("x_ref", "vx_ref", "ax_ref", "y_ref", "z_ref", "yaw_ref")))


def _id(model, object_type, name: str) -> int:
    value = int(mujoco.mj_name2id(model, object_type, name))
    if value < 0:
        raise KeyError(name)
    return value


def _write_csv(path: Path, rows: list[dict], columns: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: ("true" if isinstance(row[key], (bool, np.bool_)) and row[key] else "false" if isinstance(row[key], (bool, np.bool_)) else row[key] if isinstance(row[key], str) else format(float(row[key]), ".17g")) for key in columns})


def _rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    roll = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
    pitch = float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0)))
    yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    return roll, pitch, yaw


def ensure_calm_wind(path: str | Path, time_values: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        stream.write("time,wind_x,wind_y,wind_z,profile,seed\n")
        for t in time_values:
            stream.write(f"{format(float(t), '.17g')},0,0,0,calm,\n")


def run_controlled_scenario(model_config_path: str | Path, controller_config: dict, scenario: str, wind_csv: str | Path, reference_csv: str | Path, output_csv: str | Path, repo_root: str | Path | None = None, headless: bool = True, duration_s: float = 12.0) -> dict:
    del headless
    repo_root = Path(repo_root or Path(model_config_path).resolve().parents[1])
    runtime_xml = repo_root / "reproducibility/frozen/model/model_5link_controlled.xml"
    model = mujoco.MjModel.from_xml_path(str(runtime_xml))
    s3_config = yaml.safe_load((repo_root / "configs/s3_pid.yaml").read_text(encoding="utf-8"))
    if not np.isclose(model.opt.timestep, float(s3_config["physics_dt_s"])):
        raise ValueError("S3 physics dt does not match the frozen runtime protocol")
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    data.eq_active[:] = 0
    mujoco.mj_forward(model, data)

    from uav_sway.models.model_config import load_model_config
    model_cfg = load_model_config(model_config_path)
    wind = read_wind_csv(wind_csv)
    ref = _read_reference(reference_csv)
    if len(wind["time"]) != len(ref["time"]):
        raise ValueError("wind and reference sample counts differ")
    scenario_cfg = load_scenario_config(repo_root / "configs/scenarios.yaml")
    aero = load_aerodynamic_config(repo_root / "configs/aerodynamics.yaml")
    quad_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    tip_id = _id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip")
    equilibrium_relative_x = float(data.site_xpos[tip_id, 0] - data.xpos[quad_id, 0])
    reader = StateReader(model, model_cfg.n_links, equilibrium_relative_x)
    total_mass = float(np.sum(model.body_mass))
    inertia = np.asarray(model.body_inertia[quad_id], dtype=float)
    inner = GeometricInnerLoop(total_mass, inertia, s3_config["attitude_natural_frequency_rad_s"], s3_config["attitude_damping_ratio"], s3_config["position_gains_y"][0], s3_config["position_gains_y"][1], s3_config["position_gains_z"][0], s3_config["position_gains_z"][1])
    pid = PositionPID(controller_config["kp"], controller_config["kd"], controller_config["ki"], controller_config.get("ax_min_m_s2", -2.0), controller_config.get("ax_max_m_s2", 2.0), controller_config.get("delta_ax_max_per_outer_step", 0.25), controller_config.get("integral_error_limit_m_s", 1.0))
    state = reader.read(model, data)
    pid.reset(state, _reference_at(ref, 0))

    actuator_ids = {name: _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ("rotor_motor_0", "rotor_motor_1", "rotor_motor_2", "rotor_motor_3", "thrust_motor", "mx_motor", "my_motor", "mz_motor")}
    physics_dt = float(model.opt.timestep)
    signal_steps = int(round(float(s3_config["wind_dt_s"]) / physics_dt))
    outer_steps = int(round(float(s3_config["outer_dt_s"]) / physics_dt))
    physics_steps = int(round(duration_s / physics_dt))
    rows: list[dict] = []
    last_inner = {"thrust_raw_N": total_mass * 9.81, "torque_raw_Nm": np.zeros(3)}
    last_limited = {"thrust": total_mass * 9.81, "torque": np.zeros(3)}
    last_pid_time_ms = 0.0
    force = {"quadrotor_x": 0.0, "cutter_x": 0.0, "total_x": 0.0, **{f"link_{i}_x": 0.0 for i in range(1, model_cfg.n_links + 1)}}
    outer_calls = inner_calls = log_calls = wind_calls = 0
    for step in range(physics_steps + 1):
        index = min(step // signal_steps, len(wind["time"]) - 1)
        force = clear_and_apply_wind(model, data, model_cfg, aero, float(wind["wind_x"][index]))
        wind_calls += 1
        reference = _reference_at(ref, index)
        if step % outer_steps == 0:
            state = reader.read(model, data)
            started_ns = wall_time.perf_counter_ns()
            pid.command(state, reference, 0.05)
            last_pid_time_ms = (wall_time.perf_counter_ns() - started_ns) / 1.0e6
            outer_calls += 1
        if step % signal_steps == 0:
            state = reader.read(model, data)
            last_inner = inner.compute(state, reference, pid.diagnostics.ax_cmd_limited)
            thrust_raw = float(last_inner["thrust_raw_N"])
            torque_raw = np.asarray(last_inner["torque_raw_Nm"], dtype=float)
            thrust_limited = float(np.clip(thrust_raw, model.actuator_ctrlrange[actuator_ids["thrust_motor"], 0], model.actuator_ctrlrange[actuator_ids["thrust_motor"], 1]))
            torque_limited = np.asarray([np.clip(torque_raw[i], model.actuator_ctrlrange[actuator_ids[name], 0], model.actuator_ctrlrange[actuator_ids[name], 1]) for i, name in enumerate(("mx_motor", "my_motor", "mz_motor"))])
            last_limited = {"thrust": thrust_limited, "torque": torque_limited}
            data.ctrl[:] = 0.0
            data.ctrl[actuator_ids["thrust_motor"]] = thrust_limited
            data.ctrl[actuator_ids["mx_motor"]] = torque_limited[0]
            data.ctrl[actuator_ids["my_motor"]] = torque_limited[1]
            data.ctrl[actuator_ids["mz_motor"]] = torque_limited[2]
            inner_calls += 1
            roll, pitch, yaw = _rpy(state.rotation)
            diag = pid.diagnostics
            rows.append({
                "time": float(wind["time"][index]), "scenario": scenario, "seed": -1 if wind["seed"] is None else int(wind["seed"]), "protocol_mode": "free_flight_controlled", "wind_x": float(wind["wind_x"][index]), "wind_y": 0.0, "wind_z": 0.0,
                "x_ref": reference.x_ref, "vx_ref": reference.vx_ref, "ax_ref": reference.ax_ref, "y_ref": reference.y_ref, "z_ref": reference.z_ref, "yaw_ref": reference.yaw_ref,
                "uav_x": state.position[0], "uav_y": state.position[1], "uav_z": state.position[2], "uav_vx": state.velocity[0], "uav_vy": state.velocity[1], "uav_vz": state.velocity[2],
                "uav_qw": data.xquat[quad_id, 0], "uav_qx": data.xquat[quad_id, 1], "uav_qy": data.xquat[quad_id, 2], "uav_qz": data.xquat[quad_id, 3],
                **{f"joint_{i}_angle": state.joint_angles[i - 1] for i in range(1, model_cfg.n_links + 1)}, **{f"joint_{i}_velocity": state.joint_velocities[i - 1] for i in range(1, model_cfg.n_links + 1)},
                "tip_x": data.site_xpos[tip_id, 0], "tip_y": data.site_xpos[tip_id, 1], "tip_z": data.site_xpos[tip_id, 2], "tip_relative_x": data.site_xpos[tip_id, 0] - state.position[0], "tip_equilibrium_relative_x": equilibrium_relative_x, "tip_displacement": state.tip_displacement,
                "wind_force_quad_x": force["quadrotor_x"], **{f"wind_force_link_{i}_x": force[f"link_{i}_x"] for i in range(1, model_cfg.n_links + 1)}, "wind_force_cutter_x": force["cutter_x"], "wind_force_total_x": force["total_x"],
                "ax_cmd_raw": diag.ax_cmd_raw, "ax_cmd_limited": diag.ax_cmd_limited, "ax_saturated": diag.ax_saturated, "solve_time_ms": last_pid_time_ms,
                "controller": "pid", "anchor_active": False, "position_error_x": diag.position_error_x, "velocity_error_x": diag.velocity_error_x, "pid_integral_x": diag.pid_integral_x,
                "ax_reference_feedforward": diag.ax_reference_feedforward, "ax_pid_feedback": diag.ax_pid_feedback, "ax_cmd_amplitude_limited": diag.ax_cmd_amplitude_limited, "ax_slew_limited": diag.ax_slew_limited,
                "thrust_cmd_raw_N": last_inner["thrust_raw_N"], "thrust_cmd_limited_N": last_limited["thrust"], "mx_cmd_raw_Nm": torque_raw[0], "my_cmd_raw_Nm": torque_raw[1], "mz_cmd_raw_Nm": torque_raw[2], "mx_cmd_limited_Nm": last_limited["torque"][0], "my_cmd_limited_Nm": last_limited["torque"][1], "mz_cmd_limited_Nm": last_limited["torque"][2], "inner_loop_saturated": bool(thrust_raw != last_limited["thrust"] or np.any(torque_raw != last_limited["torque"])), "roll_rad": roll, "pitch_rad": pitch, "yaw_rad": yaw,
            })
            log_calls += 1
        if step < physics_steps:
            mujoco.mj_step(model, data)

    output_csv = Path(output_csv)
    _write_csv(output_csv, rows, controlled_schema_columns(model_cfg.n_links))
    metrics = compute_controlled_metrics(output_csv, float(scenario_cfg[scenario]["settling_start_s"]))
    metrics.update({"physics_intervals": physics_steps, "formal_log_samples": log_calls, "outer_control_updates": outer_calls, "inner_loop_updates": inner_calls, "wind_force_calls": wind_calls, "anchor_active": False})
    return metrics
