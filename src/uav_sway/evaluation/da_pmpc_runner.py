"""Runner for the S5A2 LQR-stabilized DA-PMPC pilot."""

from __future__ import annotations

import csv
import time
from pathlib import Path

import mujoco
import numpy as np
import yaml
from scipy.linalg import solve_discrete_are

from uav_sway.control.base import ReferenceState
from uav_sway.control.da_pmpc import LQRStabilizedDAPMPC
from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.control.state_reader import StateReader
from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind
from uav_sway.disturbances.wind_io import read_wind_csv
from uav_sway.evaluation.controlled_metrics import compute_controlled_metrics
from uav_sway.models.model_config import load_model_config
from uav_sway.control.reference_horizon import make_reference_horizon
from uav_sway.linearization.reduced_state import ReducedStateLayout


ROOT = Path(__file__).resolve().parents[3]
FROZEN = ROOT / "reproducibility/frozen"


def _id(model, typ, name):
    value = int(mujoco.mj_name2id(model, typ, name))
    if value < 0:
        raise KeyError(name)
    return value


def _read_csv(path):
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    keys = ("time", "x_ref", "vx_ref", "ax_ref", "y_ref", "z_ref", "yaw_ref")
    return {key: np.asarray([float(row[key]) for row in rows], dtype=float) for key in keys}


def _reference(ref, index):
    return ReferenceState(*(float(ref[key][index]) for key in
                            ("x_ref", "vx_ref", "ax_ref", "y_ref", "z_ref", "yaw_ref")))


def _rpy(rotation):
    rotation = np.asarray(rotation, dtype=float).reshape(3, 3)
    return (float(np.arctan2(rotation[2, 1], rotation[2, 2])),
            float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))),
            float(np.arctan2(rotation[1, 0], rotation[0, 0])))


def _write_csv(path, rows, columns):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            output = {}
            for column in columns:
                value = row[column]
                if isinstance(value, (bool, np.bool_)):
                    output[column] = "true" if value else "false"
                elif isinstance(value, str):
                    output[column] = value
                else:
                    output[column] = format(float(value), ".17g")
            writer.writerow(output)


def schema_columns(n_links=5):
    base = [
        "time", "scenario", "protocol_mode", "wind_x", "x_ref", "vx_ref", "ax_ref",
        "y_ref", "z_ref", "yaw_ref", "uav_x", "uav_y", "uav_z", "uav_vx", "uav_vy",
        "uav_vz", "tip_x", "tip_y", "tip_z", "tip_displacement",
    ]
    joints = [name for index in range(1, n_links + 1)
              for name in (f"joint_{index}_angle", f"joint_{index}_velocity")]
    return base + joints + [
        "ax_cmd_raw", "ax_cmd_limited", "ax_saturated", "ax_slew_limited", "anchor_active",
        "roll_rad", "pitch_rad", "yaw_rad", "thrust_cmd_raw_N", "thrust_cmd_limited_N",
        "mx_cmd_raw_Nm", "my_cmd_raw_Nm", "mz_cmd_raw_Nm", "mx_cmd_limited_Nm",
        "my_cmd_limited_Nm", "mz_cmd_limited_Nm", "rotor_motor_max_abs_cmd", "solve_time_ms",
        "controller", "controller_mode", "position_error_x", "velocity_error_x", "lqr_feedback_ax",
        "residual_v", "disturbance_hat", "qp_limiter_mismatch", "qp_status_code", "qp_iterations",
        "preview_horizon_steps", "tip_weight", "residual_weight", "observer_enabled",
        "disturbance_compensation",
    ]


def _load_controller(model, da_config, mode):
    A = np.load(FROZEN / "linear_model/A.npy")
    B = np.load(FROZEN / "linear_model/B.npy")
    Q = np.load(FROZEN / "linear_model/Q.npy")
    R_matrix = np.load(FROZEN / "linear_model/R.npy")
    gain = np.load(FROZEN / "linear_model/K.npy")
    P = solve_discrete_are(A, B, Q, R_matrix)
    C_tip = np.load(FROZEN / "linear_model/C_tip.npy")
    from uav_sway.control.disturbance_observer import MatchedDisturbanceObserver
    from uav_sway.mpc.osqp_solver import OSQPPreviewSolver

    observer_enabled = mode in ("lqr_observer", "full") and bool(da_config.get("observer_enabled", True))
    residual_enabled = mode in ("preview", "full")
    disturbance_compensation = observer_enabled and bool(da_config.get("disturbance_compensation", True))
    observer = MatchedDisturbanceObserver(
        A, B[:, 0], float(da_config["disturbance_observer_gain"]),
        float(da_config["disturbance_limit_m_s2"]),
    )
    solver = OSQPPreviewSolver(
        da_config["osqp_eps_abs"], da_config["osqp_eps_rel"],
        da_config["osqp_max_iter"], da_config["osqp_warm_start"],
    )
    return LQRStabilizedDAPMPC(
        A, B, Q, P, C_tip, gain,
        float(da_config.get("selected_tip_weight", da_config["tip_weight_candidates"][1])),
        float(da_config.get("selected_residual_weight", da_config["residual_weight_candidates"][1])),
        solver, observer, int(da_config["horizon_steps"]),
        float(da_config["ax_min_m_s2"]), float(da_config["ax_max_m_s2"]),
        float(da_config["ax_slew_limit_m_s2_per_update"]),
        observer_enabled=observer_enabled,
        residual_enabled=residual_enabled,
        disturbance_compensation=disturbance_compensation,
    )


def run_scene(model_config_path, da_config, scene, wind_path, reference_path,
              output_csv, mode="full", duration_s=12.0):
    model = mujoco.MjModel.from_xml_path(str(FROZEN / "model/model_5link_controlled.xml"))
    model_cfg = load_model_config(model_config_path)
    aero = load_aerodynamic_config(ROOT / "configs/aerodynamics.yaml")
    ref = _read_csv(reference_path)
    wind = read_wind_csv(wind_path)
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    data.eq_active[:] = 0
    mujoco.mj_forward(model, data)
    quad = _id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    tip = _id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip")
    relative_x = float(data.site_xpos[tip, 0] - data.xpos[quad, 0])
    reader = StateReader(model, model_cfg.n_links, relative_x)
    layout = ReducedStateLayout(model)
    total_mass = float(np.sum(model.body_mass))
    s3 = yaml.safe_load((ROOT / "configs/s3_pid.yaml").read_text(encoding="utf-8"))
    inner = GeometricInnerLoop(
        total_mass, np.asarray(model.body_inertia[quad], dtype=float),
        s3["attitude_natural_frequency_rad_s"], s3["attitude_damping_ratio"],
        *s3["position_gains_y"], *s3["position_gains_z"],
    )
    controller = _load_controller(model, da_config, mode)
    controller.reset()
    actuator_names = ("rotor_motor_0", "rotor_motor_1", "rotor_motor_2", "rotor_motor_3",
                      "thrust_motor", "mx_motor", "my_motor", "mz_motor")
    actuator_ids = {name: _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in actuator_names}
    physics_dt = float(model.opt.timestep)
    signal_steps = int(round(0.005 / physics_dt))
    outer_steps = int(round(0.05 / physics_dt))
    physics_steps = int(round(duration_s / physics_dt))
    rows = []
    last_ax = 0.0
    outer_calls = inner_calls = log_calls = wind_calls = 0
    last_solve = 0.0
    for step in range(physics_steps + 1):
        index = min(step // signal_steps, len(wind["time"]) - 1)
        force = clear_and_apply_wind(model, data, model_cfg, aero, float(wind["wind_x"][index]))
        del force
        wind_calls += 1
        reference = _reference(ref, index)
        if step % outer_steps == 0:
            horizon = make_reference_horizon(ref, index, controller.horizon_steps)
            reduced_state = layout.extract(model, data, reference)
            started = time.perf_counter_ns()
            last_ax = controller.command(reduced_state, horizon)
            last_solve = (time.perf_counter_ns() - started) / 1.0e6
            outer_calls += 1
        if step % signal_steps == 0:
            state = reader.read(model, data)
            inner_result = inner.compute(state, reference, last_ax)
            thrust_raw = float(inner_result["thrust_raw_N"])
            torque_raw = np.asarray(inner_result["torque_raw_Nm"], dtype=float)
            thrust_lim = float(np.clip(thrust_raw, *model.actuator_ctrlrange[actuator_ids["thrust_motor"]]))
            torque_lim = np.asarray([
                np.clip(torque_raw[i], *model.actuator_ctrlrange[actuator_ids[actuator_names[i + 5]]])
                for i in range(3)
            ])
            data.ctrl[:] = 0.0
            data.ctrl[actuator_ids["thrust_motor"]] = thrust_lim
            for i, name in enumerate(("mx_motor", "my_motor", "mz_motor")):
                data.ctrl[actuator_ids[name]] = torque_lim[i]
            roll, pitch, yaw = _rpy(state.rotation)
            diagnostics = controller.diagnostics
            rows.append({
                "time": float(wind["time"][index]), "scenario": scene,
                "protocol_mode": "free_flight_controlled", "wind_x": float(wind["wind_x"][index]),
                "x_ref": reference.x_ref, "vx_ref": reference.vx_ref, "ax_ref": reference.ax_ref,
                "y_ref": reference.y_ref, "z_ref": reference.z_ref, "yaw_ref": reference.yaw_ref,
                "uav_x": state.position[0], "uav_y": state.position[1], "uav_z": state.position[2],
                "uav_vx": state.velocity[0], "uav_vy": state.velocity[1], "uav_vz": state.velocity[2],
                "tip_x": data.site_xpos[tip, 0], "tip_y": data.site_xpos[tip, 1],
                "tip_z": data.site_xpos[tip, 2], "tip_displacement": state.tip_displacement,
                **{f"joint_{i}_angle": state.joint_angles[i - 1] for i in range(1, 6)},
                **{f"joint_{i}_velocity": state.joint_velocities[i - 1] for i in range(1, 6)},
                "ax_cmd_raw": diagnostics.ax_cmd_raw, "ax_cmd_limited": diagnostics.ax_cmd_limited,
                "ax_saturated": bool(controller.limiter.diagnostics.saturated),
                "ax_slew_limited": bool(controller.limiter.diagnostics.slew_limited),
                "anchor_active": False, "roll_rad": roll, "pitch_rad": pitch, "yaw_rad": yaw,
                "thrust_cmd_raw_N": thrust_raw, "thrust_cmd_limited_N": thrust_lim,
                "mx_cmd_raw_Nm": torque_raw[0], "my_cmd_raw_Nm": torque_raw[1],
                "mz_cmd_raw_Nm": torque_raw[2], "mx_cmd_limited_Nm": torque_lim[0],
                "my_cmd_limited_Nm": torque_lim[1], "mz_cmd_limited_Nm": torque_lim[2],
                "rotor_motor_max_abs_cmd": 0.0, "solve_time_ms": last_solve,
                "controller": "lqr_stabilized_da_pmpc", "controller_mode": mode,
                "position_error_x": reduced_state[0] if step % outer_steps == 0 else state.position[0] - reference.x_ref,
                "velocity_error_x": reduced_state[1] if step % outer_steps == 0 else state.velocity[0] - reference.vx_ref,
                "lqr_feedback_ax": diagnostics.lqr_feedback_ax, "residual_v": diagnostics.residual_v,
                "disturbance_hat": diagnostics.disturbance_hat,
                "qp_limiter_mismatch": diagnostics.qp_limiter_mismatch,
                "qp_status_code": 1.0 if diagnostics.status in ("parity", "solved", "solved inaccurate") else 0.0,
                "qp_iterations": diagnostics.iterations, "preview_horizon_steps": controller.horizon_steps,
                "tip_weight": controller.tip_weight, "residual_weight": controller.residual_weight,
                "observer_enabled": controller.observer_enabled,
                "disturbance_compensation": controller.disturbance_compensation,
            })
            inner_calls += 1
            log_calls += 1
        if step < physics_steps:
            mujoco.mj_step(model, data)
    _write_csv(output_csv, rows, schema_columns(5))
    metric = compute_controlled_metrics(output_csv, float(da_config["settling_start_s"][scene]))
    metric.update({
        "controller": "lqr_stabilized_da_pmpc", "controller_mode": mode,
        "formal_log_samples": log_calls, "outer_control_updates": outer_calls,
        "inner_loop_updates": inner_calls, "wind_force_calls": wind_calls,
        "anchor_active": False, "physics_intervals": physics_steps,
        "final_d_hat": float(rows[-1]["disturbance_hat"]),
        "max_abs_d_hat": float(max(abs(float(row["disturbance_hat"])) for row in rows)),
        "qp_limiter_mismatch_max": float(max(float(row["qp_limiter_mismatch"]) for row in rows)),
    })
    return metric


__all__ = ["run_scene", "schema_columns", "ROOT"]
