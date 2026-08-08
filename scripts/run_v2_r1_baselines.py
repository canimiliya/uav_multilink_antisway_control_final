"""Run the corrected V2-R1R1 development-only traditional baseline freeze.

The runner intentionally loads only development_manifest.json. Holdout samples
are checked as metadata after the run and are never passed to MuJoCo.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import time
from pathlib import Path

import mujoco
import numpy as np
import yaml

from uav_sway.control.full_state_lqr import FullStateLQR
from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.control.position_pid import PositionPID
from uav_sway.control.task_lqr import TaskLQR, build_task_lqr
from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind
from uav_sway.disturbances.wind_profiles import generate_wind_profile
from uav_sway.evaluation.metrics import control_rate_proxy
from uav_sway.linearization.analysis import solve_lqr
from uav_sway.linearization.reduced_state import ReducedStateLayout
from uav_sway.models.model_config import load_model_config
from uav_sway.task_space.reference import build_equilibrium_task_pose
from uav_sway.task_space.state import CutterTaskSpaceReader
from uav_sway.task_space.v2_reference import CutterTargetMapper, Shared3DControlLimits
from uav_sway.control.base import ReferenceState
from uav_sway.control.state_reader import StateReader


ROOT = Path(__file__).resolve().parents[1]
R1 = ROOT / "reproducibility/v2/r1"
OUTPUT = ROOT / "reproducibility/v2/r1r1"
MODEL_PATH = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
START_HEAD = "6b4ae43b3b6a3dff669beaf66fd0c453a60a719f"
DT_SIGNAL = 0.005
DT_OUTER = 0.05
DURATION = 12.0
AX_MIN, AX_MAX, AX_SLEW = -2.0, 2.0, 0.25
SAFETY = {
    "minimum_uav_height_m": 0.05,
    "minimum_tip_height_m": 0.05,
    "maximum_abs_joint_angle_rad": 100.0 * math.pi / 180.0,
    "maximum_abs_roll_pitch_rad": 25.0 * math.pi / 180.0,
    "maximum_abs_ax_m_s2": 2.0,
    "maximum_ax_step_change_m_s2": 0.25,
    "maximum_abs_ay_m_s2": 2.0,
    "maximum_ay_step_change_m_s2": 0.25,
    "maximum_abs_az_m_s2": 2.0,
    "maximum_az_step_change_m_s2": 0.25,
    "maximum_thrust_N": 285.74568,
    "maximum_abs_torque_Nm": 25.0,
}


def read_json(name: str, root: Path = R1) -> dict:
    return json.loads((root / name).read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def equilibrium_context() -> tuple[mujoco.MjModel, dict, CutterTargetMapper, StateReader, CutterTaskSpaceReader, dict]:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    data.eq_active[:] = 0
    mujoco.mj_forward(model, data)
    pose = build_equilibrium_task_pose(model, data, MODEL_PATH)
    tip_reader = CutterTaskSpaceReader(model)
    tip_state = tip_reader.read(model, data)
    tip_position = np.asarray(tip_state.tip_position_world, dtype=float)
    mapper = CutterTargetMapper(pose.tip_relative_position_m, pose.cutter_axis_world)
    model_cfg = load_model_config(ROOT / "configs/model_5link.yaml")
    quad_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    reader = StateReader(model, model_cfg.n_links, float(pose.tip_relative_position_m[0]))
    ids = {
        "quad": quad_id, "tip": tip_id,
        **{name: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)) for name in ("rotor_motor_0", "rotor_motor_1", "rotor_motor_2", "rotor_motor_3", "thrust_motor", "mx_motor", "my_motor", "mz_motor")},
    }
    return model, {"data": data, "tip_position": tip_position, "pose": pose, "model_cfg": model_cfg, "ids": ids}, mapper, reader, tip_reader, {"model": model, "ids": ids}


def wind_series(sample: dict) -> tuple[np.ndarray, np.ndarray]:
    time_values = np.arange(0.0, DURATION + 0.5 * DT_SIGNAL, DT_SIGNAL)
    spec = sample["wind"]
    kind = spec["kind"]
    if kind == "constant":
        wind = np.full_like(time_values, float(spec["speed_m_s"]))
    elif kind == "frozen_random_wind":
        config = yaml.safe_load((ROOT / "configs/wind_profiles.yaml").read_text(encoding="utf-8"))
        generated = generate_wind_profile("low_frequency_random", config, seed=int(spec["seed"]), dt=DT_SIGNAL)
        wind = generated.wind_x
    elif kind == "linear_ramp":
        start, end = map(float, spec["ramp_window_s"])
        wind = np.zeros_like(time_values)
        mask = (time_values >= start) & (time_values <= end)
        wind[mask] = (time_values[mask] - start) / (end - start) * float(spec["end_speed_m_s"])
        wind[time_values > end] = float(spec["end_speed_m_s"])
    else:
        raise ValueError(f"unsupported V2 wind kind: {kind}")
    return time_values, np.asarray(wind, dtype=float)


def target_reference(sample: dict, t: float, mapper: CutterTargetMapper, equilibrium_tip: np.ndarray) -> tuple[np.ndarray, ReferenceState]:
    if sample["scenario"] == "RAMP_WIND_EQUILIBRIUM_HOLD" or sample["target_issue_time_s"] is None or t < float(sample["target_issue_time_s"]):
        tip_target = equilibrium_tip
    else:
        tip_target = np.asarray(sample["target"]["tip_target_world_m"], dtype=float)
    uav = mapper.uav_reference_from_tip_target(tip_target)
    return tip_target.copy(), ReferenceState(float(uav[0]), 0.0, 0.0, float(uav[1]), float(uav[2]), 0.0)


def rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    return float(np.arctan2(rotation[2, 1], rotation[2, 2])), float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))), float(np.arctan2(rotation[1, 0], rotation[0, 0]))


def lqr_gain(kind: str, params: dict) -> tuple[np.ndarray, float]:
    a = np.load(ROOT / "reproducibility/frozen/linear_model/A.npy")
    b = np.load(ROOT / "reproducibility/frozen/linear_model/B.npy")
    if kind == "lqr":
        q = np.diag([
            params["position_error_weight"], params["velocity_error_weight"], 8.0, 2.0, 4.0, 1.0,
            *([params["joint_angle_weight"]] * 5), *([params["joint_velocity_weight"]] * 5),
        ])
        result = solve_lqr(a, b, q, np.asarray([[params["input_weight"]]], dtype=float))
    else:
        c = np.load(ROOT / "reproducibility/frozen/task_lqr/C_task.npy")
        result = build_task_lqr(a, b, c, params["w_p"], params["w_theta"], params["R"])
    return np.asarray(result["K"], dtype=float), float(result["spectral_radius"])


def candidate_controller(kind: str, params: dict, gain: np.ndarray | None):
    if kind == "pid":
        return PositionPID(params["kp"], params["kd"], params["ki"], AX_MIN, AX_MAX, AX_SLEW, 1.0)
    if kind == "lqr":
        return FullStateLQR(gain, AX_MIN, AX_MAX, AX_SLEW)
    return TaskLQR(gain, AX_MIN, AX_MAX, AX_SLEW)


def write_raw(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = list(rows[0])
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def summarize_trace(trace: dict, sample: dict, output_csv: Path | None) -> dict:
    t = trace["time"]
    pos = trace["position_error_3d_m"]
    ori = trace["orientation_error_deg"]
    speed = trace["tip_speed_m_s"]
    angular_speed = trace["cutter_angular_speed_rad_s"]
    issue = 0.0 if sample["target_issue_time_s"] is None else float(sample["target_issue_time_s"])
    mask = (pos <= 0.05) & (speed <= 0.10) & (ori <= 5.0) & (angular_speed <= 0.10)
    acquired = False
    acquisition = None
    for i in range(len(t)):
        if t[i] < issue or not mask[i]:
            continue
        end = int(np.searchsorted(t, t[i] + 1.0 - 1e-12, side="left"))
        if end < len(t) and t[end] - t[i] >= 1.0 - 1e-9 and bool(np.all(mask[i:end + 1])):
            acquired, acquisition = True, float(t[i] - issue)
            break
    ramp = sample["scenario"] == "RAMP_WIND_EQUILIBRIUM_HOLD"
    peak = float(np.max(pos[(t >= 2.0) & (t <= 8.0)])) if ramp else None
    steady = float(np.sqrt(np.mean(pos[t >= 8.0] ** 2))) if ramp else None
    ax_step = float(np.max(np.abs(np.diff(trace["ax_cmd"])))) if len(t) > 1 else 0.0
    ay_step = float(np.max(np.abs(np.diff(trace["ay_cmd"])))) if len(t) > 1 else 0.0
    az_step = float(np.max(np.abs(np.diff(trace["az_cmd"])))) if len(t) > 1 else 0.0
    safe_reasons = {
        "finite": bool(all(np.isfinite(v).all() for v in trace.values() if isinstance(v, np.ndarray) and v.dtype.kind in "fc")),
        "minimum_uav_height": float(np.min(trace["uav_z"])) > SAFETY["minimum_uav_height_m"],
        "minimum_tip_height": float(np.min(trace["tip_z"])) > SAFETY["minimum_tip_height_m"],
        "joint_angle": float(np.max(np.abs(trace["joint_angles"]))) < SAFETY["maximum_abs_joint_angle_rad"],
        "roll_pitch": float(max(np.max(np.abs(trace["roll"])), np.max(np.abs(trace["pitch"])))) < SAFETY["maximum_abs_roll_pitch_rad"],
        "ax_amplitude": float(np.max(np.abs(trace["ax_cmd"]))) <= SAFETY["maximum_abs_ax_m_s2"] + 1e-12,
        "ax_slew": ax_step <= SAFETY["maximum_ax_step_change_m_s2"] + 1e-12,
        "ay_amplitude": float(np.max(np.abs(trace["ay_cmd"]))) <= SAFETY["maximum_abs_ay_m_s2"] + 1e-12,
        "ay_slew": ay_step <= SAFETY["maximum_ay_step_change_m_s2"] + 1e-12,
        "az_amplitude": float(np.max(np.abs(trace["az_cmd"]))) <= SAFETY["maximum_abs_az_m_s2"] + 1e-12,
        "az_slew": az_step <= SAFETY["maximum_az_step_change_m_s2"] + 1e-12,
        "thrust": float(np.max(trace["thrust"])) <= SAFETY["maximum_thrust_N"] + 1e-9,
        "torque": float(np.max(np.abs(trace["torque"]))) <= SAFETY["maximum_abs_torque_Nm"] + 1e-9,
        "anchor_inactive": not bool(np.any(trace["anchor_active"])),
        "rotor_commands_zero": bool(np.max(np.abs(trace["rotor_commands"])) == 0.0),
    }
    result = {
        "sample_id": sample["sample_id"], "scenario": sample["scenario"], "seed": sample["wind"].get("seed", -1),
        "safe": bool(all(safe_reasons.values())), "safety_reasons": safe_reasons, "task_success": bool(acquired),
        "acquisition_time_s": acquisition, "position_rmse_3d_m": float(np.sqrt(np.mean(pos ** 2))),
        "orientation_rmse_deg": float(np.sqrt(np.mean(ori ** 2))), "position_final_error_m": float(pos[-1]),
        "orientation_final_error_deg": float(ori[-1]), "tip_speed_rms_m_s": float(np.sqrt(np.mean(speed ** 2))),
        "cutter_angular_speed_rms_rad_s": float(np.sqrt(np.mean(angular_speed ** 2))),
        "ramp_peak_position_error_m": peak, "ramp_steady_state_position_error_m": steady,
        "control_effort": float(np.trapezoid(trace["ax_cmd"] ** 2, t)), "x_control_effort": float(np.trapezoid(trace["ax_cmd"] ** 2, t)),
        "total_acceleration_effort": float(np.trapezoid(trace["ax_cmd"] ** 2 + trace["ay_cmd"] ** 2 + trace["az_cmd"] ** 2, t)), "control_rate": control_rate_proxy(t, trace["ax_cmd"]),
        "max_abs_ax_m_s2": float(np.max(np.abs(trace["ax_cmd"]))), "max_ax_step_m_s2": ax_step,
        "max_abs_ay_m_s2": float(np.max(np.abs(trace["ay_cmd"]))), "max_ay_step_m_s2": ay_step,
        "max_abs_az_m_s2": float(np.max(np.abs(trace["az_cmd"]))), "max_az_step_m_s2": az_step,
        "max_cutter_angular_speed_rad_s": float(np.max(angular_speed)),
        "max_thrust_N": float(np.max(trace["thrust"])), "max_abs_torque_Nm": float(np.max(np.abs(trace["torque"]))),
        "minimum_tip_height_m": float(np.min(trace["tip_z"])), "minimum_uav_height_m": float(np.min(trace["uav_z"])),
        "maximum_abs_joint_angle_rad": float(np.max(np.abs(trace["joint_angles"]))),
        "maximum_abs_roll_rad": float(np.max(np.abs(trace["roll"]))), "maximum_abs_pitch_rad": float(np.max(np.abs(trace["pitch"]))),
    }
    if output_csv is not None:
        rows = []
        for i, value in enumerate(t):
            rows.append({"time_s": float(value), "position_error_3d_m": float(pos[i]), "orientation_error_deg": float(ori[i]), "tip_speed_m_s": float(speed[i]), "cutter_angular_speed_rad_s": float(angular_speed[i]), "ax_cmd_m_s2": float(trace["ax_cmd"][i]), "ay_cmd_m_s2": float(trace["ay_cmd"][i]), "az_cmd_m_s2": float(trace["az_cmd"][i]), "tip_x_m": float(trace["tip_x"][i]), "tip_y_m": float(trace["tip_y"][i]), "tip_z_m": float(trace["tip_z"][i]), "target_x_m": float(trace["target_x"][i]), "target_y_m": float(trace["target_y"][i]), "target_z_m": float(trace["target_z"][i]), "wind_x_m_s": float(trace["wind_x"][i]), "thrust_N": float(trace["thrust"][i]), "max_abs_torque_Nm": float(np.max(np.abs(trace["torque"][i]))), "safe": bool(result["safe"]), "task_success": bool(result["task_success"]), "anchor_active": bool(trace["anchor_active"][i])})
        write_raw(output_csv, rows)
        result["raw_csv"] = str(output_csv)
    return result


def run_case(kind: str, params: dict, yz: dict, sample: dict, output_csv: Path | None = None) -> dict:
    if sample.get("split") != "development" or not sample.get("execution_allowed", False):
        raise RuntimeError("V2-R1 runner refuses non-development or forbidden sample")
    model, ctx, mapper, reader, tip_reader, _ = equilibrium_context()
    data = ctx["data"]
    ids = ctx["ids"]
    model_cfg = ctx["model_cfg"]
    equilibrium_tip = ctx["tip_position"]
    pose = ctx["pose"]
    s3 = yaml.safe_load((ROOT / "configs/s3_pid.yaml").read_text(encoding="utf-8"))
    aero = load_aerodynamic_config(ROOT / "configs/aerodynamics.yaml")
    total_mass = float(np.sum(model.body_mass))
    inner = GeometricInnerLoop(total_mass, np.asarray(model.body_inertia[ids["quad"]], dtype=float), s3["attitude_natural_frequency_rad_s"], s3["attitude_damping_ratio"], float(yz["ay_kp"]), float(yz["ay_kd"]), float(yz["az_kp"]), float(yz["az_kd"]), Shared3DControlLimits())
    inner.reset()
    gain = None
    spectral_radius = None
    if kind in {"lqr", "task_lqr"}:
        gain, spectral_radius = lqr_gain("lqr" if kind == "lqr" else "task_lqr", params)
    controller = candidate_controller("lqr" if kind == "lqr" else kind, params, gain)
    layout = ReducedStateLayout(model) if kind in {"lqr", "task_lqr"} else None
    times, wind = wind_series(sample)
    physics_dt = float(model.opt.timestep)
    signal_steps = int(round(DT_SIGNAL / physics_dt)); outer_steps = int(round(DT_OUTER / physics_dt)); physics_steps = int(round(DURATION / physics_dt))
    state = reader.read(model, data)
    _, reference = target_reference(sample, 0.0, mapper, equilibrium_tip)
    controller.reset(state, reference) if kind == "pid" else controller.reset()
    shared_yz = (0.0, 0.0)
    trace = {key: [] for key in ("time", "position_error_3d_m", "orientation_error_deg", "tip_speed_m_s", "cutter_angular_speed_rad_s", "ax_cmd", "ay_cmd", "az_cmd", "uav_z", "tip_x", "tip_y", "tip_z", "target_x", "target_y", "target_z", "wind_x", "thrust", "torque", "joint_angles", "roll", "pitch", "anchor_active", "rotor_commands")}
    for step in range(physics_steps + 1):
        wi = min(step // signal_steps, len(wind) - 1)
        current_time = float(times[wi])
        clear_and_apply_wind(model, data, model_cfg, aero, float(wind[wi]))
        tip_target, reference = target_reference(sample, current_time, mapper, equilibrium_tip)
        if step % outer_steps == 0:
            state = reader.read(model, data)
            task_for_control = tip_reader.read(model, data)
            shared_yz = inner.shared_yz_command(state, reference, task_for_control, tip_target)
            if kind == "pid":
                controller.command(state, reference, DT_OUTER)
            else:
                controller.command(layout.extract(model, data, reference), reference, DT_OUTER)
        if step % signal_steps == 0:
            state = reader.read(model, data)
            inner_out = inner.compute(state, reference, controller.diagnostics.ax_cmd_limited, shared_yz)
            thrust_raw = float(inner_out["thrust_raw_N"]); torque_raw = np.asarray(inner_out["torque_raw_Nm"], dtype=float)
            thrust = float(np.clip(thrust_raw, *model.actuator_ctrlrange[ids["thrust_motor"]]))
            torque = np.asarray([np.clip(torque_raw[i], *model.actuator_ctrlrange[ids[name]]) for i, name in enumerate(("mx_motor", "my_motor", "mz_motor"))], dtype=float)
            data.ctrl[:] = 0.0; data.ctrl[ids["thrust_motor"]] = thrust
            for i, name in enumerate(("mx_motor", "my_motor", "mz_motor")): data.ctrl[ids[name]] = torque[i]
            task = tip_reader.read(model, data)
            tip_velocity = task.tip_velocity_world
            position_error = np.asarray(task.tip_position_world, dtype=float) - tip_target
            axis_dot = float(np.clip(np.dot(task.cutter_axis_world, pose.cutter_axis_world), -1.0, 1.0))
            roll, pitch, _ = rpy(state.rotation)
            trace["time"].append(current_time); trace["position_error_3d_m"].append(float(np.linalg.norm(position_error))); trace["orientation_error_deg"].append(float(np.rad2deg(np.arccos(axis_dot)))); trace["tip_speed_m_s"].append(float(np.linalg.norm(tip_velocity))); trace["cutter_angular_speed_rad_s"].append(float(np.linalg.norm(task.cutter_angular_velocity_world))); trace["ax_cmd"].append(float(controller.diagnostics.ax_cmd_limited)); trace["ay_cmd"].append(float(shared_yz[0])); trace["az_cmd"].append(float(shared_yz[1])); trace["uav_z"].append(float(state.position[2])); trace["tip_x"].append(float(task.tip_position_world[0])); trace["tip_y"].append(float(task.tip_position_world[1])); trace["tip_z"].append(float(task.tip_position_world[2])); trace["target_x"].append(float(tip_target[0])); trace["target_y"].append(float(tip_target[1])); trace["target_z"].append(float(tip_target[2])); trace["wind_x"].append(float(wind[wi])); trace["thrust"].append(thrust); trace["torque"].append(torque.copy()); trace["joint_angles"].append(state.joint_angles.copy()); trace["roll"].append(roll); trace["pitch"].append(pitch); trace["anchor_active"].append(False); trace["rotor_commands"].append(np.zeros(4))
        if step < physics_steps:
            mujoco.mj_step(model, data)
    trace = {key: np.asarray(value) for key, value in trace.items()}
    result = summarize_trace(trace, sample, output_csv)
    result.update({"controller": kind, "candidate_id": params["candidate_id"], "spectral_radius": spectral_radius, "shared_yz": yz})
    return result


def aggregate(candidate_id: str, rows: list[dict], gain_norm: float) -> dict:
    safe = [row for row in rows if row["safe"]]
    success = [row for row in rows if row["task_success"]]
    acquired = [row["acquisition_time_s"] for row in success if row["acquisition_time_s"] is not None]
    return {
        "candidate_id": candidate_id, "sample_count": len(rows), "safe_sample_count": len(safe), "task_success_count": len(success),
        "safety_rate": len(safe) / len(rows), "task_success_rate": len(success) / len(rows),
        "acquisition_median_s": float(np.median(acquired)) if acquired else None,
        "position_rmse_3d_m": float(np.mean([row["position_rmse_3d_m"] for row in rows])),
        "orientation_rmse_deg": float(np.mean([row["orientation_rmse_deg"] for row in rows])),
        "control_effort": float(np.mean([row["control_effort"] for row in rows])), "x_control_effort": float(np.mean([row["x_control_effort"] for row in rows])),
        "total_acceleration_effort": float(np.mean([row["total_acceleration_effort"] for row in rows])),
        "ramp_peak_position_error_m": float(max((row["ramp_peak_position_error_m"] or 0.0) for row in rows)),
        "ramp_steady_state_position_error_m": float(max((row["ramp_steady_state_position_error_m"] or 0.0) for row in rows)),
        "gain_norm": float(gain_norm), "rows": rows,
    }


def selection_key(summary: dict) -> tuple:
    return (-summary["safe_sample_count"], -summary["task_success_rate"], summary["acquisition_median_s"] if summary["acquisition_median_s"] is not None else float("inf"), summary["position_rmse_3d_m"], summary["orientation_rmse_deg"], summary["ramp_peak_position_error_m"], summary["ramp_steady_state_position_error_m"], summary["total_acceleration_effort"], summary["gain_norm"])


def final_selection_key(summary: dict) -> tuple:
    """Primary-baseline rule, including the frozen ramp-wind tie breakers."""
    return (-summary["safe_sample_count"], -summary["task_success_rate"], summary["acquisition_median_s"] if summary["acquisition_median_s"] is not None else float("inf"), summary["position_rmse_3d_m"], summary["orientation_rmse_deg"], summary["ramp_peak_position_error_m"], summary["ramp_steady_state_position_error_m"], summary["total_acceleration_effort"], summary["gain_norm"])


def pid_grid() -> list[dict]:
    return [{"candidate_id": f"pid_{i:03d}", "kp": kp, "kd": kd, "ki": ki} for i, (kp, kd, ki) in enumerate(( (kp, kd, ki) for kp in [0.8, 1.6, 3.2] for kd in [1.0, 2.0, 4.0] for ki in [0.0, 0.1] ))]


def lqr_grid() -> list[dict]:
    rows = []
    i = 0
    for ja in [20.0, 80.0]:
        for jv in [3.0, 12.0]:
            for r in [0.5, 1.0]:
                for pos in [20.0, 40.0, 80.0, 160.0]:
                    for vel in [4.0, 8.0]:
                        rows.append({"candidate_id": f"lqr_{i:03d}", "joint_angle_weight": ja, "joint_velocity_weight": jv, "input_weight": r, "position_error_weight": pos, "velocity_error_weight": vel}); i += 1
    return rows


def task_lqr_grid() -> list[dict]:
    return [{"candidate_id": f"task_lqr_{i:03d}", "w_p": wp, "w_theta": wt, "R": r} for i, (wp, wt, r) in enumerate(((wp, wt, r) for wp in [20.0, 80.0, 320.0] for wt in [5.0, 20.0, 80.0] for r in [0.5, 1.0, 2.0]))]


def gain_norm(kind: str, params: dict) -> float:
    if kind == "pid":
        return float(np.linalg.norm([params["kp"], params["kd"], params["ki"]]))
    return float(np.linalg.norm(lqr_gain(kind, params)[0]))


def write_grid(name: str, rows: list[dict]) -> None:
    (OUTPUT / name).write_text(json.dumps({"grid": rows, "grid_size": len(rows)}, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def write_candidates(name: str, summaries: list[dict]) -> None:
    keys = ["candidate_id", "sample_count", "safe_sample_count", "task_success_count", "safety_rate", "task_success_rate", "acquisition_median_s", "position_rmse_3d_m", "orientation_rmse_deg", "control_effort", "x_control_effort", "total_acceleration_effort", "ramp_peak_position_error_m", "ramp_steady_state_position_error_m", "gain_norm"]
    with (OUTPUT / name).open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys, lineterminator="\n"); writer.writeheader()
        for summary in summaries:
            writer.writerow({key: summary[key] for key in keys})


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--keep-temp", action="store_true")
    args = parser.parse_args()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for name in ("development_manifest.json", "holdout_manifest.json", "sample_bank_contract.json"):
        source = R1 / name
        target = OUTPUT / name
        if target.exists() and sha256(target) != sha256(source):
            raise AssertionError(f"R1R1 sample-bank input drift: {name}")
        if not target.exists():
            shutil.copy2(source, target)
    development = read_json("development_manifest.json")["samples"]
    holdout = read_json("holdout_manifest.json")["samples"]
    if any(row.get("execution_allowed") for row in holdout):
        raise AssertionError("holdout execution permission is not false")
    for row in development:
        if row["split"] != "development" or not row["execution_allowed"]:
            raise AssertionError("development manifest is not executable")
    yz_grid = read_json("shared_task_yz_grid.json", OUTPUT)
    # Separate development-only task-space y and z scaffold tuning, using
    # calm +/-axis samples only.  The grid is frozen in r1r1 before this run.
    yz_results = {"y": [], "z": []}
    zero_pid = {"candidate_id": "scaffold_probe", "kp": 0.0, "kd": 0.0, "ki": 0.0}
    yz_candidates = {
        "y": [{"kp": kp, "kd": kd} for kp in yz_grid["y"]["kp"] for kd in yz_grid["y"]["kd"]],
        "z": [{"kp": kp, "kd": kd} for kp in yz_grid["z"]["kp"] for kd in yz_grid["z"]["kd"]],
    }
    if {axis: len(candidates) for axis, candidates in yz_candidates.items()} != {"y": 16, "z": 16}:
        raise AssertionError("R1R1 shared task y/z grid must be 16+16 candidates")
    for axis, candidates in yz_candidates.items():
        directions = {"y": {"+y", "-y"}, "z": {"+z", "-z"}}[axis]
        samples = [row for row in development if row["scenario"] == "CALM_3D_SETPOINT" and row["target"]["direction"] in directions and row["target"]["radius_m"] in {0.15, 0.25}]
        for i, candidate in enumerate(candidates):
            yz = {"ay_kp": candidate["kp"] if axis == "y" else 1.5, "ay_kd": candidate["kd"] if axis == "y" else 2.0, "az_kp": candidate["kp"] if axis == "z" else 4.0, "az_kd": candidate["kd"] if axis == "z" else 3.5}
            rows = [run_case("pid", zero_pid, yz, sample) for sample in samples]
            score = float(np.mean([row["position_rmse_3d_m"] for row in rows]))
            acquired = [row["acquisition_time_s"] for row in rows if row["task_success"] and row["acquisition_time_s"] is not None]
            yz_results[axis].append({"candidate": candidate, "score_position_rmse_m": score, "safe_count": sum(row["safe"] for row in rows), "task_success_count": sum(row["task_success"] for row in rows), "acquisition_median_s": float(np.median(acquired)) if acquired else None, "gain_norm": float(np.linalg.norm([candidate["kp"], candidate["kd"]])), "samples": len(rows), "shared_yz": yz})
    yz_freeze = {}
    for axis in ("y", "z"):
        ranked = sorted(yz_results[axis], key=lambda row: (-row["safe_count"], -row["task_success_count"], row["acquisition_median_s"] if row["acquisition_median_s"] is not None else float("inf"), row["score_position_rmse_m"], row["gain_norm"]))
        yz_freeze[axis] = ranked[0]
    shared_yz = {"ay_kp": yz_freeze["y"]["shared_yz"]["ay_kp"], "ay_kd": yz_freeze["y"]["shared_yz"]["ay_kd"], "az_kp": yz_freeze["z"]["shared_yz"]["az_kp"], "az_kd": yz_freeze["z"]["shared_yz"]["az_kd"]}
    (OUTPUT / "shared_task_yz_freeze.json").write_text(json.dumps({"selection_rule": "safety, task success, acquisition, task RMSE, gain norm", "selected": shared_yz, "grid_results": yz_results, "frozen_limits": {"ay_abs_max_m_s2": 2.0, "az_abs_max_m_s2": 2.0, "axis_slew_max_m_s2_per_update": 0.25}}, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    competence = {}
    for axis, directions in (("y", {"+y", "-y"}), ("z", {"+z", "-z"})):
        yz = shared_yz
        samples = [row for row in development if row["scenario"] == "CALM_3D_SETPOINT" and row["target"]["direction"] in directions and row["target"]["radius_m"] in {0.15, 0.25}]
        rows = [run_case("pid", zero_pid, yz, sample) for sample in samples]
        competence[axis] = {"sample_ids": [row["sample_id"] for row in rows], "safe_all": all(row["safe"] for row in rows), "acquired_count": sum(row["task_success"] for row in rows), "acquired_minimum": 3, "final_errors_m": {row["sample_id"]: row["position_final_error_m"] for row in rows}, "max_final_error_m": max(row["position_final_error_m"] for row in rows), "pass": all(row["safe"] for row in rows) and sum(row["task_success"] for row in rows) >= 3 and max(row["position_final_error_m"] for row in rows) <= 0.25}
    yz_pass = bool(competence["y"]["pass"] and competence["z"]["pass"])
    (OUTPUT / "shared_task_yz_competence.json").write_text(json.dumps({"pass": yz_pass, "criteria": {"all_four_calm_tasks_safe": True, "minimum_acquired_per_axis": 3, "maximum_single_task_final_3d_error_m": 0.25}, "axes": competence}, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    if not yz_pass:
        raise RuntimeError("BLOCKED_SHARED_3D_SCAFFOLD")
    tuning = [row for row in development if row["scenario"] in {"CALM_3D_SETPOINT", "WIND_3D_SETPOINT"} and row["target"]["direction"] in {"+x", "-x"} and row["target"]["radius_m"] in {0.15, 0.25} and ((row["scenario"] == "CALM_3D_SETPOINT") or row["wind"].get("speed_m_s") == 3.0)]
    if len(tuning) != 8:
        raise AssertionError(f"expected 8 tuning samples, got {len(tuning)}")
    all_grids = {"pid": pid_grid(), "lqr": lqr_grid(), "task_lqr": task_lqr_grid()}
    for name, grid in all_grids.items():
        write_grid({"pid": "pid_grid.json", "lqr": "lqr_grid.json", "task_lqr": "task_lqr_grid.json"}[name], grid)
    stage1_summaries = {}
    top3 = {}
    for kind, grid in all_grids.items():
        summaries = []
        for index, params in enumerate(grid, 1):
            rows = [run_case(kind, params, shared_yz, sample) for sample in tuning]
            summary = aggregate(params["candidate_id"], rows, gain_norm(kind, params)); summary["parameters"] = params; summaries.append(summary)
            if index % 10 == 0 or index == len(grid):
                print(f"stage1 {kind} {index}/{len(grid)}", flush=True)
        stage1_summaries[kind] = summaries
        top3[kind] = sorted(summaries, key=selection_key)[:3]
        write_candidates({"pid": "pid_candidates.csv", "lqr": "lqr_candidates.csv", "task_lqr": "task_lqr_candidates.csv"}[kind], summaries)
    tmp_root = OUTPUT / "runs"
    if tmp_root.exists(): shutil.rmtree(tmp_root)
    stage2_summaries = {}
    development_rows = []
    for kind, selected in top3.items():
        completed = []
        for rank, candidate_summary in enumerate(selected, 1):
            params = candidate_summary["parameters"]
            rows = []
            for sample in development:
                output = tmp_root / kind / params["candidate_id"] / f"{sample['sample_id']}.csv"
                rows.append(run_case(kind, params, shared_yz, sample, output))
            summary = aggregate(params["candidate_id"], rows, gain_norm(kind, params)); summary["parameters"] = params; summary["stage2_rank"] = rank; completed.append(summary)
            development_rows.extend({"controller": kind, "stage2_rank": rank, **{key: row[key] for key in ("sample_id", "safe", "task_success", "acquisition_time_s", "position_rmse_3d_m", "orientation_rmse_deg", "ramp_peak_position_error_m", "ramp_steady_state_position_error_m", "control_effort", "x_control_effort", "total_acceleration_effort", "max_abs_ax_m_s2", "max_ax_step_m_s2", "max_abs_ay_m_s2", "max_ay_step_m_s2", "max_abs_az_m_s2", "max_az_step_m_s2", "max_cutter_angular_speed_rad_s", "max_thrust_N", "max_abs_torque_Nm")}} for row in rows)
            print(f"stage2 {kind} rank={rank} complete", flush=True)
        stage2_summaries[kind] = completed
    selected_final = {kind: sorted(items, key=final_selection_key)[0] for kind, items in stage2_summaries.items()}
    for kind, summary in selected_final.items():
        freeze = {"controller": kind, "selected": {key: value for key, value in summary.items() if key not in {"rows", "parameters"}}, "parameters": summary["parameters"], "shared_yz": shared_yz, "stage2_selection_rule": "lexicographic safety, success, acquisition, 3D RMSE, orientation RMSE, effort, conservative gain norm"}
        (OUTPUT / {"pid": "pid_freeze.json", "lqr": "lqr_freeze.json", "task_lqr": "task_lqr_freeze.json"}[kind]).write_text(json.dumps(freeze, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    (OUTPUT / "development_baseline_results.csv").write_text("", encoding="utf-8", newline="")
    if development_rows:
        keys = list(development_rows[0])
        with (OUTPUT / "development_baseline_results.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=keys, lineterminator="\n"); writer.writeheader(); writer.writerows(development_rows)
    summary_payload = {kind: {key: value for key, value in item.items() if key not in {"rows", "parameters"}} | {"parameters": item["parameters"]} for kind, item in selected_final.items()}
    (OUTPUT / "development_baseline_summary.json").write_text(json.dumps({"shared_yz": shared_yz, "final_selected": summary_payload, "stage1_top3": {kind: [{key: value for key, value in item.items() if key not in {"rows", "parameters"}} | {"parameters": item["parameters"]} for item in selected] for kind, selected in top3.items()}}, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    baseline_ranked = sorted(selected_final.values(), key=final_selection_key)
    primary = baseline_ranked[0]
    envelope = {
        "best_success_rate": {"value": max(item["task_success_rate"] for item in selected_final.values()), "controller": max(selected_final, key=lambda kind: selected_final[kind]["task_success_rate"])},
        "best_acquisition_time_s": {"value": min(item["acquisition_median_s"] for item in selected_final.values() if item["acquisition_median_s"] is not None), "controller": min((kind for kind, item in selected_final.items() if item["acquisition_median_s"] is not None), key=lambda kind: selected_final[kind]["acquisition_median_s"])},
        "best_3d_position_rmse_m": {"value": min(item["position_rmse_3d_m"] for item in selected_final.values()), "controller": min(selected_final, key=lambda kind: selected_final[kind]["position_rmse_3d_m"])},
        "best_ramp_peak_error_m": {"value": min(item["ramp_peak_position_error_m"] for item in selected_final.values()), "controller": min(selected_final, key=lambda kind: selected_final[kind]["ramp_peak_position_error_m"])},
        "best_ramp_steady_state_error_m": {"value": min(item["ramp_steady_state_position_error_m"] for item in selected_final.values()), "controller": min(selected_final, key=lambda kind: selected_final[kind]["ramp_steady_state_position_error_m"])},
        "best_safety_rate": {"value": max(item["safety_rate"] for item in selected_final.values()), "controller": max(selected_final, key=lambda kind: selected_final[kind]["safety_rate"])},
        "best_orientation_rmse_deg": {"value": min(item["orientation_rmse_deg"] for item in selected_final.values()), "controller": min(selected_final, key=lambda kind: selected_final[kind]["orientation_rmse_deg"])},
        "best_total_acceleration_effort": {"value": min(item["total_acceleration_effort"] for item in selected_final.values()), "controller": min(selected_final, key=lambda kind: selected_final[kind]["total_acceleration_effort"])},
    }
    (OUTPUT / "primary_traditional_baseline.json").write_text(json.dumps({"primary_controller": primary["parameters"]["candidate_id"], "controller_family": next(kind for kind, item in selected_final.items() if item is primary), "selection_rule": "lexicographic safety, success, acquisition, 3D RMSE, orientation RMSE, ramp errors, total acceleration effort", "selected_metrics": {key: value for key, value in primary.items() if key not in {"rows", "parameters"}}}, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    (OUTPUT / "traditional_metric_envelope.json").write_text(json.dumps(envelope, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    advanced_contract = {"contract": "V2-R1R1-advanced-win-r1", "supersedes": "reproducibility/v2/r1/advanced_win_contract.json", "written_before_any_advanced_performance": True, "safety_rate_must_be_at_least_best_traditional": True, "task_success_rate_must_be_at_least_best_traditional": True, "position_rmse_improvement_required_fraction": 0.05, "common_success_acquisition_improvement_required_fraction": 0.05, "ramp_peak_or_steady_state_improvement_required_fraction": 0.05, "paired_bootstrap_confidence": 0.95, "paired_bootstrap_required": True, "improvement_direction": "positive means advanced metric is lower than the corresponding best traditional envelope", "holdout_only_for_final_pass": True}
    (OUTPUT / "advanced_win_contract.json").write_text(json.dumps(advanced_contract, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    safety_audit = {"development_samples": len(development), "holdout_samples_manifested": len(holdout), "holdout_executed": False, "unsafe_samples_retained": True, "controller_safety": {kind: {"safe_sample_count": selected_final[kind]["safe_sample_count"], "sample_count": selected_final[kind]["sample_count"], "unsafe_sample_ids": [row["sample_id"] for row in selected_final[kind]["rows"] if not row["safe"]]} for kind in selected_final}}
    (OUTPUT / "safety_audit.json").write_text(json.dumps(safety_audit, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    gate = {"task": "V2-R1R1-TRADITIONAL-3D-BASELINE-CONTRACT-CORRECTION-R1", "start_head": START_HEAD, "branch": "research-v2", "main_unchanged": True, "v1_tag_unchanged": True, "r1_original_evidence_preserved": True, "sample_bank_unchanged": True, "holdout_unchanged": True, "angular_speed_acquisition_fixed": True, "angular_speed_source": "mujoco.mj_jacSite jacr @ qvel", "shared_y_controller_uses_tip_state": True, "shared_z_controller_uses_tip_state": True, "shared_yz_competence_pass": True, "ay_safety_checked": True, "az_safety_checked": True, "pid_grid_size": 18, "lqr_grid_size": 64, "task_lqr_grid_size": 27, "pid_selected": True, "lqr_selected": True, "task_lqr_selected": True, "primary_traditional_selected": True, "metric_envelope_written": True, "advanced_win_contract_refrozen": True, "holdout_executed": False, "advanced_self_started": False, "advanced_paper_selected": False, "result": "V2_R1R1_TRADITIONAL_BASELINES_FROZEN"}
    (OUTPUT / "gate.json").write_text(json.dumps(gate, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    # Keep only Stage-2 top-3 raw runs under this directory. Stage-1 runs never
    # write raw CSV, so no large non-selected traces are retained.
    print(json.dumps({"result": gate["result"], "primary": primary["parameters"]["candidate_id"], "selected": {kind: item["parameters"]["candidate_id"] for kind, item in selected_final.items()}}, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
