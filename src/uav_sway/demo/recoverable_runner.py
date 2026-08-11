"""P3-R1E recoverable stress envelope and final showcase harness.

This module deliberately reuses the frozen P3-R1D plant/controller bridge.  It
only changes the preregistered showcase inputs (6 s move, 5 m/s in-motion X
wind, and an 3--10 m/s X-wind envelope) and exposes the already frozen outer
limiter diagnostics.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind_world
from uav_sway.models.model_config import load_model_config
from uav_sway.native_stack.actuation import CanonicalWrenchActuator
from uav_sway.native_stack.api import ReferenceSample, WrenchCommand
from uav_sway.native_stack.r1r1_controllers import LegacyTaskLevelAdapter
from uav_sway.native_stack.sensors import NativeSensorReader
from uav_sway.task_space.state import CutterTaskSpaceReader

ROOT = Path(__file__).resolve().parents[3]
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
MODEL_SHA256 = "19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d"
OUT = ROOT / "outputs/meeting_demo_recoverable_v4"
ART = ROOT / "artifacts/meeting_demo_recoverable_v4"
DOC = ROOT / "docs/clean_release"
DT, INNER_STRIDE, OUTER_STRIDE = 0.001, 5, 50
DURATION = 40.0
INITIAL_ANGLES_DEG = np.array([20.0, -16.0, 12.0, -8.0, 4.0])
TARGET_DELTA = np.array([2.0, 1.7, 4.5])
CONTROLLERS = ("full_lqr_048", "satc_b_027")
WIND_SPEEDS = (3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0)


def quintic(u: float) -> tuple[float, float, float]:
    u = float(np.clip(u, 0.0, 1.0))
    p = 10 * u**3 - 15 * u**4 + 6 * u**5
    v = 30 * u**2 - 60 * u**3 + 30 * u**4
    a = 60 * u - 180 * u**2 + 120 * u**3
    if u <= 0.0 or u >= 1.0:
        v = a = 0.0
    return p, v, a


def six_second_reference(p0: np.ndarray, target: np.ndarray, t: float, move_duration: float = 6.0) -> ReferenceSample:
    """Minimum-jerk reference retained for legacy R1E and parameterized for R1F."""
    move_duration = float(move_duration)
    move_end = 1.0 + move_duration
    if t < 1.0:
        return ReferenceSample(p0, np.zeros(3), np.zeros(3), np.zeros(3), t)
    if t >= move_end:
        return ReferenceSample(target, np.zeros(3), np.zeros(3), np.zeros(3), t)
    p, v, a = quintic((t - 1.0) / move_duration)
    d = target - p0
    return ReferenceSample(p0 + p * d, v * d / move_duration, a * d / move_duration**2, np.zeros(3), t)


def wind_profile(task: str, t: float, speed: float = 0.0) -> np.ndarray:
    if task == "T2":
        onset, ramp = 3.0, 1.0
        speed = 5.0 if speed == 0.0 else float(speed)
    elif task == "T3":
        onset, ramp = 8.0, 1.0
    else:
        return np.zeros(3)
    if t < onset:
        return np.zeros(3)
    factor = 1.0 if t >= onset + ramp else 0.5 * (1.0 - math.cos(math.pi * (t - onset) / ramp))
    return np.array([float(speed) * factor, 0.0, 0.0])


def continuous_time(times: np.ndarray, mask: np.ndarray, hold_s: float, start_s: float) -> float | None:
    if len(times) < 2:
        return None
    dt = float(np.median(np.diff(times)))
    needed = max(1, int(math.ceil(hold_s / dt)))
    count = 0
    for i, ok in enumerate(mask):
        if times[i] < start_s:
            count = 0
            continue
        count = count + 1 if bool(ok) else 0
        if count >= needed:
            return float(times[i - needed + 1])
    return None


def _rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    return (math.atan2(float(rotation[2, 1]), float(rotation[2, 2])),
            math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0))),
            math.atan2(float(rotation[1, 0]), float(rotation[0, 0])))


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def jobs() -> list[dict[str, Any]]:
    result = [{"task": "T1", "controller": c} for c in CONTROLLERS]
    result += [{"task": "T2", "controller": c} for c in CONTROLLERS]
    result += [{"task": "T3", "speed_mps": s, "controller": c} for s in WIND_SPEEDS for c in CONTROLLERS]
    return result


def _initialise(task: str, model: mujoco.MjModel, data: mujoco.MjData, qaddr: list[int]) -> None:
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    if task in ("T1", "T2"):
        data.qpos[qaddr] = np.deg2rad(INITIAL_ANGLES_DEG)
    data.qvel[:] = 0.0; data.ctrl[:] = 0.0; data.eq_active[:] = 0
    mujoco.mj_forward(model, data)


def _outer_truth(controller: LegacyTaskLevelAdapter, previous: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, float]:
    """Return raw, post-limit, amplitude-limit, saturation, slew and limits."""
    hist = controller.legacy_controller
    diag = getattr(hist, "diagnostics", None)
    raw = np.asarray(getattr(diag, "raw_command", controller._acceleration), dtype=float).reshape(3)
    amplitude = np.asarray(getattr(diag, "amplitude_limited", controller._acceleration), dtype=float).reshape(3)
    command = np.asarray(getattr(diag, "command", controller._acceleration), dtype=float).reshape(3)
    saturated = np.asarray(getattr(diag, "saturated", np.abs(raw) > 2.0), dtype=bool).reshape(3)
    slew = np.asarray(getattr(diag, "slew_limited", np.abs(command - previous) >= 0.25 - 1e-9), dtype=bool).reshape(3)
    limiter = getattr(hist, "limiter", None)
    absolute = float(getattr(limiter, "absolute_limit_m_s2", 2.0))
    slew_limit = float(getattr(limiter, "slew_limit_m_s2_per_update", 0.25))
    hit_abs = np.abs(command) >= absolute - 1e-6
    delta = np.abs(command - previous)
    hit_slew = delta >= slew_limit - 1e-6
    return raw, command, amplitude, np.logical_or(saturated, hit_abs), np.logical_or(slew, hit_slew), absolute, slew_limit


def _run_job(job: dict[str, Any]) -> dict[str, Any]:
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[key] = "1"
    digest = hashlib.sha256(MODEL.read_bytes()).hexdigest()
    if digest != MODEL_SHA256:
        raise RuntimeError(f"BLOCK_MODEL_SHA_MISMATCH:{digest}")
    task, controller_id = str(job["task"]), str(job["controller"])
    speed = float(job.get("speed_mps", 0.0))
    move_duration = float(job.get("move_duration_s", 6.0))
    output_root = Path(job.get("output_root", OUT))
    model = mujoco.MjModel.from_xml_path(str(MODEL)); data = mujoco.MjData(model)
    jids = [int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}")) for i in range(1, 6)]
    qaddr = [int(model.jnt_qposadr[j]) for j in jids]; _initialise(task, model, data, qaddr)
    cfg = load_model_config(ROOT / "configs/model_5link.yaml"); aero = load_aerodynamic_config(ROOT / "configs/aerodynamics.yaml")
    sensor, reader, actuator = NativeSensorReader(model), CutterTaskSpaceReader(model), CanonicalWrenchActuator(model)
    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip")); quad_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    p0 = np.asarray(data.site_xpos[tip_id], dtype=float).copy(); target = p0 + TARGET_DELTA if task in ("T1", "T2") else p0.copy()
    controller = LegacyTaskLevelAdapter(controller_id, 0.0, 0.0, historical_id=controller_id); controller.reset()
    previous_wrench = WrenchCommand(0.0, np.zeros(3)); previous_outer = np.zeros(3); command = np.zeros(3)
    rows: list[dict[str, Any]] = []; render_t: list[float] = []; render_q: list[np.ndarray] = []
    outer_times: list[float] = []; inner_times: list[float] = []; outer_hits = np.zeros(3, dtype=int); slew_hits = np.zeros(3, dtype=int); outer_count = 0
    saturation_count = np.zeros(4, dtype=int); sample_count = 0; next_render = 0.0; started = time.perf_counter(); nsteps = int(round(DURATION / DT))
    safety: dict[str, Any] = {"finite": True, "nan_count": 0, "min_uav_height_m": float("inf"), "min_cutter_tip_height_m": float("inf"), "max_roll_pitch_deg": 0.0, "max_joint_angle_rad": 0.0, "violations": []}
    for tick in range(nsteps + 1):
        t = tick * DT; wind = np.zeros(3) if bool(job.get("zero_wind", False)) else wind_profile(task, t, speed)
        clear_and_apply_wind_world(model, data, cfg, aero, wind)
        ref = six_second_reference(p0, target, t, move_duration) if task in ("T1", "T2") else ReferenceSample(p0, np.zeros(3), np.zeros(3), np.zeros(3), t)
        packet = sensor.read(model, data, ref, previous_wrench, tick, DT); controller.observe(packet)
        raw = amplitude = np.zeros(3); accel_hit = slew_hit = np.zeros(3, dtype=bool); abs_lim = slew_lim = 0.0
        if tick % OUTER_STRIDE == 0:
            stamp = time.perf_counter_ns(); controller.update_high_level(); outer_times.append((time.perf_counter_ns() - stamp) / 1e6)
            raw, command, amplitude, accel_hit, slew_hit, abs_lim, slew_lim = _outer_truth(controller, previous_outer)
            previous_outer = command.copy(); outer_hits += accel_hit.astype(int); slew_hits += slew_hit.astype(int); outer_count += 1
        applied = None
        if tick % INNER_STRIDE == 0:
            stamp = time.perf_counter_ns(); controller.update_inner(); inner_times.append((time.perf_counter_ns() - stamp) / 1e6)
            requested = controller.physical_command(); applied = actuator.apply(data, requested, tick, DT); previous_wrench = applied.actual
            saturation_count += np.r_[applied.thrust_saturated, applied.torque_saturated].astype(int)
            state = reader.read(model, data); roll, pitch, yaw = _rpy(packet.rotation_world_from_body); q = np.asarray(packet.joint_position, dtype=float)
            finite_values = np.r_[packet.uav_position_world, packet.uav_velocity_world, q, packet.joint_velocity, state.tip_position_world, state.tip_velocity_world, raw, command, requested.as_array(), applied.actual.as_array(), roll, pitch, yaw]
            finite = bool(np.isfinite(finite_values).all()); safety["finite"] &= finite; safety["nan_count"] += int(np.count_nonzero(~np.isfinite(finite_values)))
            safety["min_uav_height_m"] = min(safety["min_uav_height_m"], float(packet.uav_position_world[2])); safety["min_cutter_tip_height_m"] = min(safety["min_cutter_tip_height_m"], float(state.tip_position_world[2])); safety["max_roll_pitch_deg"] = max(safety["max_roll_pitch_deg"], abs(math.degrees(roll)), abs(math.degrees(pitch))); safety["max_joint_angle_rad"] = max(safety["max_joint_angle_rad"], float(np.max(np.abs(q))))
            if not finite: safety["violations"].append({"time_s": t, "reason": "non_finite"})
            if packet.uav_position_world[2] <= 0.0: safety["violations"].append({"time_s": t, "reason": "uav_height_nonpositive"})
            if state.tip_position_world[2] <= 0.0: safety["violations"].append({"time_s": t, "reason": "tip_height_nonpositive"})
            rows.append({"time": t, "uav_x": packet.uav_position_world[0], "uav_y": packet.uav_position_world[1], "uav_z": packet.uav_position_world[2], "tip_x": state.tip_position_world[0], "tip_y": state.tip_position_world[1], "tip_z": state.tip_position_world[2], "tip_vx": state.tip_velocity_world[0], "tip_vy": state.tip_velocity_world[1], "tip_vz": state.tip_velocity_world[2], "ref_x": ref.position_world[0], "ref_y": ref.position_world[1], "ref_z": ref.position_world[2], "wind_x": wind[0], "wind_y": wind[1], "wind_z": wind[2], "uav_roll_deg": math.degrees(roll), "uav_pitch_deg": math.degrees(pitch), "uav_yaw_deg": math.degrees(yaw), "raw_ax": raw[0], "raw_ay": raw[1], "raw_az": raw[2], "ax_cmd": command[0], "ay_cmd": command[1], "az_cmd": command[2], "ax_limit": abs_lim, "ay_limit": abs_lim, "az_limit": abs_lim, "ax_at_limit": bool(accel_hit[0]), "ay_at_limit": bool(accel_hit[1]), "az_at_limit": bool(accel_hit[2]), "any_accel_at_limit": bool(np.any(accel_hit)), "slew_ax_hit": bool(slew_hit[0]), "slew_ay_hit": bool(slew_hit[1]), "slew_az_hit": bool(slew_hit[2]), "any_slew_at_limit": bool(np.any(slew_hit)), "requested_thrust_N": requested.thrust_N, "applied_thrust_N": applied.actual.thrust_N, "requested_Mx_Nm": requested.torque_Nm[0], "requested_My_Nm": requested.torque_Nm[1], "requested_Mz_Nm": requested.torque_Nm[2], "applied_Mx_Nm": applied.actual.torque_Nm[0], "applied_My_Nm": applied.actual.torque_Nm[1], "applied_Mz_Nm": applied.actual.torque_Nm[2], "thrust_saturated": applied.thrust_saturated, "mx_saturated": bool(applied.torque_saturated[0]), "my_saturated": bool(applied.torque_saturated[1]), "mz_saturated": bool(applied.torque_saturated[2]), "safe": finite, **{f"q{i+1}": q[i] for i in range(5)}, **{f"qd{i+1}": packet.joint_velocity[i] for i in range(5)}})
            sample_count += 1
        if t + 1e-9 >= next_render:
            render_t.append(t); render_q.append(data.qpos.copy()); next_render += 1.0 / 30.0
        if tick < nsteps: mujoco.mj_step(model, data)
    runtime = time.perf_counter() - started
    times = np.asarray([r["time"] for r in rows], dtype=float); tip = np.asarray([[r["tip_x"], r["tip_y"], r["tip_z"]] for r in rows]); ref_arr = np.asarray([[r["ref_x"], r["ref_y"], r["ref_z"]] for r in rows]); vel_vec = np.asarray([[r["tip_vx"], r["tip_vy"], r["tip_vz"]] for r in rows]); vel = np.linalg.norm(vel_vec, axis=1); err = np.linalg.norm(tip - ref_arr, axis=1); dev = np.linalg.norm(tip - p0, axis=1); q = np.asarray([[r[f"q{i}"] for i in range(1, 6)] for r in rows]); jrms = np.sqrt(np.mean(q*q, axis=1)); effort = np.asarray([sum(float(r[f"a{a}_cmd"])**2 for a in "xyz") for r in rows]); integ = float(np.trapezoid(effort, times) if hasattr(np, "trapezoid") else np.trapz(effort, times))
    sat_rates = saturation_count / max(1, sample_count); no_safety = bool(safety["finite"] and not safety["violations"]); common: dict[str, Any] = {"task": task, "controller": controller_id, "speed_mps": speed, "duration_s": DURATION, "move_duration_s": move_duration, "initial_angles_deg": INITIAL_ANGLES_DEG.tolist() if task in ("T1", "T2") else [0.0]*5, "target_delta_m": TARGET_DELTA.tolist() if task in ("T1", "T2") else [0.0]*3, "wind_axis": "+X" if task in ("T2", "T3") else "OFF", "safety": safety, "runtime_s": runtime, "runtime_mean_ms": float(np.mean(outer_times + inner_times)), "runtime_p95_ms": float(np.percentile(outer_times + inner_times, 95)), "control_effort": integ, "physical_wrench_saturation_rate": float(np.mean(sat_rates)), "outer_accel_limit_hit_rate": float(np.mean(outer_hits / max(1, outer_count))), "outer_accel_axis_hit_rate": (outer_hits / max(1, outer_count)).tolist(), "outer_slew_limit_hit_rate": float(np.mean(slew_hits / max(1, outer_count))), "outer_slew_axis_hit_rate": (slew_hits / max(1, outer_count)).tolist(), "max_roll_deg": float(max(abs(float(r["uav_roll_deg"])) for r in rows)), "max_pitch_deg": float(max(abs(float(r["uav_pitch_deg"])) for r in rows)), "peak_joint_rms_deg": float(np.degrees(np.max(jrms))), "peak_joint_angle_deg": float(np.degrees(np.max(np.abs(q)))), "final_5s_joint_rms_deg": float(np.degrees(np.sqrt(np.mean(q[times >= 35.0]**2)))), "finite": no_safety}
    if task == "T1":
        final5_tip = float(np.sqrt(np.mean(err[times >= 35.0] ** 2))); final5_joint = common["final_5s_joint_rms_deg"]
        valid = continuous_time(times, (err <= 0.15) & (vel <= 0.20), 1.0, 1.0 + move_duration); stable = bool(no_safety and err[-1] <= 0.15 and vel[-1] <= 0.20 and final5_tip <= 0.20 and final5_joint <= 1.0 and valid is not None)
        common.update({"tip_tracking_rmse_m": float(np.sqrt(np.mean(err*err))), "peak_tip_error_m": float(np.max(err)), "final_tip_error_m": float(err[-1]), "final_tip_speed_mps": float(vel[-1]), "final_5s_tip_rms_m": final5_tip, "settling_after_move_s": None if valid is None else valid - (1.0 + move_duration), "STABLE_RECOVERED": stable, "classification": "STABLE_RECOVERED" if stable else "SAFETY_FAILURE" if not no_safety else "CONTROLLED_BUT_NOT_STABLE"})
    elif task == "T2":
        peak_idx = int(np.argmax(np.where(times >= 3.0, err, -np.inf))); valid = continuous_time(times, (err <= 0.15) & (vel <= 0.20), 1.0, max(4.0, times[peak_idx])); final5 = float(np.sqrt(np.mean(err[times >= 35.0]**2))); final5_joint = common["final_5s_joint_rms_deg"]; stable = bool(no_safety and err[-1] <= 0.15 and vel[-1] <= 0.20 and final5 <= 0.20 and final5_joint <= 1.0 and valid is not None and valid >= 4.0)
        common.update({"wind_onset_time_s": 3.0, "peak_error_after_wind_m": float(err[peak_idx]), "time_to_peak_after_wind_s": float(times[peak_idx]-3.0), "recovery_after_peak_s": None if valid is None else float(valid-times[peak_idx]), "recovery_from_onset_s": None if valid is None else float(valid-3.0), "final_error_under_wind_m": float(err[-1]), "postwind_tip_rms_m": float(np.sqrt(np.mean(err[times >= 4.0]**2))), "final_5s_tip_rms_m": final5, "final_5s_tip_speed_rms_mps": float(np.sqrt(np.mean(vel[times >= 35.0]**2))), "STABLE_RECOVERED": stable, "classification": "STABLE_RECOVERED" if stable else "SAFETY_FAILURE" if not no_safety else "NOT_RECOVERED"})
    else:
        peak_idx = int(np.argmax(np.where(times >= 8.0, dev, -np.inf))); valid = continuous_time(times, (dev <= 0.15) & (vel <= 0.20), 1.0, float(times[peak_idx])); final5 = float(np.sqrt(np.mean(dev[times >= 35.0]**2))); final5_speed = float(np.sqrt(np.mean(vel[times >= 35.0]**2)))
        recoverable = bool(no_safety and valid is not None and final5 <= 0.20 and final5_speed <= 0.20)
        common.update({"wind_onset_time_s": 8.0, "prewind_tip_rms_m": float(np.sqrt(np.mean(dev[times < 8.0]**2))), "postwind_tip_rms_m": float(np.sqrt(np.mean(dev[times >= 9.0]**2))), "peak_tip_deviation_m": float(dev[peak_idx]), "peak_time_s": float(times[peak_idx]), "recovery_after_peak_s": None if valid is None else float(valid-times[peak_idx]), "recovery_from_wind_onset_s": None if valid is None else float(valid-8.0), "final_5s_tip_rms_m": final5, "final_5s_tip_speed_rms_mps": final5_speed, "recoverable": recoverable, "classification": "RECOVERABLE" if recoverable else "NOT_RECOVERABLE"})
    case_dir = str(job.get("case_dir", f"{int(speed):02d}mps" if task == "T3" else "")); path = output_root / task / case_dir / controller_id if case_dir else output_root / task / controller_id; path.mkdir(parents=True, exist_ok=True); _write_csv(path / "run.csv", rows); (path / "metrics.json").write_text(json.dumps(common, indent=2, allow_nan=False)+"\n", encoding="utf-8"); np.savez_compressed(path / "render_states.npz", time=np.asarray(render_t), qpos=np.asarray(render_q))
    return {"job": job, "metrics": common, "runtime_s": runtime, "path": str(path)}


def _overlay(frame: np.ndarray, text: str) -> np.ndarray:
    from PIL import Image, ImageDraw
    image = Image.fromarray(frame); draw = ImageDraw.Draw(image); draw.rectangle((8, 8, 500, 38), fill=(0, 0, 0)); draw.text((16, 14), text, fill=(255, 255, 255)); return np.asarray(image)


def _render(path: Path, filename: str, label: str) -> str:
    import imageio.v2 as imageio
    states = np.load(path / "render_states.npz"); model = mujoco.MjModel.from_xml_path(str(MODEL)); data = mujoco.MjData(model); renderer = mujoco.Renderer(model, height=720, width=1280); frames = []
    for i, qpos in enumerate(states["qpos"]):
        data.qpos[:] = qpos; mujoco.mj_forward(model, data); renderer.update_scene(data, camera="oblique_camera"); frames.append(_overlay(renderer.render().copy(), f"{label}  Time {float(states['time'][i]):.2f}s"))
    renderer.close() if hasattr(renderer, "close") else None; out = path / filename
    try: imageio.mimsave(out, frames, fps=30, codec="libx264"); return str(out)
    except Exception:
        fallback = out.with_suffix(".gif"); imageio.mimsave(fallback, frames, fps=30); return str(fallback)


def _side_by_side(left: Path, right: Path, out: Path, label_left: str, label_right: str) -> str:
    import imageio.v2 as imageio
    a, b = imageio.get_reader(left), imageio.get_reader(right); writer = imageio.get_writer(out, fps=30, codec="libx264")
    for fa, fb in zip(a, b): writer.append_data(np.concatenate([_overlay(fa, label_left), _overlay(fb, label_right)], axis=1))
    writer.close(); a.close(); b.close(); return str(out)


def _plot_run(result: dict[str, Any]) -> None:
    path = Path(result["path"]); rows = list(csv.DictReader((path / "run.csv").open(encoding="utf-8"))); t = np.asarray([float(r["time"]) for r in rows]); tip = np.asarray([[float(r[f"tip_{a}"]) for a in "xyz"] for r in rows]); ref = np.asarray([[float(r[f"ref_{a}"]) for a in "xyz"] for r in rows]); q = np.asarray([[float(r[f"q{i}"]) for i in range(1, 6)] for r in rows]); dev = np.linalg.norm(tip-ref, axis=1)
    fig, ax = plt.subplots(2, 2, figsize=(9, 6)); ax[0,0].plot(t, dev); ax[0,0].set_title("tip error / deviation"); ax[0,1].plot(t, q); ax[0,1].set_title("joint angles"); ax[1,0].plot(t, [float(r["ax_cmd"]) for r in rows], label="ax"); ax[1,0].plot(t, [float(r["ay_cmd"]) for r in rows], label="ay"); ax[1,0].plot(t, [float(r["az_cmd"]) for r in rows], label="az"); ax[1,0].set_title("outer acceleration"); ax[1,1].plot(t, [float(r["wind_x"]) for r in rows]); ax[1,1].set_title("wind X (m/s)"); [a.grid(True) for a in ax.flat]; ax[1,0].legend(fontsize=7); fig.tight_layout(); fig.savefig(path / "timeseries.png", dpi=130); plt.close(fig)


def _make_envelope(results: list[dict[str, Any]]) -> dict[str, Any]:
    by = {(float(r["job"]["speed_mps"]), r["job"]["controller"]): r["metrics"] for r in results if r["job"]["task"] == "T3"}
    rows = []
    for speed in WIND_SPEEDS:
        l, s = by[(speed, CONTROLLERS[0])], by[(speed, CONTROLLERS[1])]
        rows.append((speed, l, s))
    max_l = max([speed for speed, l, _ in rows if l["recoverable"]], default=None); max_s = max([speed for speed, _, s in rows if s["recoverable"]], default=None)
    first_l = next((speed for speed, l, _ in rows if not l["recoverable"]), None); first_s = next((speed for speed, _, s in rows if not s["recoverable"]), None)
    lines = ["# Wind rejection envelope", "", "DEMO CAPABILITY ENVELOPE ONLY; not a holdout claim.", "", "| Wind (m/s) | LQR recoverable | SATC recoverable | LQR RMS (m) | SATC RMS (m) | LQR recovery (s) | SATC recovery (s) | LQR accel hit | SATC accel hit | safety |", "|---:|:---:|:---:|---:|---:|---:|---:|---:|---:|:---|"]
    for speed, l, s in rows:
        lines.append(f"| {speed:g} | {l['recoverable']} | {s['recoverable']} | {l['postwind_tip_rms_m']:.6g} | {s['postwind_tip_rms_m']:.6g} | {l['recovery_after_peak_s']} | {s['recovery_after_peak_s']} | {l['outer_accel_limit_hit_rate']:.6g} | {s['outer_accel_limit_hit_rate']:.6g} | {l['finite']}/{s['finite']} |")
    (OUT / "WIND_REJECTION_ENVELOPE.md").write_text("\n".join(lines)+"\n", encoding="utf-8")
    metrics = {"speeds_mps": list(WIND_SPEEDS), "LQR_MAX_RECOVERABLE_WIND_MPS": max_l, "SATC_MAX_RECOVERABLE_WIND_MPS": max_s, "LQR_FIRST_FAILURE_MPS": first_l, "SATC_FIRST_FAILURE_MPS": first_s}
    for name, key, ylabel in [("wind_envelope_postwind_rms.png", "postwind_tip_rms_m", "postwind RMS (m)"), ("wind_envelope_peak_error.png", "peak_tip_deviation_m", "peak deviation (m)"), ("wind_envelope_recovery_time.png", "recovery_after_peak_s", "recovery (s)"), ("wind_envelope_final5s_rms.png", "final_5s_tip_rms_m", "final 5 s RMS (m)"), ("wind_envelope_accel_limit_hit.png", "outer_accel_limit_hit_rate", "accel limit hit rate"), ("wind_envelope_wrench_saturation.png", "physical_wrench_saturation_rate", "wrench saturation rate")]:
        fig, ax = plt.subplots(figsize=(7,4)); ax.plot(WIND_SPEEDS, [by[(s, CONTROLLERS[0])][key] if by[(s, CONTROLLERS[0])][key] is not None else np.nan for s in WIND_SPEEDS], "o-", label="LQR"); ax.plot(WIND_SPEEDS, [by[(s, CONTROLLERS[1])][key] if by[(s, CONTROLLERS[1])][key] is not None else np.nan for s in WIND_SPEEDS], "o-", label="SATC"); ax.set_xlabel("wind speed [m/s]"); ax.set_ylabel(ylabel); ax.grid(True); ax.legend(); fig.tight_layout(); fig.savefig(OUT / name, dpi=150); plt.close(fig)
    fig, ax = plt.subplots(figsize=(7,4)); ax.plot(WIND_SPEEDS, [int(by[(s, CONTROLLERS[0])]["finite"]) for s in WIND_SPEEDS], "o-", label="LQR safety"); ax.plot(WIND_SPEEDS, [int(by[(s, CONTROLLERS[1])]["finite"]) for s in WIND_SPEEDS], "o-", label="SATC safety"); ax.set_ylim(-.1,1.1); ax.set_yticks([0,1]); ax.set_yticklabels(["FAIL","PASS"]); ax.set_xlabel("wind speed [m/s]"); ax.grid(True); ax.legend(); fig.tight_layout(); fig.savefig(OUT / "wind_envelope_safety.png", dpi=150); plt.close(fig)
    (ART / "wind_envelope_summary.json").write_text(json.dumps(metrics, indent=2)+"\n", encoding="utf-8"); return metrics


def _make_docs(results: list[dict[str, Any]], envelope: dict[str, Any]) -> None:
    by = {(r["job"]["task"], r["job"].get("speed_mps", 0), r["job"]["controller"]): r["metrics"] for r in results}
    t1l, t1s = by[("T1",0,"full_lqr_048")], by[("T1",0,"satc_b_027")]; t2l, t2s = by[("T2",0,"full_lqr_048")], by[("T2",0,"satc_b_027")]
    (OUT / "P3_R1D_CONTROL_LOSS_AUDIT.md").write_text("# P3-R1D control-loss postmortem\n\n- T1 LQR: TASK_CONTROL_LOST (large final error).\n- T1 SATC: SAFETY_FAILURE (historical 430 height violations).\n- T2: LARGE_EXCURSION_NO_RECOVERY.\n- T3: DISTURBANCE_OVERWHELMED_BOTH at 10 m/s.\n\nP3-R1D is retained as failure-boundary evidence, not a successful showcase.\n", encoding="utf-8")
    guide = f"""# Final showcase guide (CN)\n\nP3-R1D 是 failure-boundary evidence：4 s 大转场、初始 20° 摆动和 10 m/s 持续风超出当前任务控制能力。\n\nP3-R1E 是 controlled stress showcase：固定模型和固定控制器，6 s quintic 转场，T2 在转场中加入 5 m/s 世界 +X 风，T3 扫描 3--10 m/s 世界 +X 风。\n\n## 会议解读\n\n- T1 分类：LQR `{t1l['classification']}`，SATC `{t1s['classification']}`。\n- T2 分类：LQR `{t2l['classification']}`，SATC `{t2s['classification']}`。\n- 风包络：LQR 最大可恢复风速 `{envelope['LQR_MAX_RECOVERABLE_WIND_MPS']}` m/s；SATC 最大可恢复风速 `{envelope['SATC_MAX_RECOVERABLE_WIND_MPS']}` m/s。\n- 所有数字都属于 `DEMO CAPABILITY ENVELOPE ONLY`，不是 Holdout 或新的论文 claim。\n\nY/XY30 历史结果仍保留，但由于五个 y-axis hinge 的平面链模型，不作为核心多连杆消摆结论。\n"""
    (DOC / "FINAL_SHOWCASE_GUIDE_CN.md").write_text(guide, encoding="utf-8")
    fig, ax = plt.subplots(2,2, figsize=(10,7)); ax[0,0].bar(["LQR","SATC"],[t1l["final_tip_error_m"],t1s["final_tip_error_m"]]); ax[0,0].set_title("T1 final error (m)"); ax[0,1].bar(["LQR","SATC"],[t2l["postwind_tip_rms_m"],t2s["postwind_tip_rms_m"]]); ax[0,1].set_title("T2 postwind RMS (m)"); ax[1,0].plot(WIND_SPEEDS,[by[("T3",s,"full_lqr_048")]["postwind_tip_rms_m"] for s in WIND_SPEEDS],"o-",label="LQR"); ax[1,0].plot(WIND_SPEEDS,[by[("T3",s,"satc_b_027")]["postwind_tip_rms_m"] for s in WIND_SPEEDS],"o-",label="SATC"); ax[1,0].set_title("T3 wind envelope"); ax[1,0].legend(fontsize=7); ax[1,1].plot(WIND_SPEEDS,[by[("T3",s,"full_lqr_048")]["outer_accel_limit_hit_rate"] for s in WIND_SPEEDS],"o-",label="LQR"); ax[1,1].plot(WIND_SPEEDS,[by[("T3",s,"satc_b_027")]["outer_accel_limit_hit_rate"] for s in WIND_SPEEDS],"o-",label="SATC"); ax[1,1].set_title("outer accel limit hit rate"); ax[1,1].legend(fontsize=7); [a.grid(True) for a in ax.flat]; fig.tight_layout(); fig.savefig(OUT / "FINAL_SHOWCASE_SUMMARY.png", dpi=160); plt.close(fig)


def _render_showcase(results: list[dict[str, Any]], envelope: dict[str, Any]) -> dict[str, str]:
    by = {(r["job"]["task"], r["job"].get("speed_mps", 0), r["job"]["controller"]): r for r in results}; paths: dict[str,str] = {}
    for task, label in (("T1", "T1"), ("T2", "T2")):
        l, s = by[(task,0,"full_lqr_048")], by[(task,0,"satc_b_027")]; lp = _render(Path(l["path"]), f"{label}_LQR.mp4", f"{label} LQR"); sp = _render(Path(s["path"]), f"{label}_SATC.mp4", f"{label} SATC"); paths[f"{task}_lqr"] = lp; paths[f"{task}_satc"] = sp; paths[f"{task}_side"] = _side_by_side(Path(lp), Path(sp), OUT / task / f"{label}_LQR_vs_SATC.mp4", f"{label} LQR {l['metrics']['classification']}", f"{label} SATC {s['metrics']['classification']}")
    max_speed = envelope["SATC_MAX_RECOVERABLE_WIND_MPS"] or 5.0; chosen = sorted({3.0,5.0,float(max_speed)})
    for speed in chosen:
        l, s = by[("T3",speed,"full_lqr_048")], by[("T3",speed,"satc_b_027")]; lp = _render(Path(l["path"]), f"T3_{int(speed)}mps_LQR.mp4", f"T3 {speed:g}m/s LQR"); sp = _render(Path(s["path"]), f"T3_{int(speed)}mps_SATC.mp4", f"T3 {speed:g}m/s SATC"); paths[f"T3_{speed:g}_side"] = _side_by_side(Path(lp), Path(sp), OUT / "T3" / f"T3_{int(speed)}mps_LQR_vs_SATC.mp4", f"{speed:g}m/s LQR {l['metrics']['classification']}", f"{speed:g}m/s SATC {s['metrics']['classification']}")
    return paths


def run_all() -> dict[str, Any]:
    if hashlib.sha256(MODEL.read_bytes()).hexdigest() != MODEL_SHA256: raise RuntimeError("BLOCK_MODEL_SHA_MISMATCH")
    OUT.mkdir(parents=True, exist_ok=True); ART.mkdir(parents=True, exist_ok=True); registry = jobs(); (ART / "job_registry.json").write_text(json.dumps(registry, indent=2)+"\n", encoding="utf-8")
    cores = os.cpu_count() or 1; workers = min(20, max(1, cores-4)); started = time.perf_counter(); results = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        future_map = {pool.submit(_run_job, job): job for job in registry}
        for future in as_completed(future_map): results.append(future.result())
    wall = time.perf_counter()-started; serial = sum(float(r["runtime_s"]) for r in results); (ART / "parallel_execution_audit.json").write_text(json.dumps({"cpu_count":cores,"max_workers":workers,"job_count":len(registry),"total_wall_time_s":wall,"serial_time_sum_s":serial,"parallel_speedup_estimate":serial/wall if wall else None}, indent=2)+"\n", encoding="utf-8")
    for result in results: _plot_run(result)
    envelope = _make_envelope(results); videos = _render_showcase(results, envelope); _make_docs(results, envelope)
    authority = {"controller_ids": list(CONTROLLERS), "model_sha256": MODEL_SHA256, "limits_read_from_frozen_controller": True, "acceleration_limits_m_s2": {c: 2.0 for c in CONTROLLERS}, "slew_limits_m_s2_per_outer_update": {c: 0.25 for c in CONTROLLERS}, "outer_logging": ["raw_command","post_limit_command","axis_limit_hit","slew_limit_hit"]}
    (ART / "outer_authority_audit.json").write_text(json.dumps(authority, indent=2)+"\n", encoding="utf-8")
    (ART / "run_manifest.json").write_text(json.dumps({"task":"P3-R1E","jobs":len(results),"model_sha256":MODEL_SHA256,"controller_retuned":False,"model_modified":False,"holdout_executed":False,"videos":videos,"envelope":envelope}, indent=2)+"\n", encoding="utf-8")
    return {"results":results,"cpu_count":cores,"workers":workers,"wall_time_s":wall,"speedup":serial/wall if wall else None,"envelope":envelope,"videos":videos}


if __name__ == "__main__":
    print(json.dumps(run_all(), indent=2, default=str))
