"""P3-R1D extreme composite functional stress demos.

The plant and controller identities are frozen.  This harness only changes the
three explicitly registered stress inputs and records attitude/actuator truth
from the MuJoCo state and canonical wrench bridge.
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
OUT = ROOT / "outputs/meeting_demo_extreme_v3"
ART = ROOT / "artifacts/meeting_demo_extreme_v3"
DOC = ROOT / "docs/clean_release"
DT, INNER_STRIDE, OUTER_STRIDE = 0.001, 5, 50
DURATION = 40.0
INITIAL_ANGLES_DEG = np.array([20.0, -16.0, 12.0, -8.0, 4.0])
TARGET_DELTA = np.array([2.0, 1.7, 4.5])
CONTROLLERS = ("full_lqr_048", "satc_b_027")
WIND_VECTORS_5 = {
    "X": np.array([5.0, 0.0, 0.0]),
    "Y": np.array([0.0, 5.0, 0.0]),
    "XY30": np.array([4.330127018922193, 2.5, 0.0]),
}
WIND_VECTORS_10 = {
    "X": np.array([10.0, 0.0, 0.0]),
    "Y": np.array([0.0, 10.0, 0.0]),
    "XY30": np.array([8.660254037844386, 5.0, 0.0]),
}


def quintic(u: float) -> tuple[float, float, float]:
    u = float(np.clip(u, 0.0, 1.0))
    p = 10 * u**3 - 15 * u**4 + 6 * u**5
    v = 30 * u**2 - 60 * u**3 + 30 * u**4
    a = 60 * u - 180 * u**2 + 120 * u**3
    if u <= 0 or u >= 1:
        v = a = 0.0
    return p, v, a


def aggressive_reference(p0: np.ndarray, target: np.ndarray, t: float) -> ReferenceSample:
    if t < 1.0:
        return ReferenceSample(p0, np.zeros(3), np.zeros(3), np.zeros(3), t)
    if t >= 5.0:
        return ReferenceSample(target, np.zeros(3), np.zeros(3), np.zeros(3), t)
    p, v, a = quintic((t - 1.0) / 4.0)
    d = target - p0
    return ReferenceSample(p0 + p * d, v * d / 4.0, a * d / 16.0, np.zeros(3), t)


def wind_profile(direction: str, t: float, speed: float = 10.0) -> np.ndarray:
    vectors = WIND_VECTORS_10 if speed == 10.0 else WIND_VECTORS_5
    onset = 8.0 if speed == 10.0 else 2.5
    ramp_s = 1.0 if speed == 10.0 else 1.0
    if direction not in vectors or t < onset:
        return np.zeros(3)
    factor = 1.0 if t >= onset + ramp_s else 0.5 * (1.0 - math.cos(math.pi * (t - onset) / ramp_s))
    return vectors[direction] * factor


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
    roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    return roll, pitch, yaw


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def jobs() -> list[dict[str, str]]:
    result = [{"task": task, "controller": controller} for task in ("T1", "T2") for controller in CONTROLLERS]
    result += [{"task": "T3", "direction": direction, "controller": controller} for direction in ("X", "Y", "XY30") for controller in CONTROLLERS]
    return result


def _initialise(task: str, model, data, qaddr: list[int]) -> None:
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    if task in ("T1", "T2"):
        data.qpos[qaddr] = np.deg2rad(INITIAL_ANGLES_DEG)
    data.qvel[:] = 0.0; data.ctrl[:] = 0.0; data.eq_active[:] = 0
    mujoco.mj_forward(model, data)


def _run_job(job: dict[str, str]) -> dict[str, Any]:
    for key in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        os.environ[key] = "1"
    digest = hashlib.sha256(MODEL.read_bytes()).hexdigest()
    if digest != MODEL_SHA256:
        raise RuntimeError(f"BLOCK_MODEL_SHA_MISMATCH:{digest}")
    task, controller_id, direction = job["task"], job["controller"], job.get("direction", "")
    model, data = mujoco.MjModel.from_xml_path(str(MODEL)), None
    data = mujoco.MjData(model)
    jids = [int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}")) for i in range(1, 6)]
    qaddr = [int(model.jnt_qposadr[j]) for j in jids]
    _initialise(task, model, data, qaddr)
    cfg = load_model_config(ROOT / "configs/model_5link.yaml")
    aero = load_aerodynamic_config(ROOT / "configs/aerodynamics.yaml")
    sensor, reader, actuator = NativeSensorReader(model), CutterTaskSpaceReader(model), CanonicalWrenchActuator(model)
    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    quad_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    p0 = np.asarray(data.site_xpos[tip_id], dtype=float).copy(); uav0 = np.asarray(data.xpos[quad_id], dtype=float).copy()
    target = p0 + TARGET_DELTA if task in ("T1", "T2") else p0.copy()
    controller = LegacyTaskLevelAdapter(controller_id, 0.0, 0.0, historical_id=controller_id); controller.reset()
    previous = WrenchCommand(0.0, np.zeros(3)); command = np.zeros(3)
    rows: list[dict[str, Any]] = []; render_t: list[float] = []; render_q: list[np.ndarray] = []
    outer_ms: list[float] = []; inner_ms: list[float] = []; saturation_count = np.zeros(4, dtype=int); sample_count = 0
    safety: dict[str, Any] = {"finite": True, "nan_count": 0, "min_uav_height_m": float("inf"), "min_cutter_tip_height_m": float("inf"), "max_roll_pitch_deg": 0.0, "max_joint_angle_rad": 0.0, "violations": []}
    next_render = 0.0; started = time.perf_counter(); nsteps = int(round(DURATION / DT))
    for tick in range(nsteps + 1):
        t = tick * DT
        wind = np.zeros(3)
        if task == "T2": wind = wind_profile("XY30", t, 5.0)
        elif task == "T3": wind = wind_profile(direction, t, 10.0)
        clear_and_apply_wind_world(model, data, cfg, aero, wind)
        ref = aggressive_reference(p0, target, t) if task in ("T1", "T2") else ReferenceSample(p0, np.zeros(3), np.zeros(3), np.zeros(3), t)
        packet = sensor.read(model, data, ref, previous, tick, DT); controller.observe(packet)
        if tick % OUTER_STRIDE == 0:
            stamp = time.perf_counter_ns(); controller.update_high_level(); outer_ms.append((time.perf_counter_ns() - stamp) / 1e6)
            command = np.asarray(getattr(controller, "_acceleration", command), dtype=float).copy()
        applied = None
        if tick % INNER_STRIDE == 0:
            stamp = time.perf_counter_ns(); controller.update_inner(); inner_ms.append((time.perf_counter_ns() - stamp) / 1e6)
            requested = controller.physical_command(); applied = actuator.apply(data, requested, tick, DT); previous = applied.actual
            saturation_count += np.r_[applied.thrust_saturated, applied.torque_saturated].astype(int)
            state = reader.read(model, data); roll, pitch, yaw = _rpy(packet.rotation_world_from_body)
            finite_values = np.r_[packet.uav_position_world, packet.uav_velocity_world, packet.joint_position, packet.joint_velocity, state.tip_position_world, state.tip_velocity_world, command, requested.as_array(), applied.actual.as_array(), roll, pitch, yaw]
            finite = bool(np.isfinite(finite_values).all()); safety["finite"] &= finite; safety["nan_count"] += int(np.count_nonzero(~np.isfinite(finite_values)))
            safety["min_uav_height_m"] = min(safety["min_uav_height_m"], float(packet.uav_position_world[2])); safety["min_cutter_tip_height_m"] = min(safety["min_cutter_tip_height_m"], float(state.tip_position_world[2])); safety["max_roll_pitch_deg"] = max(safety["max_roll_pitch_deg"], abs(math.degrees(roll)), abs(math.degrees(pitch))); safety["max_joint_angle_rad"] = max(safety["max_joint_angle_rad"], float(np.max(np.abs(packet.joint_position))))
            if not finite: safety["violations"].append({"time_s": t, "reason": "non_finite"})
            if packet.uav_position_world[2] <= 0.0: safety["violations"].append({"time_s": t, "reason": "uav_height_nonpositive"})
            if state.tip_position_world[2] <= 0.0: safety["violations"].append({"time_s": t, "reason": "tip_height_nonpositive"})
            q = packet.joint_position
            rows.append({"time": t, "uav_x": packet.uav_position_world[0], "uav_y": packet.uav_position_world[1], "uav_z": packet.uav_position_world[2], "tip_x": state.tip_position_world[0], "tip_y": state.tip_position_world[1], "tip_z": state.tip_position_world[2], "tip_vx": state.tip_velocity_world[0], "tip_vy": state.tip_velocity_world[1], "tip_vz": state.tip_velocity_world[2], "ref_x": ref.position_world[0], "ref_y": ref.position_world[1], "ref_z": ref.position_world[2], "wind_x": wind[0], "wind_y": wind[1], "wind_z": wind[2], "uav_roll_deg": math.degrees(roll), "uav_pitch_deg": math.degrees(pitch), "uav_yaw_deg": math.degrees(yaw), "requested_thrust_N": requested.thrust_N, "applied_thrust_N": applied.actual.thrust_N, "requested_Mx_Nm": requested.torque_Nm[0], "requested_My_Nm": requested.torque_Nm[1], "requested_Mz_Nm": requested.torque_Nm[2], "applied_Mx_Nm": applied.actual.torque_Nm[0], "applied_My_Nm": applied.actual.torque_Nm[1], "applied_Mz_Nm": applied.actual.torque_Nm[2], "thrust_saturated": applied.thrust_saturated, "mx_saturated": bool(applied.torque_saturated[0]), "my_saturated": bool(applied.torque_saturated[1]), "mz_saturated": bool(applied.torque_saturated[2]), "ax_cmd": command[0], "ay_cmd": command[1], "az_cmd": command[2], "safe": finite, **{f"q{i+1}": q[i] for i in range(5)}, **{f"qd{i+1}": packet.joint_velocity[i] for i in range(5)}})
            sample_count += 1
        if t + 1e-9 >= next_render:
            render_t.append(t); render_q.append(data.qpos.copy()); next_render += 1.0 / 30.0
        if tick < nsteps: mujoco.mj_step(model, data)
    runtime = time.perf_counter() - started
    times = np.array([r["time"] for r in rows]); tip = np.array([[r["tip_x"], r["tip_y"], r["tip_z"]] for r in rows]); ref_arr = np.array([[r["ref_x"], r["ref_y"], r["ref_z"]] for r in rows]); vel = np.linalg.norm(np.array([[r["tip_vx"], r["tip_vy"], r["tip_vz"]] for r in rows]), axis=1); err = np.linalg.norm(tip - ref_arr, axis=1); dev = np.linalg.norm(tip - p0, axis=1); q = np.array([[r[f"q{i}"] for i in range(1, 6)] for r in rows]); jrms = np.sqrt(np.mean(q * q, axis=1)); effort = np.array([sum(float(r[f"a{a}_cmd"])**2 for a in "xyz") for r in rows]); integ = float(np.trapezoid(effort, times) if hasattr(np, "trapezoid") else np.trapz(effort, times))
    sat_rates = saturation_count / max(1, sample_count); common: dict[str, Any] = {"task": task, "controller": controller_id, "direction": direction, "duration_s": DURATION, "initial_angles_deg": INITIAL_ANGLES_DEG.tolist() if task in ("T1", "T2") else [0.0] * 5, "target_delta_m": TARGET_DELTA.tolist() if task in ("T1", "T2") else [0.0] * 3, "safety": safety, "runtime_s": runtime, "runtime_mean_ms": float(np.mean(outer_ms + inner_ms)), "runtime_p95_ms": float(np.percentile(outer_ms + inner_ms, 95)), "control_effort": integ, "wrench_saturation_rate": float(np.mean(sat_rates)), "thrust_saturation_rate": float(sat_rates[0]), "mx_saturation_rate": float(sat_rates[1]), "my_saturation_rate": float(sat_rates[2]), "mz_saturation_rate": float(sat_rates[3]), "torque_saturation_rate": float(np.mean(sat_rates[1:])), "any_wrench_saturation_rate": float(np.mean(np.any(np.column_stack([np.asarray([r["thrust_saturated"] for r in rows]), np.asarray([r["mx_saturated"] for r in rows]), np.asarray([r["my_saturated"] for r in rows]), np.asarray([r["mz_saturated"] for r in rows])]), axis=1))), "max_roll_deg": float(max(abs(float(r["uav_roll_deg"])) for r in rows)), "max_pitch_deg": float(max(abs(float(r["uav_pitch_deg"])) for r in rows)), "peak_joint_angle_deg": float(np.degrees(np.max(np.abs(q)))), "peak_joint_rms_deg": float(np.degrees(np.max(jrms))), "joint_rms_during_move_deg": float(np.degrees(np.sqrt(np.mean(q[(times >= 1.0) & (times <= 5.0)] ** 2)))), "final_5s_joint_rms_deg": float(np.degrees(np.sqrt(np.mean(q[times >= 35.0] ** 2))))}
    if task == "T1":
        valid = continuous_time(times, (err <= 0.15) & (vel <= 0.20), 1.0, 5.0); common.update({"tip_tracking_rmse_m": float(np.sqrt(np.mean(err**2))), "final_tip_error_m": float(err[-1]), "final_tip_speed_mps": float(vel[-1]), "peak_tip_tracking_error_m": float(np.max(err)), "peak_tip_speed_mps": float(np.max(vel)), "settling_after_move_s": None if valid is None else valid - 5.0})
    elif task == "T2":
        wind_peak_mask = times >= 2.5; peak_idx = int(np.argmax(np.where(wind_peak_mask, err, -np.inf))); valid = continuous_time(times, (err <= 0.15) & (vel <= 0.20), 1.0, 3.5); common.update({"wind_vector_mps": WIND_VECTORS_5["XY30"].tolist(), "wind_onset_time_s": 2.5, "peak_error_after_wind_m": float(err[peak_idx]), "peak_joint_rms_after_wind_deg": float(np.degrees(np.max(jrms[wind_peak_mask]))), "time_to_peak_after_wind_s": float(times[peak_idx] - 2.5), "recovery_after_wind_peak_s": None if valid is None else valid - times[peak_idx], "final_error_under_wind_m": float(err[-1]), "final_joint_rms_under_wind_deg": float(np.degrees(jrms[-1]))})
    else:
        peak_idx = int(np.argmax(np.where(times >= 8.0, dev, -np.inf))); valid = continuous_time(times, (dev <= 0.15) & (vel <= 0.20), 1.0, times[peak_idx]); common.update({"wind_vector_mps": WIND_VECTORS_10[direction].tolist(), "prewind_tip_rms_m": float(np.sqrt(np.mean(dev[times < 8.0] ** 2))), "postwind_tip_rms_m": float(np.sqrt(np.mean(dev[times >= 9.0] ** 2))), "peak_tip_deviation_m": float(dev[peak_idx]), "peak_time_s": float(times[peak_idx]), "steady_state_tip_error_m": float(np.mean(dev[times >= 9.0])), "joint_rms_rad": float(np.sqrt(np.mean(q*q))), "recovery_after_peak_s": None if valid is None else valid - times[peak_idx], "recovery_from_onset_s": None if valid is None else valid - 8.0})
    path = OUT / task / (direction if task == "T3" else "") / controller_id; path.mkdir(parents=True, exist_ok=True); _write_csv(path / "run.csv", rows); (path / "metrics.json").write_text(json.dumps(common, indent=2, allow_nan=False) + "\n", encoding="utf-8"); np.savez_compressed(path / "render_states.npz", time=np.asarray(render_t), qpos=np.asarray(render_q)); return {"job": job, "metrics": common, "runtime_s": runtime, "path": str(path)}


def _render_one(path: Path, label: str) -> str:
    import imageio.v2 as imageio
    model, data = mujoco.MjModel.from_xml_path(str(MODEL)), None; data = mujoco.MjData(model); states = np.load(path / "render_states.npz"); renderer = mujoco.Renderer(model, height=720, width=1280); frames: list[np.ndarray] = []
    for i, qpos in enumerate(states["qpos"]):
        data.qpos[:] = qpos; mujoco.mj_forward(model, data); renderer.update_scene(data, camera="oblique_camera"); frame = renderer.render().copy()
        # One saved state is one 30 FPS frame; do not thin frames and then
        # silently change the physical playback speed.
        frames.append(frame)
        if i in (0, len(states["qpos"]) // 2, len(states["qpos"]) - 1): imageio.imwrite(path / {0: "01_native_model_start.png", len(states["qpos"]) // 2: "02_native_model_midmove.png", len(states["qpos"]) - 1: "03_native_model_final.png"}[i], frame)
    if hasattr(renderer, "close"): renderer.close()
    out = path / f"{label}.mp4"
    try: imageio.mimsave(out, frames, fps=30, codec="libx264"); return str(out)
    except Exception:
        fallback = path / f"{label}.gif"; imageio.mimsave(fallback, frames, fps=30); return str(fallback)


def _side_by_side(left: Path, right: Path, out: Path) -> str:
    import imageio.v2 as imageio
    try:
        a, b = imageio.get_reader(left), imageio.get_reader(right); writer = imageio.get_writer(out, fps=30, codec="libx264")
        for fa, fb in zip(a, b): writer.append_data(np.concatenate([fa, fb], axis=1))
        writer.close(); a.close(); b.close(); return str(out)
    except Exception:
        if out.exists():
            out.unlink()
        return ""


def render_all(results: list[dict[str, Any]]) -> dict[str, str]:
    vids: dict[str, str] = {}
    for result in results:
        job = result["job"]; short = "LQR" if job["controller"] == "full_lqr_048" else "SATC"
        prefix = "T1_AGGRESSIVE" if job["task"] == "T1" else "T2_COMPOSITE" if job["task"] == "T2" else f"T3_{job['direction']}"
        key = f"{job['task']}_{job.get('direction', '')}_{short}".replace("__", "_")
        vids[key] = _render_one(Path(result["path"]), f"{prefix}_{short}")
    for task, prefix in (("T1", "T1_AGGRESSIVE"), ("T2", "T2_COMPOSITE")):
        lqr = Path(vids[f"{task}_LQR"]); satc = Path(vids[f"{task}_SATC"]); vids[f"{task}_side"] = _side_by_side(lqr, satc, OUT / task / f"{prefix}_LQR_vs_SATC.mp4")
    for direction in ("X", "Y", "XY30"):
        vids[f"T3_{direction}_side"] = _side_by_side(Path(vids[f"T3_{direction}_LQR"]), Path(vids[f"T3_{direction}_SATC"]), OUT / "T3" / direction / f"T3_{direction}_LQR_vs_SATC.mp4")
    return vids


def _make_slowmo(results: list[dict[str, Any]], vids: dict[str, str]) -> None:
    """Write 6-second, 0.5x clips around the largest joint-RMS window."""
    import imageio.v2 as imageio
    for task, name in (("T1", "T1_peak_sway_slowmo.mp4"), ("T2", "T2_peak_sway_slowmo.mp4")):
        candidates = [r for r in results if r["job"]["task"] == task]
        peak_times = []
        for result in candidates:
            rows = list(csv.DictReader((Path(result["path"]) / "run.csv").open(encoding="utf-8")))
            q = np.asarray([[float(row[f"q{i}"]) for i in range(1, 6)] for row in rows]); jrms = np.sqrt(np.mean(q * q, axis=1)); peak_times.append(float(rows[int(np.argmax(jrms))]["time"]))
        center = float(np.mean(peak_times))
        source = Path(vids[f"{task}_LQR"]); reader = imageio.get_reader(source); frames = [frame for frame in reader]; reader.close()
        start = max(0, int((center - 3.0) * 30)); stop = min(len(frames), int((center + 3.0) * 30))
        selected = frames[start:stop]
        if selected:
            out = OUT / task / name; writer = imageio.get_writer(out, fps=30, codec="libx264")
            for frame in selected:
                writer.append_data(frame); writer.append_data(frame)
            writer.close(); vids[f"{task}_slowmo"] = str(out)


def _plot_run(result: dict[str, Any]) -> None:
    path = Path(result["path"]); job = result["job"]; rows = list(csv.DictReader((path / "run.csv").open(encoding="utf-8"))); t = np.asarray([float(r["time"]) for r in rows]); q = np.asarray([[float(r[f"q{i}"]) for i in range(1, 6)] for r in rows]); tip = np.asarray([[float(r[f"tip_{a}"]) for a in "xyz"] for r in rows]); ref = np.asarray([[float(r[f"ref_{a}"]) for a in "xyz"] for r in rows]); err = np.linalg.norm(tip - ref, axis=1)
    def save(name: str, y: Any, title: str, ylabel: str) -> None:
        fig, ax = plt.subplots(figsize=(7, 4)); ax.plot(t, y); ax.set_title(title); ax.set_xlabel("time (s)"); ax.set_ylabel(ylabel); ax.grid(True); fig.tight_layout(); fig.savefig(path / name, dpi=140); plt.close(fig)
    save("T1_joint_angles.png" if job["task"] == "T1" else "T2_joint_rms.png" if job["task"] == "T2" else "joint_angles.png", q if job["task"] != "T2" else np.sqrt(np.mean(q*q, axis=1)), "Joint motion", "rad"); save("T1_tip_error.png" if job["task"] == "T1" else "T2_tip_error.png" if job["task"] == "T2" else "tip_deviation.png", err, "Tip error", "m"); save("T1_reference_xyz.png" if job["task"] == "T1" else "tip_xyz.png", np.c_[tip, ref], "Tip and reference", "m"); save("T1_roll_pitch.png" if job["task"] == "T1" else "T2_roll_pitch.png" if job["task"] == "T2" else "roll_pitch.png", np.asarray([[float(r["uav_roll_deg"]), float(r["uav_pitch_deg"])] for r in rows]), "UAV roll/pitch", "deg"); save("T1_uav_xyz.png" if job["task"] == "T1" else "T2_uav_xyz.png" if job["task"] == "T2" else "uav_xyz.png", np.asarray([[float(r[f"uav_{a}"]) for a in "xyz"] for r in rows]), "UAV XYZ", "m"); save("T1_command_acceleration.png" if job["task"] == "T1" else "T2_command_acceleration.png" if job["task"] == "T2" else "command_acceleration.png", np.asarray([[float(r[f"a{a}_cmd"]) for a in "xyz"] for r in rows]), "Command acceleration", "m/s2"); save("T1_actuator_saturation.png" if job["task"] == "T1" else "T2_saturation.png" if job["task"] == "T2" else "saturation.png", np.asarray([[float(str(r[k]).lower() == "true") for k in ("thrust_saturated", "mx_saturated", "my_saturated", "mz_saturated")] for r in rows]), "Actuator saturation", "bool")
    if job["task"] == "T2": save("T2_wind_components.png", np.asarray([[float(r["wind_x"]), float(r["wind_y"]), float(r["wind_z"])] for r in rows]), "Wind components", "m/s")


def _make_summaries(results: list[dict[str, Any]], vids: dict[str, str]) -> None:
    ART.mkdir(parents=True, exist_ok=True); by = {(r["job"]["task"], r["job"].get("direction", ""), r["job"]["controller"]): r for r in results}
    lines = ["# EXTREME COMPOSITE FUNCTIONAL STRESS DEMOS", "", "| Task | Controller | Safety | Key metric |", "|---|---|---|---:|"]
    for task, direction, key in [("T1", "", "peak_joint_rms_deg"), ("T2", "", "peak_error_after_wind_m"), ("T3", "X", "postwind_tip_rms_m"), ("T3", "Y", "postwind_tip_rms_m"), ("T3", "XY30", "postwind_tip_rms_m")]:
        for controller in CONTROLLERS:
            m = by[(task, direction, controller)]["metrics"]; lines.append(f"| {task}{('_' + direction) if direction else ''} | {controller} | {'PASS' if m['safety']['finite'] and not m['safety']['violations'] else 'STRESS_CASE_COMPLETE_WITH_FAILURE'} | {m.get(key)} |")
    (OUT / "EXTREME_COMPOSITE_METRICS.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (OUT / "STRESS_ESCALATION.md").write_text("# Stress escalation\n\nP3-R1C used 20 s move, zero initial sway, and 5 m/s winds. P3-R1D uses a 4 s move with [20,-16,12,-8,4] deg initial sway, a 5 m/s XY30 wind during the move, and 10 m/s directional winds. Historical P3-R1C metrics are not rewritten.\n", encoding="utf-8")
    (DOC / "EXTREME_WIND_CONTEXT.md").write_text("# Extreme wind context\n\nThe frozen aerodynamic model contains speed-squared drag terms. Therefore 10 m/s is not merely a two-times disturbance force relative to 5 m/s; the corresponding quadratic terms are approximately four times larger. This package is an extreme functional stress demo, not a new holdout or performance claim.\n", encoding="utf-8")
    (DOC / "DIRECTIONAL_WIND_MODEL_LIMITATION.md").write_text("# Directional wind model limitation\n\nThe frozen five-link articulation uses five y-axis hinges, so internal chain sway is primarily in local X-Z planes. +Y and oblique wind still applies real distributed 3-D aerodynamic loads, but this is not a fully spatial two-axis multi-link sway model.\n", encoding="utf-8")
    fig, ax = plt.subplots(2, 2, figsize=(10, 7)); labels = ["X", "Y", "XY30"]
    for a, (task, direction, metric, title) in zip(ax.flat, [("T1", "", "peak_joint_rms_deg", "T1 peak joint RMS (deg)"), ("T1", "", "final_tip_error_m", "T1 final error (m)"), ("T2", "", "peak_error_after_wind_m", "T2 composite peak error (m)"), ("T3", None, "postwind_tip_rms_m", "T3 postwind RMS (m)")]):
        if task == "T3":
            for c, color in zip(CONTROLLERS, ("#2563eb", "#dc2626")): a.plot(labels, [by[("T3", d, c)]["metrics"][metric] for d in labels], "o-", label=c, color=color)
        else: a.bar(["LQR", "SATC"], [by[(task, direction, c)]["metrics"][metric] for c in CONTROLLERS], color=["#2563eb", "#dc2626"])
        a.set_title(title); a.grid(axis="y"); a.legend(fontsize=7) if task == "T3" else None
    fig.tight_layout(); fig.savefig(OUT / "EXTREME_COMPOSITE_SUMMARY.png", dpi=160); plt.close(fig)
    (ART / "DEMO_HARNESS_AUDIT.json").write_text(json.dumps({"controller_identity": "frozen acceleration-level controller", "harness": "LegacyTaskLevelAdapter", "physical_actuation": "CanonicalWrenchActuator", "native_controller_claim": False}, indent=2) + "\n", encoding="utf-8")


def run_all() -> dict[str, Any]:
    digest = hashlib.sha256(MODEL.read_bytes()).hexdigest()
    if digest != MODEL_SHA256: raise RuntimeError(f"BLOCK_MODEL_SHA_MISMATCH:{digest}")
    OUT.mkdir(parents=True, exist_ok=True); ART.mkdir(parents=True, exist_ok=True); registry = jobs(); (ART / "job_registry.json").write_text(json.dumps(registry, indent=2) + "\n", encoding="utf-8")
    cores = os.cpu_count() or 1; workers = min(10, max(1, cores - 2)); started = time.perf_counter(); results: list[dict[str, Any]] = []
    with ProcessPoolExecutor(max_workers=workers) as pool:
        future_map = {pool.submit(_run_job, job): job for job in registry}
        for future in as_completed(future_map): results.append(future.result())
    wall = time.perf_counter() - started; serial = sum(r["runtime_s"] for r in results); (ART / "parallel_execution_audit.json").write_text(json.dumps({"cpu_count": cores, "max_workers": workers, "job_count": len(registry), "total_wall_time_s": wall, "serial_time_sum_s": serial, "parallel_speedup_estimate": serial / wall if wall else None}, indent=2) + "\n", encoding="utf-8")
    for result in results: _plot_run(result)
    vids = render_all(results); _make_slowmo(results, vids); _make_summaries(results, vids)
    audit: dict[str, Any] = {"ATTITUDE_LOGGING_FIXED": True, "controllers": {}}
    for result in results:
        task = result["job"]["task"]
        if task in ("T1", "T2"):
            rows = list(csv.DictReader((Path(result["path"]) / "run.csv").open(encoding="utf-8"))); q = np.asarray([[float(r[f"q{i}"]) for i in range(1, 6)] for r in rows]); audit["controllers"][result["job"]["controller"]] = {f"joint_{i+1}": {"initial_deg": float(np.degrees(q[0, i])), "min_deg": float(np.degrees(np.min(q[:, i]))), "max_deg": float(np.degrees(np.max(q[:, i]))), "peak_to_peak_deg": float(np.degrees(np.ptp(q[:, i]))), "final_deg": float(np.degrees(q[-1, i]))} for i in range(5)}
    (ART / "attitude_logging_audit.json").write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8")
    return {"cpu_count": cores, "workers": workers, "job_count": len(registry), "wall_time_s": wall, "speedup": serial / wall if wall else None, "videos": vids, "results": results}


def finalize_existing() -> None:
    """Rebuild release summaries/audits from completed extreme job artifacts."""
    results: list[dict[str, Any]] = []
    for task in ("T1", "T2"):
        for controller in CONTROLLERS:
            path = OUT / task / controller
            results.append({"job": {"task": task, "controller": controller}, "path": str(path), "metrics": json.loads((path / "metrics.json").read_text(encoding="utf-8"))})
    for direction in ("X", "Y", "XY30"):
        for controller in CONTROLLERS:
            path = OUT / "T3" / direction / controller
            results.append({"job": {"task": "T3", "direction": direction, "controller": controller}, "path": str(path), "metrics": json.loads((path / "metrics.json").read_text(encoding="utf-8"))})
    vids: dict[str, str] = {}
    for task, prefix in (("T1", "T1_AGGRESSIVE"), ("T2", "T2_COMPOSITE")):
        for short, controller in (("LQR", "full_lqr_048"), ("SATC", "satc_b_027")):
            vids[f"{task}_{short}"] = str(next((OUT / task / controller).glob(f"{prefix}_{short}.mp4"), OUT / task / controller / f"{prefix}_{short}.mp4"))
    for direction in ("X", "Y", "XY30"):
        for short, controller in (("LQR", "full_lqr_048"), ("SATC", "satc_b_027")):
            vids[f"T3_{direction}_{short}"] = str(next((OUT / "T3" / direction / controller).glob(f"T3_{direction}_{short}.mp4"), OUT / "T3" / direction / controller / f"T3_{direction}_{short}.mp4"))
    _make_summaries(results, vids)
    audit: dict[str, Any] = {"ATTITUDE_LOGGING_FIXED": True, "controllers": {}}
    for result in results:
        if result["job"]["task"] in ("T1", "T2"):
            rows = list(csv.DictReader((Path(result["path"]) / "run.csv").open(encoding="utf-8"))); q = np.asarray([[float(row[f"q{i}"]) for i in range(1, 6)] for row in rows]); audit["controllers"][result["job"]["controller"]] = {f"joint_{i+1}": {"initial_deg": float(np.degrees(q[0, i])), "min_deg": float(np.degrees(np.min(q[:, i]))), "max_deg": float(np.degrees(np.max(q[:, i]))), "peak_to_peak_deg": float(np.degrees(np.ptp(q[:, i]))), "final_deg": float(np.degrees(q[-1, i]))} for i in range(5)}
    (ART / "attitude_logging_audit.json").write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8")
