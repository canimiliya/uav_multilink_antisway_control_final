"""Three-task meeting demonstration on the frozen Native-Stack plant.

The runner is deliberately a thin diagnostic harness.  It reuses the frozen
model, the existing Native-Stack controller adapters, the unchanged
``GeometricInnerLoop``, and the registered distributed-wind implementation.
It never loads Holdout data and never changes controller parameters.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any

import matplotlib
matplotlib.use("Agg")
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import mujoco
import numpy as np

from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind_world
from uav_sway.models.model_config import load_model_config
from uav_sway.native_stack.actuation import CanonicalWrenchActuator
from uav_sway.native_stack.api import ReferenceSample, WrenchCommand
from uav_sway.native_stack.r1r1_controllers import LegacyTaskLevelAdapter, MASS_KG
from uav_sway.native_stack.sensors import NativeSensorReader
from uav_sway.task_space.state import CutterTaskSpaceReader


ROOT = Path(__file__).resolve().parents[3]
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
MODEL_SHA256 = "19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d"
OUT = ROOT / "outputs/meeting_demo"
DT = 0.001
INNER_RATE = 200
OUTER_RATE = 20
INNER_STRIDE = 5
OUTER_STRIDE = 50


def _quintic(u: float) -> tuple[float, float, float, float]:
    u = float(np.clip(u, 0.0, 1.0))
    p = 10*u**3 - 15*u**4 + 6*u**5
    v = 30*u**2 - 60*u**3 + 30*u**4
    a = 60*u - 180*u**2 + 120*u**3
    j = 60 - 360*u + 360*u**2
    if u <= 0.0 or u >= 1.0:
        v = a = j = 0.0
    return p, v, a, j


def _reference(p0: np.ndarray, target: np.ndarray, time_s: float, start: float, end: float) -> ReferenceSample:
    if time_s < start:
        return ReferenceSample(p0, np.zeros(3), np.zeros(3), np.zeros(3), time_s)
    if time_s >= end:
        return ReferenceSample(target, np.zeros(3), np.zeros(3), np.zeros(3), time_s)
    p, v, a, j = _quintic((time_s-start)/(end-start))
    d = target-p0; duration = end-start
    return ReferenceSample(p0+p*d, (v/duration)*d, (a/duration**2)*d, (j/duration**3)*d, time_s)


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)


def _continuous_time(times: np.ndarray, mask: np.ndarray, hold_s: float, start_s: float = 0.0) -> float | None:
    if len(times) < 2: return None
    dt = float(np.median(np.diff(times))); needed = max(1, int(math.ceil(hold_s/dt)))
    count = 0
    for i, ok in enumerate(mask):
        if times[i] < start_s: count = 0; continue
        count = count + 1 if bool(ok) else 0
        if count >= needed: return float(times[i-needed+1])
    return None


def _run_case(task: str, controller_id: str, duration: float = 12.0) -> tuple[list[dict[str, Any]], dict[str, Any], dict[str, Any]]:
    digest = hashlib.sha256(MODEL.read_bytes()).hexdigest()
    if digest != MODEL_SHA256:
        raise RuntimeError(f"BLOCK_MODEL_SHA_MISMATCH: {digest}")
    model = mujoco.MjModel.from_xml_path(str(MODEL)); data = mujoco.MjData(model)
    data.qpos[:] = 0.0; data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    if task == "task2_initial_sway":
        addresses = [int(model.jnt_qposadr[int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}"))]) for i in range(1, 6)]
        data.qpos[addresses] = np.deg2rad([5.0, -4.0, 3.0, -2.0, 1.0])
    data.qvel[:] = 0.0; data.ctrl[:] = 0.0; data.eq_active[:] = 0; mujoco.mj_forward(model, data)
    model_cfg = load_model_config(ROOT / "configs/model_5link.yaml")
    aero_cfg = load_aerodynamic_config(ROOT / "configs/aerodynamics.yaml")
    sensor = NativeSensorReader(model); task_reader = CutterTaskSpaceReader(model); actuator = CanonicalWrenchActuator(model)
    quad = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")); tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    link_ids = [int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"link_{i}")) for i in range(1,6)]
    p0 = np.asarray(data.site_xpos[tip_id], dtype=float).copy(); uav0 = np.asarray(data.xpos[quad], dtype=float).copy()
    target = p0.copy() if task != "task1_move" else p0 + np.array([0.30, 0.20, 0.15])
    controller = LegacyTaskLevelAdapter(controller_id, 0.0, 0.0, historical_id=controller_id); controller.reset()
    rows: list[dict[str, Any]] = []; traces = {"uav": [], "links": [], "tip": []}; previous = WrenchCommand(0.0, np.zeros(3))
    safety = {"finite": True, "nan_count": 0, "max_joint_angle_rad": 0.0, "min_tip_height_m": float("inf"), "min_uav_height_m": float("inf"), "max_roll_pitch_deg": 0.0, "violations": []}
    outer_times: list[float] = []; inner_times: list[float] = []
    qpos_addresses = [int(model.jnt_qposadr[int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}"))]) for i in range(1,6)]
    qvel_addresses = [int(model.jnt_dofadr[int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}"))]) for i in range(1,6)]
    last_ref = _reference(p0, target, 0.0, 1.0, 5.0); command = np.zeros(3)
    for tick in range(int(round(duration/DT)) + 1):
        t = tick*DT
        wind = np.array([3.0, 0.0, 0.0]) if task == "task3_wind" and t >= 4.0 else np.zeros(3)
        clear_and_apply_wind_world(model, data, model_cfg, aero_cfg, wind)
        if task == "task2_initial_sway": ref = _reference(p0, p0, t, 0.0, 1.0)
        elif task == "task3_wind": ref = _reference(p0, p0, t, 0.0, 1.0)
        else: ref = _reference(p0, target, t, 1.0, 5.0)
        packet = sensor.read(model, data, ref, previous, tick, DT); controller.observe(packet)
        if tick % OUTER_STRIDE == 0:
            started = time.perf_counter_ns(); controller.update_high_level(); outer_times.append((time.perf_counter_ns()-started)*1e-6)
            diag = controller.diagnostics()
            # LegacyTaskLevelAdapter exposes the unchanged acceleration through
            # its protected state; its diagnostics intentionally only report
            # provenance metadata.
            command = np.asarray(getattr(controller, "_acceleration", command), dtype=float).copy()
        if tick % INNER_STRIDE == 0:
            started = time.perf_counter_ns(); controller.update_inner(); inner_times.append((time.perf_counter_ns()-started)*1e-6)
            applied = actuator.apply(data, controller.physical_command(), tick, DT); previous = applied.actual
            task_state = task_reader.read(model, data); rotation = packet.rotation_world_from_body
            roll = math.degrees(math.atan2(rotation[2,1], rotation[2,2])); pitch = math.degrees(math.asin(np.clip(-rotation[2,0], -1.0, 1.0)))
            finite_values = np.r_[packet.uav_position_world, packet.uav_velocity_world, packet.joint_position, packet.joint_velocity, task_state.tip_position_world, task_state.tip_velocity_world, command, applied.actual.as_array()]
            finite = bool(np.isfinite(finite_values).all()); safety["finite"] &= finite; safety["nan_count"] += int(np.count_nonzero(~np.isfinite(finite_values)))
            safety["max_joint_angle_rad"] = max(safety["max_joint_angle_rad"], float(np.max(np.abs(packet.joint_position))))
            safety["min_tip_height_m"] = min(safety["min_tip_height_m"], float(task_state.tip_position_world[2])); safety["min_uav_height_m"] = min(safety["min_uav_height_m"], float(packet.uav_position_world[2]))
            safety["max_roll_pitch_deg"] = max(safety["max_roll_pitch_deg"], abs(roll), abs(pitch))
            if not finite: safety["violations"].append({"time_s": t, "reason": "non_finite"})
            rows.append({"time": t, "uav_x": packet.uav_position_world[0], "uav_y": packet.uav_position_world[1], "uav_z": packet.uav_position_world[2], "uav_vx": packet.uav_velocity_world[0], "uav_vy": packet.uav_velocity_world[1], "uav_vz": packet.uav_velocity_world[2], "tip_x": task_state.tip_position_world[0], "tip_y": task_state.tip_position_world[1], "tip_z": task_state.tip_position_world[2], "tip_vx": task_state.tip_velocity_world[0], "tip_vy": task_state.tip_velocity_world[1], "tip_vz": task_state.tip_velocity_world[2], "ref_x": ref.position_world[0], "ref_y": ref.position_world[1], "ref_z": ref.position_world[2], **{f"q{i+1}": packet.joint_position[i] for i in range(5)}, **{f"qd{i+1}": packet.joint_velocity[i] for i in range(5)}, "ax_cmd": command[0], "ay_cmd": command[1], "az_cmd": command[2], "controller_runtime_ms": (outer_times[-1] if outer_times else 0.0) + (inner_times[-1] if inner_times else 0.0), "wind_x": wind[0], "safe": finite})
            traces["uav"].append(np.asarray(data.xpos[quad], dtype=float).copy()); traces["links"].append(np.asarray([data.xpos[i] for i in link_ids], dtype=float).copy()); traces["tip"].append(task_state.tip_position_world.copy())
        if tick < int(round(duration/DT)): mujoco.mj_step(model, data)
    times = np.asarray([r["time"] for r in rows]); tip = np.asarray([[r["tip_x"],r["tip_y"],r["tip_z"]] for r in rows]); ref_arr = np.asarray([[r["ref_x"],r["ref_y"],r["ref_z"]] for r in rows]); err = np.linalg.norm(tip-ref_arr, axis=1); speed = np.linalg.norm(np.asarray([[r["tip_vx"],r["tip_vy"],r["tip_vz"]] for r in rows]), axis=1); q = np.asarray([[r[f"q{i}"] for i in range(1,6)] for r in rows]); uav = np.asarray([[r["uav_x"],r["uav_y"],r["uav_z"]] for r in rows]); effort_signal = np.sum(np.asarray([[r["ax_cmd"],r["ay_cmd"],r["az_cmd"]] for r in rows])**2, axis=1)
    metrics: dict[str, Any] = {"task": task.upper(), "controller": controller_id, "duration_s": duration, "sample_count": len(rows), "safety": safety, "runtime_mean_ms": float(np.mean(outer_times+inner_times)), "runtime_p95_ms": float(np.percentile(outer_times+inner_times,95)), "control_effort": float(np.trapezoid(effort_signal, times) if hasattr(np,"trapezoid") else np.trapz(effort_signal,times))}
    if task == "task1_move":
        metrics.update({"tip_rmse_3d_m": float(np.sqrt(np.mean(err**2))), "final_tip_error_m": float(err[-1]), "final_tip_speed_mps": float(speed[-1]), "peak_tip_error_m": float(np.max(err)), "uav_rmse_3d_m": float(np.sqrt(np.mean(np.sum((uav-uav0)**2,axis=1)))), "joint_rms_rad": float(np.sqrt(np.mean(q**2))), "max_joint_angle_rad": float(np.max(np.abs(q))), "settling_time_s": _continuous_time(times, (err<=.15)&(speed<=.20), 1.0, 1.0)})
    elif task == "task2_initial_sway":
        jrms = np.sqrt(np.mean(q**2,axis=1)); initial = float(jrms[0]); metrics.update({"initial_joint_rms_rad": initial, "overall_joint_rms_rad": float(np.sqrt(np.mean(q**2))), "final_joint_rms_rad": float(jrms[-1]), "peak_joint_rms_rad": float(np.max(jrms)), "tip_velocity_rms_mps": float(np.sqrt(np.mean(speed**2))), "tip_displacement_rms_m": float(np.sqrt(np.mean(err**2))), "uav_correction_displacement_m": float(np.max(np.linalg.norm(uav-uav0,axis=1))), "decay50_time_s": _continuous_time(times,jrms<=.5*initial,.5,0.0) if initial else 0.0, "decay90_time_s": _continuous_time(times,jrms<=.1*initial,.5,0.0) if initial else 0.0})
    else:
        pre = times < 4.0; post = times >= 4.0; deviation = np.linalg.norm(tip-p0,axis=1); metrics.update({"prewind_tip_rms_m": float(np.sqrt(np.mean(deviation[pre]**2))), "postwind_tip_rms_m": float(np.sqrt(np.mean(deviation[post]**2))), "peak_tip_deviation_m": float(np.max(deviation[post])), "uav_rmse_3d_m": float(np.sqrt(np.mean(np.sum((uav-uav0)**2,axis=1)))), "steady_state_tip_error_m": float(np.mean(deviation[times>=10.0])), "joint_rms_rad": float(np.sqrt(np.mean(q**2))), "recovery_time_s": (lambda x: None if x is None else x-4.0)(_continuous_time(times,(deviation<=.15)&(speed<=.20),1.0,4.0))})
    return rows, metrics, {k: np.asarray(v) for k,v in traces.items()}


def _plot_task(task: str, controller_rows: dict[str, list[dict[str, Any]]], out: Path) -> int:
    out.mkdir(parents=True, exist_ok=True); count = 0
    colors = {"full_lqr_048": "#2563eb", "satc_b_027": "#dc2626"}
    def save(fig, name):
        nonlocal count; fig.tight_layout(); fig.savefig(out/name, dpi=160); plt.close(fig); count += 1
    def arr(rows, names): return np.asarray([[float(r[n]) for n in names] for r in rows], dtype=float)
    for idx, (title, values, ylabel) in enumerate([("Tip XYZ", ["tip_x","tip_y","tip_z"], "position (m)"), ("Tip error norm", None, "error (m)"), ("Tip speed", ["tip_vx","tip_vy","tip_vz"], "speed (m/s)"), ("Joint angles", [f"q{i}" for i in range(1,6)], "angle (rad)"), ("UAV XYZ", ["uav_x","uav_y","uav_z"], "position (m)"), ("Command acceleration", ["ax_cmd","ay_cmd","az_cmd"], "acceleration (m/s2)"), ("Runtime", ["controller_runtime_ms"], "runtime (ms)"), ("Safety", None, "safe")]):
        fig, ax = plt.subplots(figsize=(7,4.2))
        for cid, rows in controller_rows.items():
            t = np.asarray([r["time"] for r in rows]);
            if values is None:
                if title == "Tip error norm": y = np.linalg.norm(arr(rows,["tip_x","tip_y","tip_z"])-arr(rows,["ref_x","ref_y","ref_z"]),axis=1)
                else: y = np.asarray([1.0 if str(r["safe"]).lower() == "true" else 0.0 for r in rows])
                ax.plot(t,y,label=cid,color=colors[cid])
            else:
                data = arr(rows,values)
                if title == "Tip speed": data=np.linalg.norm(data,axis=1)[:, None]
                if data.ndim == 1: data = data[:, None]
                for j in range(data.shape[1]): ax.plot(t,data[:,j],label=(cid if data.shape[1]==1 else f"{cid}:{j+1}"),color=colors[cid],alpha=1.0-0.15*j)
        ax.set_title(title); ax.set_xlabel("time (s)"); ax.set_ylabel(ylabel); ax.grid(True); ax.legend(fontsize=7); save(fig, f"{idx+1:02d}_{title.lower().replace(' ','_')}.png")
    if task in {"task1_move", "task3_wind"}:
        fig = plt.figure(figsize=(6,5)); ax=fig.add_subplot(111,projection="3d")
        for cid, rows in controller_rows.items():
            data=arr(rows,["tip_x","tip_y","tip_z"]); ax.plot(data[:,0],data[:,1],data[:,2],label=cid,color=colors[cid])
        ax.set_title("3D trajectory"); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)"); ax.legend(); save(fig,"09_trajectory_3d.png")
    return count


def _animate(task: str, traces: dict[str, dict[str, np.ndarray]], out: Path) -> str:
    # A compact GIF fallback is deterministic, portable, and explicitly keeps
    # the single simulation trajectory as its source.
    frames = min(120, len(next(iter(traces.values()))["uav"])); indices = np.linspace(0, len(next(iter(traces.values()))["uav"])-1, frames).astype(int)
    fig = plt.figure(figsize=(9.6,7.2)); ax=fig.add_subplot(111,projection="3d"); ax.set_xlim(-2,2); ax.set_ylim(-2,2); ax.set_zlim(0,4); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.set_zlabel("z (m)"); ax.set_title(task)
    lines = {cid: ax.plot([],[],[],label=cid)[0] for cid in traces}; dots={cid:ax.plot([],[],[],"o")[0] for cid in traces}; ax.legend(fontsize=7)
    def update(frame):
        for cid, trace in traces.items():
            i=indices[frame]; chain=np.vstack([trace["uav"][i][None,:],trace["links"][i],trace["tip"][i][None,:]])
            lines[cid].set_data(chain[:,0],chain[:,1]); lines[cid].set_3d_properties(chain[:,2]); dots[cid].set_data([trace["tip"][i,0]],[trace["tip"][i,1]]); dots[cid].set_3d_properties([trace["tip"][i,2]])
        return (*lines.values(),*dots.values())
    path = out / f"{task}_satc.gif"; ani=animation.FuncAnimation(fig,update,frames=frames,interval=1000/30,blit=False); ani.save(path,writer=animation.PillowWriter(fps=30)); plt.close(fig); return str(path)


def run_all(plots_only: bool = False) -> dict[str, Any]:
    OUT.mkdir(parents=True, exist_ok=True)
    task_names = ("task1_move", "task2_initial_sway", "task3_wind")
    all_metrics: dict[str, dict[str, Any]] = {}; traces: dict[str, dict[str, dict[str,np.ndarray]]] = {}
    if not plots_only:
        for task in task_names:
            all_metrics[task] = {}; traces[task] = {}
            task_dir = OUT/task
            for controller_id in ("full_lqr_048", "satc_b_027"):
                rows, metrics, trace = _run_case(task, controller_id)
                _write_csv(task_dir/controller_id/"run.csv", rows); (task_dir/controller_id/"metrics.json").parent.mkdir(parents=True,exist_ok=True); (task_dir/controller_id/"metrics.json").write_text(json.dumps(metrics,indent=2,allow_nan=False)+"\n",encoding="utf-8")
                all_metrics[task][controller_id] = metrics; traces[task][controller_id] = trace
            (task_dir/"safety.json").write_text(json.dumps({cid:all_metrics[task][cid]["safety"] for cid in all_metrics[task]},indent=2,allow_nan=False)+"\n",encoding="utf-8")
    else:
        for task in task_names:
            all_metrics[task] = {}
            for cid in ("full_lqr_048","satc_b_027"):
                all_metrics[task][cid] = json.loads((OUT/task/cid/"metrics.json").read_text(encoding="utf-8"))
    plot_counts = {task: _plot_task(task, {cid: [dict(r) for r in csv.DictReader((OUT/task/cid/"run.csv").open(encoding="utf-8"))] for cid in ("full_lqr_048","satc_b_027")}, OUT/task) for task in task_names}
    if not plots_only:
        for task in task_names: _animate(task, traces[task], OUT/task)
    summary = OUT/"meeting_summary.png"; fig, axes = plt.subplots(1,3,figsize=(12,4.2)); panels=(("task1_move","tip_rmse_3d_m","T1: 3D tip RMSE"),("task2_initial_sway","final_joint_rms_rad","T2: final joint RMS"),("task3_wind","postwind_tip_rms_m","T3: post-wind tip RMS"))
    for ax,(task,key,title) in zip(axes,panels):
        vals=[all_metrics[task][cid].get(key,float("nan")) for cid in ("full_lqr_048","satc_b_027")]; ax.bar(["LQR","SATC"],vals,color=["#2563eb","#dc2626"]); ax.set_title(title); ax.set_ylabel("value"); ax.grid(axis="y")
    fig.tight_layout(); fig.savefig(summary,dpi=160); plt.close(fig)
    lines=["# Meeting metrics", "", "| Task | Controller | RMSE / recovery metric | Control effort | Safety |", "|---|---|---:|---:|---|"]
    for task in task_names:
        for cid in ("full_lqr_048","satc_b_027"):
            m=all_metrics[task][cid]; key={"task1_move":"tip_rmse_3d_m","task2_initial_sway":"final_joint_rms_rad","task3_wind":"postwind_tip_rms_m"}[task]; lines.append(f"| {task} | {cid} | {m.get(key)} | {m['control_effort']:.6g} | {'PASS' if m['safety']['finite'] and not m['safety']['violations'] else 'FAIL'} |")
    (OUT/"MEETING_METRICS.md").write_text("\n".join(lines)+"\n",encoding="utf-8")
    return {"metrics":all_metrics,"plot_counts":plot_counts,"summary_png":str(summary)}


def run_single(task: str, controller: str) -> dict[str, Any]:
    """Run one task/controller pair for quick inspection."""
    controller_id = {"lqr": "full_lqr_048", "satc": "satc_b_027", "full_lqr_048": "full_lqr_048", "satc_b_027": "satc_b_027"}.get(controller, controller)
    if controller_id not in {"full_lqr_048", "satc_b_027"}:
        raise ValueError("controller must be lqr or satc")
    rows, metrics, _ = _run_case(task, controller_id)
    target = OUT / task / controller_id
    _write_csv(target / "run.csv", rows)
    target.mkdir(parents=True, exist_ok=True)
    (target / "metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    return metrics


if __name__ == "__main__":
    run_all()
