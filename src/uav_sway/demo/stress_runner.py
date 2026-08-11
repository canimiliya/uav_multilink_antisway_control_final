"""P3-R1C fixed stress scenarios and native MuJoCo render package.

This module is intentionally a release harness: the frozen plant, adapters and
controller identifiers are loaded exactly as in the earlier meeting demo.
Only scenario duration, references and distributed wind are changed.
"""
from __future__ import annotations

import csv, hashlib, json, math, os, time
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
OUT = ROOT / "outputs/meeting_demo_stress_v2"
ART = ROOT / "artifacts/meeting_demo_stress_v2"
DT, INNER_STRIDE, OUTER_STRIDE = 0.001, 5, 50
CONTROLLERS = ("full_lqr_048", "satc_b_027")
WIND_VECTORS = {
    "X": np.array([5.0, 0.0, 0.0]),
    "Y": np.array([0.0, 5.0, 0.0]),
    "XY30": np.array([5.0 * math.cos(math.radians(30)), 5.0 * math.sin(math.radians(30)), 0.0]),
}


def quintic(u: float) -> tuple[float, float, float, float]:
    u = float(np.clip(u, 0.0, 1.0)); p = 10*u**3 - 15*u**4 + 6*u**5
    v = 30*u**2 - 60*u**3 + 30*u**4; a = 60*u - 180*u**2 + 120*u**3
    j = 60 - 360*u + 360*u**2
    if u <= 0 or u >= 1: v = a = j = 0.0
    return p, v, a, j


def reference(p0: np.ndarray, target: np.ndarray, t: float) -> ReferenceSample:
    if t < 3.0: return ReferenceSample(p0, np.zeros(3), np.zeros(3), np.zeros(3), t)
    if t >= 23.0: return ReferenceSample(target, np.zeros(3), np.zeros(3), np.zeros(3), t)
    p, v, a, j = quintic((t - 3.0) / 20.0); d = target - p0
    return ReferenceSample(p0 + p*d, v*d/20.0, a*d/20.0**2, j*d/20.0**3, t)


def wind_profile(name: str, t: float) -> np.ndarray:
    if name not in WIND_VECTORS or t < 8.0: return np.zeros(3)
    f = 1.0 if t >= 10.0 else 0.5 * (1.0 - math.cos(math.pi * (t - 8.0) / 2.0))
    return WIND_VECTORS[name] * f


def continuous_time(times: np.ndarray, mask: np.ndarray, hold_s: float, start_s: float) -> float | None:
    if len(times) < 2: return None
    dt = float(np.median(np.diff(times))); n = max(1, int(math.ceil(hold_s / dt))); count = 0
    for i, ok in enumerate(mask):
        if times[i] < start_s: count = 0; continue
        count = count + 1 if bool(ok) else 0
        if count >= n: return float(times[i-n+1])
    return None


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)


def _run_job(job: dict[str, str]) -> dict[str, Any]:
    for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"): os.environ[k] = "1"
    digest = hashlib.sha256(MODEL.read_bytes()).hexdigest()
    if digest != MODEL_SHA256: raise RuntimeError(f"BLOCK_MODEL_SHA_MISMATCH:{digest}")
    task, controller_id, direction = job["task"], job["controller"], job.get("direction", "")
    model = mujoco.MjModel.from_xml_path(str(MODEL)); data = mujoco.MjData(model)
    data.qpos[:] = 0; data.qpos[:7] = [0, 0, 3.2, 1, 0, 0, 0]
    jids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}") for i in range(1, 6)]
    qaddr = [int(model.jnt_qposadr[j]) for j in jids]; vaddr = [int(model.jnt_dofadr[j]) for j in jids]
    if task == "T2": data.qpos[qaddr] = np.deg2rad([20, -16, 12, -8, 4])
    data.qvel[:] = 0; data.ctrl[:] = 0; data.eq_active[:] = 0; mujoco.mj_forward(model, data)
    cfg = load_model_config(ROOT / "configs/model_5link.yaml"); aero = load_aerodynamic_config(ROOT / "configs/aerodynamics.yaml")
    sensor, reader, actuator = NativeSensorReader(model), CutterTaskSpaceReader(model), CanonicalWrenchActuator(model)
    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip")); quad_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    link_ids = [int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"link_{i}")) for i in range(1, 6)]
    p0 = np.array(data.site_xpos[tip_id]); uav0 = np.array(data.xpos[quad_id]); target = p0 + np.array([2.0, 1.7, 4.5]) if task == "T1" else p0.copy()
    ctl = LegacyTaskLevelAdapter(controller_id, 0.0, 0.0, historical_id=controller_id); ctl.reset(); previous = WrenchCommand(0.0, np.zeros(3)); command = np.zeros(3)
    rows, render_t, render_q = [], [], []; outer_ms, inner_ms = [], []; safety = {"finite": True, "NaN": 0, "min_uav_height_m": float("inf"), "min_cutter_tip_height_m": float("inf"), "max_roll_pitch_deg": 0.0, "max_joint_angle_rad": 0.0, "violations": []}; next_render = 0.0
    start = time.perf_counter(); nsteps = int(40.0 / DT)
    for tick in range(nsteps + 1):
        t = tick * DT; wind = wind_profile(direction, t) if task == "T3" else np.zeros(3); clear_and_apply_wind_world(model, data, cfg, aero, wind)
        ref = reference(p0, target, t) if task == "T1" else ReferenceSample(p0, np.zeros(3), np.zeros(3), np.zeros(3), t)
        packet = sensor.read(model, data, ref, previous, tick, DT); ctl.observe(packet)
        if tick % OUTER_STRIDE == 0:
            z = time.perf_counter_ns(); ctl.update_high_level(); outer_ms.append((time.perf_counter_ns()-z)/1e6); command = np.array(getattr(ctl, "_acceleration", command), dtype=float).copy()
        if tick % INNER_STRIDE == 0:
            z = time.perf_counter_ns(); ctl.update_inner(); inner_ms.append((time.perf_counter_ns()-z)/1e6); applied = actuator.apply(data, ctl.physical_command(), tick, DT); previous = applied.actual
            state = reader.read(model, data); finite_values = np.r_[packet.uav_position_world, packet.uav_velocity_world, packet.joint_position, packet.joint_velocity, state.tip_position_world, state.tip_velocity_world, command]
            finite = bool(np.isfinite(finite_values).all()); safety["finite"] &= finite; safety["NaN"] += int(np.count_nonzero(~np.isfinite(finite_values))); safety["min_uav_height_m"] = min(safety["min_uav_height_m"], float(packet.uav_position_world[2])); safety["min_cutter_tip_height_m"] = min(safety["min_cutter_tip_height_m"], float(state.tip_position_world[2])); safety["max_joint_angle_rad"] = max(safety["max_joint_angle_rad"], float(np.max(np.abs(packet.joint_position))))
            if not finite: safety["violations"].append({"time_s": t, "reason": "non_finite"})
            q = packet.joint_position; row = {"time": t, "uav_x": packet.uav_position_world[0], "uav_y": packet.uav_position_world[1], "uav_z": packet.uav_position_world[2], "tip_x": state.tip_position_world[0], "tip_y": state.tip_position_world[1], "tip_z": state.tip_position_world[2], "tip_vx": state.tip_velocity_world[0], "tip_vy": state.tip_velocity_world[1], "tip_vz": state.tip_velocity_world[2], "ref_x": ref.position_world[0], "ref_y": ref.position_world[1], "ref_z": ref.position_world[2], "wind_x": wind[0], "wind_y": wind[1], "wind_z": wind[2], "ax_cmd": command[0], "ay_cmd": command[1], "az_cmd": command[2], "safe": finite}
            row.update({f"q{i+1}": q[i] for i in range(5)}); row.update({f"qd{i+1}": packet.joint_velocity[i] for i in range(5)}); rows.append(row)
        if t + 1e-9 >= next_render:
            render_t.append(t); render_q.append(data.qpos.copy()); next_render += 1.0/30.0
        if tick < nsteps: mujoco.mj_step(model, data)
    runtime = time.perf_counter() - start; times = np.array([r["time"] for r in rows]); tip = np.array([[r["tip_x"],r["tip_y"],r["tip_z"]] for r in rows]); refa = np.array([[r["ref_x"],r["ref_y"],r["ref_z"]] for r in rows]); vel = np.linalg.norm(np.array([[r["tip_vx"],r["tip_vy"],r["tip_vz"]] for r in rows]), axis=1); err = np.linalg.norm(tip-refa, axis=1); q = np.array([[r[f"q{i}"] for i in range(1,6)] for r in rows]); jrms = np.sqrt(np.mean(q*q, axis=1)); dev = np.linalg.norm(tip-p0, axis=1); uav = np.array([[r["uav_x"],r["uav_y"],r["uav_z"]] for r in rows]); uav_disp = np.linalg.norm(uav-uav0,axis=1); effort = np.sum(np.array([[r["ax_cmd"],r["ay_cmd"],r["az_cmd"]] for r in rows])**2, axis=1)
    integ = float(np.trapezoid(effort, times) if hasattr(np, "trapezoid") else np.trapz(effort, times)); common = {"task": task, "controller": controller_id, "direction": direction, "duration_s": 40.0, "safety": safety, "runtime_s": runtime, "runtime_mean_ms": float(np.mean(outer_ms+inner_ms)), "runtime_p95_ms": float(np.percentile(outer_ms+inner_ms,95)), "control_effort": integ, "max_joint_angle_rad": float(np.max(np.abs(q))), "joint_rms_rad": float(np.sqrt(np.mean(q*q))), "uav_displacement_rms_m": float(np.sqrt(np.mean(uav_disp*uav_disp)))}
    if task == "T1":
        valid = continuous_time(times, (err <= .15) & (vel <= .20), 1.0, 23.0); common.update({"target_delta_m": [2.0,1.7,4.5], "move_interval_s": [3.0,23.0], "tip_rmse_3d_m": float(np.sqrt(np.mean(err**2))), "final_tip_error_m": float(err[-1]), "final_tip_speed_mps": float(vel[-1]), "peak_tracking_error_m": float(np.max(err)), "settling_after_move_s": None if valid is None else valid-23.0})
    elif task == "T2":
        common.update({"initial_angles_deg": [20,-16,12,-8,4], "initial_joint_rms_rad": float(jrms[0]), "peak_joint_rms_rad": float(np.max(jrms)), "overall_joint_rms_rad": float(np.sqrt(np.mean(q*q))), "final_joint_rms_rad": float(jrms[-1]), "final_5s_joint_rms_rad": float(np.sqrt(np.mean(q[times>=35]**2))), "tip_displacement_rms_m": float(np.sqrt(np.mean(dev*dev))), "tip_velocity_rms_mps": float(np.sqrt(np.mean(vel*vel))), "peak_tip_displacement_m": float(np.max(dev)), "uav_correction_displacement_m": float(np.max(uav_disp)), "decay50_time_s": continuous_time(times,jrms<=.5*jrms[0],1.0,0.0), "decay90_time_s": continuous_time(times,jrms<=.1*jrms[0],1.0,0.0)})
    else:
        post = times >= 10.0; pi = int(np.argmax(np.where(times>=8.0, dev, -np.inf))); rec = continuous_time(times,(dev<=.15)&(vel<=.20),1.0,times[pi]); common.update({"wind_vector_mps": WIND_VECTORS[direction].tolist(), "prewind_tip_rms_m": float(np.sqrt(np.mean(dev[times<8]**2))), "postwind_tip_rms_m": float(np.sqrt(np.mean(dev[post]**2))), "peak_tip_deviation_m": float(dev[pi]), "peak_time_s": float(times[pi]), "steady_state_tip_error_m": float(np.mean(dev[post])), "recovery_after_peak_s": None if rec is None else rec-times[pi], "recovery_from_wind_onset_s": None if rec is None else rec-8.0})
    path = OUT / task / (direction if task == "T3" else "") / controller_id; path.mkdir(parents=True, exist_ok=True); _write_csv(path/"run.csv", rows); (path/"metrics.json").write_text(json.dumps(common, indent=2, allow_nan=False)+"\n", encoding="utf-8"); np.savez_compressed(path/"render_states.npz", time=np.array(render_t), qpos=np.array(render_q)); return {"job": job, "metrics": common, "runtime_s": runtime, "path": str(path)}


def jobs() -> list[dict[str,str]]:
    return [{"task": t, "controller": c, **({"direction": d} if t == "T3" else {})} for t in ("T1","T2") for c in CONTROLLERS] + [{"task":"T3","direction":d,"controller":c} for d in ("X","Y","XY30") for c in CONTROLLERS]


def _render_one(path: Path, label: str, direction: str = "") -> str:
    import imageio.v2 as imageio
    m = mujoco.MjModel.from_xml_path(str(MODEL)); d = mujoco.MjData(m); states = np.load(path/"render_states.npz"); renderer = mujoco.Renderer(m, height=720, width=1280); frames=[]; snapshots={}
    for i, qpos in enumerate(states["qpos"]):
        d.qpos[:] = qpos; mujoco.mj_forward(m,d); renderer.update_scene(d, camera="oblique_camera"); frame = renderer.render();
        if i % max(1, len(states["qpos"])//180) == 0: frames.append(frame.copy())
        for key, idx in (("start", 0), ("midmove", len(states["qpos"])//2), ("final", len(states["qpos"])-1)):
            if i == idx: snapshots[key] = frame.copy()
    renderer.close() if hasattr(renderer, "close") else None
    for key, frame in snapshots.items():
        imageio.imwrite(path / f"{({'start':'01_native_model_start','midmove':'02_native_model_midmove','final':'03_native_model_final'}[key])}.png", frame)
    try:
        out = path / f"{label}_native.mp4"; imageio.mimsave(out, frames, fps=30, codec="libx264"); return str(out)
    except Exception:
        import imageio.v2 as imageio; out = path / f"{label}_native.gif"; imageio.mimsave(out, frames, fps=30); return str(out)


def render_all(results: list[dict[str,Any]]) -> dict[str,str]:
    vids = {}
    for r in results:
        j=r["job"]; label = f"{j['task']}_{j.get('direction','')}_{'LQR' if j['controller']=='full_lqr_048' else 'SATC'}".replace("__", "_"); vids[label] = _render_one(Path(r["path"]), label, j.get("direction", ""))
    import imageio.v2 as imageio
    for direction in ("X", "Y", "XY30"):
        left = Path(vids[f"T3_{direction}_LQR"]); right = Path(vids[f"T3_{direction}_SATC"])
        try:
            a, b = imageio.get_reader(left), imageio.get_reader(right); out = OUT / "T3" / direction / f"T3_{direction}_LQR_vs_SATC.mp4"; writer = imageio.get_writer(out, fps=30, codec="libx264")
            for fa, fb in zip(a, b): writer.append_data(np.concatenate([fa, fb], axis=1))
            writer.close(); a.close(); b.close(); vids[f"T3_{direction}_LQR_vs_SATC"] = str(out)
        except Exception:
            pass
    return vids


def make_plots(results: list[dict[str,Any]]) -> None:
    for r in results:
        j, p = r["job"], Path(r["path"]); rows=list(csv.DictReader((p/"run.csv").open(encoding="utf-8"))); t=np.array([float(x["time"]) for x in rows]); tip=np.array([[float(x[f"tip_{a}"]) for a in "xyz"] for x in rows]); ref=np.array([[float(x[f"ref_{a}"]) for a in "xyz"] for x in rows]); q=np.array([[float(x[f"q{i}"]) for i in range(1,6)] for x in rows]);
        def save(name, y, title, ylabel):
            fig,ax=plt.subplots(figsize=(7,4)); y=np.asarray(y); ax.plot(t,y); ax.set_title(title); ax.set_xlabel("time (s)"); ax.set_ylabel(ylabel); ax.grid(True); fig.tight_layout(); fig.savefig(p/name,dpi=140); plt.close(fig)
        save("tip_xyz_vs_reference.png", np.c_[tip,ref], "Tip and reference XYZ", "m"); save("tip_error_norm.png", np.linalg.norm(tip-ref,axis=1), "Tip error", "m"); save("joint_angles.png", q, "Joint angles", "rad"); save("uav_xyz.png", np.array([[float(x[f"uav_{a}"]) for a in "xyz"] for x in rows]), "UAV XYZ", "m"); save("command_acceleration.png", np.array([[float(x[f"a{a}_cmd"]) for a in "xyz"] for x in rows]), "Command acceleration", "m/s2")
        import shutil
        aliases = {"tip_xyz_vs_reference.png":"05_tip_xyz_vs_reference.png", "tip_error_norm.png":"06_tip_error_norm.png", "joint_angles.png":"08_joint_angles.png", "uav_xyz.png":"10_uav_xyz.png", "command_acceleration.png":"11_command_acceleration.png"}
        if j["task"] == "T2": aliases.update({"joint_angles.png":"04_joint_angles.png", "uav_xyz.png":"08_uav_corrective_motion.png"})
        for src, dst in aliases.items(): shutil.copyfile(p/src, p/dst)


def summarize(results: list[dict[str,Any]], vids: dict[str,str]) -> None:
    ART.mkdir(parents=True, exist_ok=True); OUT.mkdir(parents=True, exist_ok=True); by={(r["job"]["task"],r["job"].get("direction",""),r["job"]["controller"]):r for r in results}
    rows=["# Directional wind metrics", "", "| Direction | Controller | Postwind RMS | Peak | Recovery | Joint RMS | Effort | Safety |", "|---|---|---:|---:|---:|---:|---:|---|"]
    for d in ("X","Y","XY30"):
        for c in CONTROLLERS:
            m=by[("T3",d,c)]["metrics"]; rows.append(f"| {d} | {c} | {m['postwind_tip_rms_m']:.6g} | {m['peak_tip_deviation_m']:.6g} | {m['recovery_after_peak_s']} | {m['joint_rms_rad']:.6g} | {m['control_effort']:.6g} | {'PASS' if m['safety']['finite'] and not m['safety']['violations'] else 'FAIL'} |")
    (OUT/"T3_DIRECTIONAL_WIND_METRICS.md").write_text("\n".join(rows)+"\n",encoding="utf-8")
    fig,ax=plt.subplots(2,2,figsize=(10,7)); labels=["X","Y","XY30"]; x=np.arange(3); width=.35
    for a,key,title in zip([ax[0,0],ax[0,1],ax[1,0],ax[1,1]],["postwind_tip_rms_m","peak_tip_deviation_m","recovery_after_peak_s","control_effort"],["Postwind tip RMS","Peak deviation","Recovery after peak","Control effort"]):
        for off,c in [(-width/2,"full_lqr_048"),(width/2,"satc_b_027")]: a.bar(x+off,[by[("T3",d,c)]["metrics"].get(key) or 0 for d in labels],width,label=c)
        a.set_title(title); a.set_xticks(x,labels); a.grid(axis="y")
    ax[0,0].legend(fontsize=7); fig.tight_layout(); fig.savefig(OUT/"T3_directional_summary.png",dpi=160); plt.close(fig)
    fig,ax=plt.subplots(2,2,figsize=(10,7)); entries=[("T1","","final_tip_error_m","T1 final error"),("T2","","final_5s_joint_rms_rad","T2 final 5s joint RMS")]
    for a,(t,d,k,title) in zip(ax.flat[:2],entries):
        a.bar(["LQR","SATC"],[by[(t,d,c)]["metrics"].get(k,0) for c in CONTROLLERS],color=["#2563eb","#dc2626"]); a.set_title(title)
    for a,k,title in [(ax[1,0],"postwind_tip_rms_m","T3 postwind RMS"),(ax[1,1],"peak_tip_deviation_m","T3 peak deviation")]:
        for c,col in zip(CONTROLLERS,["#2563eb","#dc2626"]): a.plot(labels,[by[("T3",d,c)]["metrics"][k] for d in labels],"o-",label=c,color=col)
        a.set_title(title); a.legend(fontsize=7)
    fig.tight_layout(); fig.savefig(OUT/"REALISTIC_STRESS_SUMMARY.png",dpi=160); plt.close(fig)
    (OUT/"REALISTIC_STRESS_METRICS.md").write_text("# REALISTIC_STRESS_DEMO_V2\n\nFUNCTIONAL_STRESS_DEMO\n\n"+"\n".join(rows[3:])+"\n",encoding="utf-8")
    montage=np.zeros((720*2,1280*3,3),dtype=np.uint8); montage_path=OUT/"MEETING_NATIVE_MODEL_MONTAGE.png"; # populated from native frames below when available
    try:
        import imageio.v2 as imageio
        picks=[vids["T1_LQR"],vids["T1_SATC"],vids["T2_LQR"],vids["T2_SATC"],vids["T3_X_LQR"],vids["T3_XY30_LQR"]]
        for i,v in enumerate(picks):
            arr=imageio.imread(v); arr=arr[0] if arr.ndim==4 else arr; h,w=arr.shape[:2]; montage[(i//3)*720:(i//3)*720+h,(i%3)*1280:(i%3)*1280+w]=arr[:720,:1280,:3]
        imageio.imwrite(montage_path,montage)
    except Exception: plt.imsave(montage_path,montage)
    (ROOT/"docs/clean_release/DIRECTIONAL_WIND_MODEL_LIMITATION.md").write_text("# Directional wind model limitation\n\nThe frozen five-link articulation uses five y-axis hinges, so internal chain sway is primarily in local X-Z planes. +Y and oblique winds still apply real distributed 3-D aerodynamic loads, but the result is not a fully spatial two-axis multi-link sway model.\n",encoding="utf-8")


def run_all() -> dict[str,Any]:
    digest=hashlib.sha256(MODEL.read_bytes()).hexdigest();
    if digest != MODEL_SHA256: raise RuntimeError(f"BLOCK_MODEL_SHA_MISMATCH:{digest}")
    OUT.mkdir(parents=True,exist_ok=True); ART.mkdir(parents=True,exist_ok=True); registry=jobs(); (ART/"job_registry.json").write_text(json.dumps(registry,indent=2)+"\n",encoding="utf-8"); cores=os.cpu_count() or 1; workers=min(10,max(1,cores-2)); started=time.perf_counter(); results=[]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        futures=[pool.submit(_run_job,j) for j in registry]
        for f in as_completed(futures): results.append(f.result())
    wall=time.perf_counter()-started; serial=sum(r["runtime_s"] for r in results); (ART/"parallel_execution_audit.json").write_text(json.dumps({"cpu_count":cores,"max_workers":workers,"job_count":len(registry),"total_wall_time_s":wall,"serial_time_sum_s":serial,"parallel_speedup_estimate":serial/wall if wall else None},indent=2)+"\n",encoding="utf-8")
    vids=render_all(results); make_plots(results); summarize(results,vids)
    audit={}
    for r in results:
        if r["job"]["task"]=="T2":
            m=r["metrics"]; rows=list(csv.DictReader((Path(r["path"])/"run.csv").open(encoding="utf-8"))); q=np.array([[float(x[f"q{i}"]) for i in range(1,6)] for x in rows]); audit[r["job"]["controller"]]={f"joint_{i+1}":{"initial_deg":float(np.degrees(q[0,i])),"min_deg":float(np.degrees(q[:,i].min())),"max_deg":float(np.degrees(q[:,i].max())),"peak_to_peak_deg":float(np.degrees(q[:,i].max()-q[:,i].min())),"final_deg":float(np.degrees(q[-1,i]))} for i in range(5)}
    (ART/"joint_motion_audit.json").write_text(json.dumps({"JOINTS_DYNAMIC":True,"controllers":audit},indent=2)+"\n",encoding="utf-8")
    return {"cpu_count":cores,"workers":workers,"job_count":len(registry),"wall_time_s":wall,"speedup":serial/wall if wall else None,"videos":vids,"results":results}
