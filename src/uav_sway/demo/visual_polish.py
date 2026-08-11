"""Frozen showcase rendering and acceleration-mainline controller audit.

This module is deliberately presentation-only.  It reads the frozen T1/T2
render states, adds visual annotations in the rendered scene, and never
changes the plant, controller parameters, scenario references, or metrics.
OpenGL rendering is intentionally sequential so that the output is
deterministic and safe for MuJoCo's renderer context.
"""
from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import mujoco
import numpy as np

from uav_sway.demo.recoverable_runner import _run_job

ROOT = Path(__file__).resolve().parents[3]
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
MODEL_SHA256 = "19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d"
OUT = ROOT / "outputs/meeting_demo_boundary_v5"
DOC = ROOT / "docs/clean_release"
LINEUP_RUNS = OUT / "_lineup_runs"

T1_MOVE_DURATION_S = 5.0
T2_WIND_MPS = 3.0
INITIAL_ANGLES_DEG = [20.0, -16.0, 12.0, -8.0, 4.0]
TARGET_DELTA_M = [2.0, 1.7, 4.5]

PUBLIC_SHOWCASE_SET: tuple[dict[str, Any], ...] = (
    {
        "name": "PID",
        "class_id": "V3CascadedTaskPID",
        "controller_id": "hybrid_x007_y041_z041",
        "runner_id": "corrected_pid",
        "source_path": "src/uav_sway/v3/controllers.py",
        "config_path": "configs/s3_pid.yaml",
        "evidence_path": "reproducibility/v3/r1r1/pid_freeze.json",
        "acceleration_mainline": True,
        "native_stack_only": False,
        "closed_negative_result": False,
        "debug_candidate": False,
    },
    {
        "name": "Full-LQR",
        "class_id": "V3FullStateLQR",
        "controller_id": "full_lqr_048",
        "runner_id": "full_lqr_048",
        "source_path": "src/uav_sway/v3/controllers.py",
        "config_path": "configs/lqr.yaml",
        "evidence_path": "reproducibility/v3/r1/full_lqr_freeze.json",
        "acceleration_mainline": True,
        "native_stack_only": False,
        "closed_negative_result": False,
        "debug_candidate": False,
    },
    {
        "name": "SATC",
        "class_id": "SATC-OFMPC",
        "controller_id": "satc_b_027",
        "runner_id": "satc_b_027",
        "source_path": "src/uav_sway/v5/satc_ofmpc.py",
        "config_path": "reproducibility/v5/self/self_freeze.json",
        "evidence_path": "reproducibility/v5/self/self_freeze.json",
        "acceleration_mainline": True,
        "native_stack_only": False,
        "closed_negative_result": False,
        "debug_candidate": False,
    },
)


def _assert_model() -> None:
    digest = hashlib.sha256(MODEL.read_bytes()).hexdigest()
    if digest != MODEL_SHA256:
        raise RuntimeError(f"BLOCK_MODEL_SHA_MISMATCH:{digest}")


def frozen_state_path(task: str, controller_id: str) -> Path:
    if task == "T1":
        return OUT / "T1" / "5.0s" / controller_id / "render_states.npz"
    if task == "T2":
        return OUT / "T2" / "3mps" / controller_id / "render_states.npz"
    raise ValueError(f"unsupported showcase task: {task}")


def _pid_state_path() -> Path:
    return LINEUP_RUNS / "T1" / "5.0s" / "corrected_pid" / "render_states.npz"


def _ensure_pid_run() -> dict[str, Any]:
    path = _pid_state_path()
    metrics_path = path.with_name("metrics.json")
    if path.exists() and metrics_path.exists():
        return {"path": str(path.parent), "metrics": json.loads(metrics_path.read_text(encoding="utf-8"))}
    result = _run_job(
        {
            "task": "T1",
            "controller": "corrected_pid",
            "move_duration_s": T1_MOVE_DURATION_S,
            "speed_mps": 0.0,
            "case_dir": "5.0s",
            "output_root": str(LINEUP_RUNS),
            "zero_wind": True,
        }
    )
    return result


def audit_lineup(runtime: dict[str, dict[str, Any]]) -> dict[str, Any]:
    rows: list[dict[str, Any]] = []
    for candidate in PUBLIC_SHOWCASE_SET:
        source_ok = (ROOT / candidate["source_path"]).is_file()
        config_ok = (ROOT / candidate["config_path"]).is_file()
        evidence_ok = (ROOT / candidate["evidence_path"]).is_file()
        runnable = candidate["runner_id"] in runtime
        all_rules = all(
            (
                candidate["acceleration_mainline"],
                source_ok,
                config_ok,
                evidence_ok,
                runnable,
                not candidate["native_stack_only"],
                not candidate["closed_negative_result"],
                not candidate["debug_candidate"],
            )
        )
        row = {
            **candidate,
            "source_exists": source_ok,
            "config_exists": config_ok,
            "evidence_exists": evidence_ok,
            "runnable_on_frozen_t1": runnable,
            "included": all_rules,
            "reason": "all four public-showcase rules passed" if all_rules else "one or more frozen lineup rules failed",
        }
        if runnable:
            row["t1_runtime_path"] = runtime[candidate["runner_id"]]["path"]
            row["t1_stable_recovered"] = bool(runtime[candidate["runner_id"]]["metrics"].get("STABLE_RECOVERED", False))
        rows.append(row)
    return {
        "task": "P3-R1G-VISUAL-POLISH-AND-CONTROLLER-LINEUP-AUDIT-R1",
        "public_showcase_set": rows,
        "frozen_t1": {
            "initial_sway_deg": INITIAL_ANGLES_DEG,
            "target_delta_m": TARGET_DELTA_M,
            "move_duration_s": T1_MOVE_DURATION_S,
            "wind_mps": 0.0,
        },
        "frozen_t2": {
            "move_duration_s": T1_MOVE_DURATION_S,
            "wind_direction": "+X",
            "wind_mps": T2_WIND_MPS,
            "onset_s": 3.0,
            "ramp_s": [3.0, 4.0],
        },
        "extra_controllers": [],
        "controller_retuned": False,
        "model_modified": False,
        "holdout_executed": False,
        "native_stack_revived": False,
    }


def _camera(start: np.ndarray, target: np.ndarray, trackbodyid: int | None = None) -> mujoco.MjvCamera:
    start = np.asarray(start, dtype=float)
    target = np.asarray(target, dtype=float)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING if trackbodyid is not None else mujoco.mjtCamera.mjCAMERA_FREE
    if trackbodyid is not None:
        camera.trackbodyid = int(trackbodyid)
    camera.distance = max(12.0, float(np.linalg.norm(target - start)) * 1.6)
    camera.azimuth = 125.0
    camera.elevation = 18.0
    return camera


def _add_marker(scene: mujoco.MjvScene, position: np.ndarray, rgba: tuple[float, float, float, float], geom_type: int) -> None:
    geom = scene.geoms[scene.ngeom]
    size = np.array([0.11, 0.0, 0.0], dtype=float) if geom_type == mujoco.mjtGeom.mjGEOM_SPHERE else np.array([0.11, 0.11, 0.11], dtype=float)
    mujoco.mjv_initGeom(
        geom,
        geom_type,
        size,
        np.asarray(position, dtype=float),
        np.eye(3, dtype=float).reshape(-1),
        np.asarray(rgba, dtype=np.float32),
    )
    scene.ngeom += 1


def _scene_frame(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    renderer: mujoco.Renderer,
    qpos: np.ndarray,
    camera: mujoco.MjvCamera,
    start: np.ndarray,
    target: np.ndarray,
) -> np.ndarray:
    data.qpos[:] = qpos
    mujoco.mj_forward(model, data)
    renderer.update_scene(data, camera=camera)
    _add_marker(renderer.scene, start, (0.15, 0.85, 1.0, 1.0), mujoco.mjtGeom.mjGEOM_SPHERE)
    _add_marker(renderer.scene, target, (1.0, 0.75, 0.10, 1.0), mujoco.mjtGeom.mjGEOM_BOX)
    return renderer.render().copy()


def _overlay(frame: np.ndarray, *, scenario: str, controller: str, wind: str, time_s: float, stable: bool | None) -> np.ndarray:
    from PIL import Image, ImageDraw

    image = Image.fromarray(frame)
    draw = ImageDraw.Draw(image)
    width, height = image.size
    draw.rectangle((10, 10, min(width - 10, 430), 101), fill=(10, 15, 24), outline=(130, 170, 210), width=2)
    draw.text((20, 18), scenario, fill=(245, 250, 255))
    draw.text((20, 39), f"Wind: {wind}   --> world +X", fill=(210, 235, 255))
    draw.text((20, 60), f"t = {time_s:05.2f} s", fill=(220, 220, 220))
    draw.text((width - 260, 16), controller, fill=(255, 255, 255))
    status = "STABLE_RECOVERED" if stable else "RUNNABLE SHOWCASE"
    draw.text((width - 260, 39), status, fill=(145, 245, 170) if stable else (255, 220, 120))
    legend_y = height - 33
    draw.ellipse((20, legend_y, 32, legend_y + 12), fill=(38, 210, 235))
    draw.text((38, height - 34), "START", fill=(235, 240, 245))
    draw.rectangle((94, legend_y, 106, legend_y + 12), fill=(255, 190, 30))
    draw.text((112, height - 34), "TARGET", fill=(235, 240, 245))
    return np.asarray(image)


def _load_states(path: Path) -> tuple[np.ndarray, np.ndarray]:
    if not path.is_file():
        raise FileNotFoundError(path)
    states = np.load(path)
    return np.asarray(states["time"], dtype=float), np.asarray(states["qpos"], dtype=float)


def render_single(state_path: Path, output: Path, *, scenario: str, wind: str, controller: str, stable: bool | None) -> str:
    import imageio.v2 as imageio

    times, qpos = _load_states(state_path)
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    data.qpos[:] = qpos[0]
    mujoco.mj_forward(model, data)
    start = np.asarray(data.site_xpos[tip_id], dtype=float).copy()
    target = start + np.asarray(TARGET_DELTA_M, dtype=float)
    trackbodyid = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_3"))
    camera = _camera(start, target, trackbodyid)
    renderer = mujoco.Renderer(model, height=720, width=1280)
    frames = [
        _overlay(
            _scene_frame(model, data, renderer, state, camera, start, target),
            scenario=scenario,
            controller=controller,
            wind=wind,
            time_s=float(times[index]),
            stable=stable,
        )
        for index, state in enumerate(qpos)
    ]
    if hasattr(renderer, "close"):
        renderer.close()
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        imageio.mimsave(output, frames, fps=30, codec="libx264")
    except Exception:
        output = output.with_suffix(".gif")
        imageio.mimsave(output, frames, fps=30)
    return str(output)


def render_comparison(left_path: Path, right_path: Path, output: Path, *, scenario: str, wind: str, left_stable: bool, right_stable: bool) -> str:
    import imageio.v2 as imageio

    left_times, left_qpos = _load_states(left_path)
    right_times, right_qpos = _load_states(right_path)
    count = min(len(left_qpos), len(right_qpos))
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    data.qpos[:] = left_qpos[0]
    mujoco.mj_forward(model, data)
    start = np.asarray(data.site_xpos[tip_id], dtype=float).copy()
    target = start + np.asarray(TARGET_DELTA_M, dtype=float)
    trackbodyid = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_3"))
    camera = _camera(start, target, trackbodyid)
    renderer = mujoco.Renderer(model, height=720, width=640)
    frames: list[np.ndarray] = []
    for index in range(count):
        left = _overlay(_scene_frame(model, data, renderer, left_qpos[index], camera, start, target), scenario=scenario, controller="Full-LQR", wind=wind, time_s=float(left_times[index]), stable=left_stable)
        right = _overlay(_scene_frame(model, data, renderer, right_qpos[index], camera, start, target), scenario=scenario, controller="SATC", wind=wind, time_s=float(right_times[index]), stable=right_stable)
        frames.append(np.concatenate((left, right), axis=1))
    if hasattr(renderer, "close"):
        renderer.close()
    output.parent.mkdir(parents=True, exist_ok=True)
    try:
        imageio.mimsave(output, frames, fps=30, codec="libx264")
    except Exception:
        output = output.with_suffix(".gif")
        imageio.mimsave(output, frames, fps=30)
    return str(output)


def _write_audit(audit: dict[str, Any]) -> None:
    DOC.mkdir(parents=True, exist_ok=True)
    (DOC / "PUBLIC_SHOWCASE_CONTROLLER_SET.json").write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8")
    lines = [
        "# Public Controller Lineup Audit",
        "",
        "Task: `P3-R1G-VISUAL-POLISH-AND-CONTROLLER-LINEUP-AUDIT-R1`.",
        "",
        "The audit is presentation-only. Frozen controllers, model, scenarios, and metrics were not changed; no Holdout or Native-Stack execution was used.",
        "",
        "## Frozen public showcase scenarios",
        "",
        "- T1: large-sway transfer, 5.0 s move, zero wind.",
        "- T2: the same transfer with world +X wind at 3.0 m/s, onset 3.0 s, ramp 3.0--4.0 s.",
        "- T3 remains historical only: LQR 3 m/s and SATC 5 m/s.",
        "",
        "## Controller decisions",
        "",
        "| Controller | Class | ID | Source | Config/evidence | Runnable on frozen T1 | Included |",
        "|---|---|---|---|---|---:|---:|",
    ]
    for row in audit["public_showcase_set"]:
        lines.append(
            f"| {row['name']} | `{row['class_id']}` | `{row['controller_id']}` | `{row['source_path']}` | `{row['config_path']}` / `{row['evidence_path']}` | {row['runnable_on_frozen_t1']} | {row['included']} |"
        )
    lines += [
        "",
        "All included controllers satisfy the acceleration-mainline, identifiable frozen provenance, frozen-scenario runnable, and non-debug/non-Native-Stack rules. The PID video is a supplementary runnable showcase; its T1 stability status is reported from the frozen run and is not converted into a new science conclusion.",
        "",
        "`PUBLIC_SHOWCASE_SET` is intentionally limited to the three audited candidates above; no extra controller was claimed.",
        "",
    ]
    (DOC / "CONTROLLER_LINEUP_AUDIT.md").write_text("\n".join(lines), encoding="utf-8")


def run_all() -> dict[str, Any]:
    _assert_model()
    OUT.mkdir(parents=True, exist_ok=True)
    gallery = OUT / "gallery"
    gallery.mkdir(parents=True, exist_ok=True)

    runtime: dict[str, dict[str, Any]] = {}
    for candidate in PUBLIC_SHOWCASE_SET:
        if candidate["runner_id"] == "corrected_pid":
            runtime[candidate["runner_id"]] = _ensure_pid_run()
        else:
            state_path = frozen_state_path("T1", candidate["runner_id"])
            metrics = json.loads(state_path.with_name("metrics.json").read_text(encoding="utf-8"))
            runtime[candidate["runner_id"]] = {"path": str(state_path.parent), "metrics": metrics}

    audit = audit_lineup(runtime)
    _write_audit(audit)
    included = [row for row in audit["public_showcase_set"] if row["included"]]
    if not included:
        raise RuntimeError("BLOCK_NO_PUBLIC_SHOWCASE_CONTROLLER")

    videos: dict[str, str] = {}
    t1_lqr = frozen_state_path("T1", "full_lqr_048")
    t1_satc = frozen_state_path("T1", "satc_b_027")
    t2_lqr = frozen_state_path("T2", "full_lqr_048")
    t2_satc = frozen_state_path("T2", "satc_b_027")
    t1_lqr_metrics = runtime["full_lqr_048"]["metrics"]
    t1_satc_metrics = runtime["satc_b_027"]["metrics"]
    t2_lqr_metrics = json.loads(t2_lqr.with_name("metrics.json").read_text(encoding="utf-8"))
    t2_satc_metrics = json.loads(t2_satc.with_name("metrics.json").read_text(encoding="utf-8"))

    videos["T1_primary"] = render_comparison(t1_lqr, t1_satc, OUT / "T1" / "T1_FINAL_LQR_vs_SATC_WIDECAM.mp4", scenario="Large-sway transfer", wind="0.0 m/s", left_stable=bool(t1_lqr_metrics.get("STABLE_RECOVERED")), right_stable=bool(t1_satc_metrics.get("STABLE_RECOVERED")))
    videos["T2_primary"] = render_comparison(t2_lqr, t2_satc, OUT / "T2" / "T2_FINAL_LQR_vs_SATC_WIDECAM_WINDHUD.mp4", scenario="Large-sway transfer with +X wind", wind="+X 3.0 m/s", left_stable=bool(t2_lqr_metrics.get("STABLE_RECOVERED")), right_stable=bool(t2_satc_metrics.get("STABLE_RECOVERED")))

    for candidate in included:
        runner_id = candidate["runner_id"]
        state_path = Path(runtime[runner_id]["path"]) / "render_states.npz"
        filename = f"T1_{candidate['controller_id']}.mp4"
        videos[f"gallery_{candidate['controller_id']}"] = render_single(state_path, gallery / filename, scenario="Large-sway transfer", wind="0.0 m/s", controller=candidate["name"], stable=bool(runtime[runner_id]["metrics"].get("STABLE_RECOVERED")))

    (gallery / "CONTROLLER_GALLERY.md").write_text(
        "# T1 Controller Gallery\n\n" + "\n".join(f"- `{row['name']}`: `T1_{row['controller_id']}.mp4`" for row in included) + "\n",
        encoding="utf-8",
    )
    manifest = {
        "task": "P3-R1G-VISUAL-POLISH-AND-CONTROLLER-LINEUP-AUDIT-R1",
        "start_head": "8d9768f9f49091c322a6f6c51feb6f303fb3cfbb",
        "model_sha256": MODEL_SHA256,
        "controller_retuned": False,
        "model_modified": False,
        "holdout_executed": False,
        "native_stack_revived": False,
        "render_sequential": True,
        "resolution": "1280x720",
        "fps": 30,
        "videos": videos,
    }
    (OUT / "P3_R1G_RENDER_MANIFEST.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    return {"audit": audit, "videos": videos, "manifest": manifest}


if __name__ == "__main__":
    print(json.dumps(run_all(), indent=2))
