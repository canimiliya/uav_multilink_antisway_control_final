from pathlib import Path

import mujoco
import numpy as np

from uav_sway.demo.visual_polish import (
    MODEL_SHA256,
    PUBLIC_SHOWCASE_SET,
    T1_MOVE_DURATION_S,
    T2_WIND_MPS,
    _camera,
    _scene_frame,
    audit_lineup,
)


def test_public_showcase_set_is_exactly_the_audited_acceleration_mainline():
    assert [row["controller_id"] for row in PUBLIC_SHOWCASE_SET] == [
        "hybrid_x007_y041_z041",
        "full_lqr_048",
        "satc_b_027",
    ]
    assert all(row["acceleration_mainline"] for row in PUBLIC_SHOWCASE_SET)
    assert all(not row["native_stack_only"] for row in PUBLIC_SHOWCASE_SET)


def test_visual_polish_keeps_frozen_scenario_contract():
    assert T1_MOVE_DURATION_S == 5.0
    assert T2_WIND_MPS == 3.0
    assert MODEL_SHA256 == "19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d"


def test_camera_is_wide_oblique_free_camera():
    camera = _camera([0.0, 0.0, 0.5], [2.0, 1.7, 5.0], trackbodyid=6)
    assert camera.type == 1
    assert camera.trackbodyid == 6
    assert camera.distance >= 12.0
    assert camera.elevation == 18.0


def test_audit_requires_runtime_proof_for_inclusion(tmp_path: Path):
    runtime = {
        row["runner_id"]: {"path": str(tmp_path), "metrics": {"STABLE_RECOVERED": True}}
        for row in PUBLIC_SHOWCASE_SET
    }
    audit = audit_lineup(runtime)
    assert all(row["included"] for row in audit["public_showcase_set"])
    assert audit["extra_controllers"] == []


def test_real_mujoco_overlay_scene_frame_smoke():
    model_path = Path(__file__).resolve().parents[2] / "reproducibility/frozen/model/model_5link_controlled.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)
    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    start = data.site_xpos[tip_id].copy()
    target = start + np.array([2.0, 1.7, 4.5])
    renderer = mujoco.Renderer(model, height=720, width=640)
    frame = _scene_frame(model, data, renderer, data.qpos.copy(), _camera(start, target, trackbodyid=6), start, target)
    assert frame.shape == (720, 640, 3)
    assert frame.dtype == np.uint8
