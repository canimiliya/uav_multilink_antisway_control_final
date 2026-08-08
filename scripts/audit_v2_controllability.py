"""Audit local 3-D cutter-tip position controllability at the frozen equilibrium."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run(model_path: Path) -> dict:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    cutter_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cutter"))
    quad_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    if min(tip_id, cutter_id, quad_id) < 0:
        raise KeyError("frozen model must contain quadrotor, cutter, and cutter_tip")

    jacp = np.zeros((3, model.nv), dtype=float)
    jacr = np.zeros((3, model.nv), dtype=float)
    mujoco.mj_jacSite(model, data, jacp, jacr, tip_id)
    cutter_jacp = np.zeros((3, model.nv), dtype=float)
    cutter_jacr = np.zeros((3, model.nv), dtype=float)
    mujoco.mj_jacBody(model, data, cutter_jacp, cutter_jacr, cutter_id)

    uav_translation = jacp[:, :3]
    uav_attitude = jacp[:, 3:6]
    uav_pose = jacp[:, :6]
    tip = np.asarray(data.site_xpos[tip_id], dtype=float)
    quad = np.asarray(data.xpos[quad_id], dtype=float)
    cutter_rotation = np.asarray(data.xmat[cutter_id], dtype=float).reshape(3, 3)

    result = {
        "audit": "V2-R0-local-cutter-tip-position-controllability",
        "model_path": "reproducibility/frozen/model/model_5link_controlled.xml",
        "model_sha256": _sha256(model_path),
        "plant": {"free_uav_dof": 6, "suspended_links": 5, "model_nq": int(model.nq), "model_nv": int(model.nv)},
        "equilibrium": {
            "uav_position_m": quad.tolist(),
            "tip_position_m": tip.tolist(),
            "tip_relative_position_m": (tip - quad).tolist(),
            "cutter_axis_world": cutter_rotation[:, 0].tolist(),
            "tip_velocity_m_s": (jacp @ data.qvel).tolist(),
            "cutter_angular_velocity_rad_s": (cutter_jacr @ data.qvel).tolist(),
            "joint_angles_rad": data.qpos[7:].tolist(),
        },
        "jacobian_definition": {
            "output": "world-frame cutter_tip position [x,y,z] in metres",
            "input": "MuJoCo qvel coordinates; free-joint columns 0:3 are UAV translation and 3:6 are UAV angular coordinates",
            "full_qvel_jacobian": jacp.tolist(),
            "uav_translation_jacobian": uav_translation.tolist(),
            "uav_attitude_jacobian": uav_attitude.tolist(),
            "uav_translation_attitude_jacobian": uav_pose.tolist(),
        },
        "rank": {
            "uav_translation": int(np.linalg.matrix_rank(uav_translation, tol=1.0e-12)),
            "uav_translation_attitude": int(np.linalg.matrix_rank(uav_pose, tol=1.0e-12)),
            "singular_values_uav_translation_attitude": np.linalg.svd(uav_pose, compute_uv=False).tolist(),
            "required_position_rank": 3,
        },
        "terminal_state": {
            "target_tip_velocity_m_s": [0.0, 0.0, 0.0],
            "target_cutter_angular_velocity_rad_s": [0.0, 0.0, 0.0],
            "orientation_reference": "equilibrium cutter orientation; arbitrary 3-D orientation is out of scope",
            "static_equilibrium_valid": bool(
                np.allclose(data.qvel, 0.0)
                and np.allclose(tip, [0.225, 0.0, 0.39])
                and np.allclose(cutter_rotation[:, 0], [1.0, 0.0, 0.0])
            ),
        },
    }
    result["local_3d_position_rank"] = result["rank"]["uav_translation_attitude"]
    result["local_3d_task_justified"] = bool(
        result["local_3d_position_rank"] == result["rank"]["required_position_rank"]
        and result["terminal_state"]["static_equilibrium_valid"]
    )
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    model_path = args.repo_root / "reproducibility/frozen/model/model_5link_controlled.xml"
    result = run(model_path)
    text = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8", newline="\n")
    print(text, end="")
    return 0 if result["local_3d_task_justified"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
