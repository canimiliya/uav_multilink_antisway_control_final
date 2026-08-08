"""Build the frozen V2-R1 development and holdout sample manifests."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from uav_sway.task_space.reference import build_equilibrium_task_pose


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v2/r1"
START_HEAD = "562bb1c99b43d25a07386ec5e3e99ec927b90cb6"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def target_rows(equilibrium: np.ndarray, radius: float, directions: list[tuple[str, np.ndarray]]) -> list[dict]:
    rows = []
    for name, direction in directions:
        delta = radius * direction
        rows.append({
            "target_id": f"{name}_r{radius:.2f}",
            "direction": name,
            "radius_m": radius,
            "delta_tip_m": delta.tolist(),
            "tip_target_world_m": (equilibrium + delta).tolist(),
            "target_orientation_axis_world": [1.0, 0.0, 0.0],
            "target_angular_velocity_rad_s": [0.0, 0.0, 0.0],
        })
    return rows


def main() -> int:
    model_path = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    data.eq_active[:] = 0
    mujoco.mj_forward(model, data)
    pose = build_equilibrium_task_pose(model, data, model_path)
    equilibrium = np.asarray(data.site_xpos[int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))], dtype=float)
    axis_names = [
        ("+x", np.array([1.0, 0.0, 0.0])), ("-x", np.array([-1.0, 0.0, 0.0])),
        ("+y", np.array([0.0, 1.0, 0.0])), ("-y", np.array([0.0, -1.0, 0.0])),
        ("+z", np.array([0.0, 0.0, 1.0])), ("-z", np.array([0.0, 0.0, -1.0])),
    ]
    diagonal = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                name = f"diag_{'p' if sx > 0 else 'm'}{'p' if sy > 0 else 'm'}{'p' if sz > 0 else 'm'}"
                diagonal.append((name, np.asarray([sx, sy, sz], dtype=float) / np.sqrt(3.0)))
    targets = target_rows(equilibrium, 0.15, axis_names) + target_rows(equilibrium, 0.25, axis_names)
    dev = []
    for target in targets:
        for mode, wind in (("calm", {"kind": "constant", "speed_m_s": 0.0}), ("wind_1p5", {"kind": "constant", "speed_m_s": 1.5}), ("wind_3p0", {"kind": "constant", "speed_m_s": 3.0})):
            dev.append({
                "sample_id": f"{mode}__{target['target_id']}", "split": "development", "execution_allowed": True,
                "scenario": "CALM_3D_SETPOINT" if mode == "calm" else "WIND_3D_SETPOINT", "target": target,
                "wind": wind, "wind_direction_world": [1.0, 0.0, 0.0],
                "target_issue_time_s": 1.0 if mode == "calm" else 3.0, "duration_s": 12.0,
            })
    dev = [row for row in dev if row["scenario"] != "WIND_3D_SETPOINT" or row["target"]["direction"] in {"+x", "-x", "+y", "-y", "+z", "-z"}]
    stochastic = []
    for seed in range(20):
        target = targets[seed % 12]
        stochastic.append({
            "sample_id": f"stochastic_dev_{seed:04d}", "split": "development", "execution_allowed": True,
            "scenario": "WIND_3D_SETPOINT", "target": target, "wind": {"kind": "frozen_random_wind", "seed": seed},
            "wind_direction_world": [1.0, 0.0, 0.0], "target_issue_time_s": 3.0, "duration_s": 12.0,
        })
    dev.extend(stochastic)
    dev.append({
        "sample_id": "ramp_dev_0_to_3p0", "split": "development", "execution_allowed": True,
        "scenario": "RAMP_WIND_EQUILIBRIUM_HOLD", "target": target_rows(equilibrium, 0.0, [("equilibrium", np.zeros(3))])[0],
        "wind": {"kind": "linear_ramp", "start_speed_m_s": 0.0, "end_speed_m_s": 3.0, "ramp_window_s": [2.0, 8.0]},
        "wind_direction_world": [1.0, 0.0, 0.0], "target_issue_time_s": None, "duration_s": 12.0,
    })
    # The validation bank is exactly 12 + 12 + 12 + 20 + 1 samples.
    # Stage-1 tuning is declared separately and is not part of this 57-sample bank.
    holdout = []
    for seed in range(1000, 1020):
        target = target_rows(equilibrium, 0.20, [diagonal[(seed - 1000) % 8]])[0]
        holdout.append({
            "sample_id": f"holdout_{seed:04d}", "split": "holdout", "execution_allowed": False,
            "scenario": "WIND_3D_SETPOINT", "target": target,
            "wind": {"kind": "frozen_random_wind", "seed": seed}, "wind_direction_world": [1.0, 0.0, 0.0],
            "target_issue_time_s": 3.0, "duration_s": 12.0,
        })
    holdout.append({
        "sample_id": "holdout_constant_2p25", "split": "holdout", "execution_allowed": False,
        "scenario": "WIND_3D_SETPOINT", "target": target_rows(equilibrium, 0.20, [diagonal[0]])[0],
        "wind": {"kind": "constant", "speed_m_s": 2.25}, "wind_direction_world": [1.0, 0.0, 0.0],
        "target_issue_time_s": 3.0, "duration_s": 12.0,
    })
    holdout.append({
        "sample_id": "holdout_constant_3p75", "split": "holdout", "execution_allowed": False,
        "scenario": "WIND_3D_SETPOINT", "target": target_rows(equilibrium, 0.20, [diagonal[1]])[0],
        "wind": {"kind": "constant", "speed_m_s": 3.75}, "wind_direction_world": [1.0, 0.0, 0.0],
        "target_issue_time_s": 3.0, "duration_s": 12.0,
    })
    holdout.append({
        "sample_id": "holdout_ramp_0_to_3p75", "split": "holdout", "execution_allowed": False,
        "scenario": "RAMP_WIND_EQUILIBRIUM_HOLD", "target": target_rows(equilibrium, 0.0, [("equilibrium", np.zeros(3))])[0],
        "wind": {"kind": "linear_ramp", "start_speed_m_s": 0.0, "end_speed_m_s": 3.75, "ramp_window_s": [2.0, 8.0]},
        "wind_direction_world": [1.0, 0.0, 0.0], "target_issue_time_s": None, "duration_s": 12.0,
    })
    if len(dev) != 57 or len(targets) != 12 or len(holdout) != 23:
        raise AssertionError((len(dev), len(targets), len(holdout)))
    min_holdout_z = min(float(row["target"]["tip_target_world_m"][2]) for row in holdout)
    if min_holdout_z <= 0.05:
        raise AssertionError("holdout target violates frozen safety floor")
    OUT.mkdir(parents=True, exist_ok=True)
    model_sha = sha256(model_path)
    contract = {
        "contract": "V2-R1-sample-bank-r1", "start_head": START_HEAD, "model_sha256": model_sha,
        "equilibrium_tip_position_world_m": equilibrium.tolist(), "tip_relative_equilibrium_m": pose.tip_relative_position_m.tolist(),
        "equilibrium_cutter_axis_world": pose.cutter_axis_world.tolist(), "target_orientation_is_frozen": True,
        "development_target_count": 12, "development_targets": targets, "development_validation_sample_count": len(dev),
        "development_seed_namespace": list(range(20)), "holdout_target_count": 8,
        "holdout_target_radius_m": 0.20, "holdout_seed_namespace": list(range(1000, 1020)),
        "holdout_execution_forbidden": True, "holdout_min_target_tip_z_m": min_holdout_z,
        "tuning_core": {"target_directions": ["+x", "-x"], "radii_m": [0.15, 0.25], "wind_strength_m_s": 3.0, "samples_per_candidate": 8},
        "timing": {"calm_issue_s": 1.0, "wind_issue_s": 3.0, "duration_s": 12.0, "ramp_window_s": [2.0, 8.0]},
    }
    (OUT / "sample_bank_contract.json").write_text(json.dumps(contract, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    (OUT / "development_manifest.json").write_text(json.dumps({"execution_allowed": True, "samples": dev}, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    (OUT / "holdout_manifest.json").write_text(json.dumps({"execution_allowed": False, "samples": holdout}, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    (OUT / "control_interface_contract.json").write_text(json.dumps({
        "contract": "V2-R1-3d-control-interface-r1", "start_head": START_HEAD,
        "external_task_output": "cutter_tip_position_world", "target_to_uav_mapping": "p_uav_ref = p_tip_star - tip_relative_equilibrium",
        "formal_metric_target": "external_cutter_target_only", "cutter_orientation_axis_world": [1.0, 0.0, 0.0],
        "target_angular_velocity_rad_s": [0.0, 0.0, 0.0], "x_channel": "controller-specific ax only",
        "shared_yz_scaffold": {"ay_kp": 1.5, "ay_kd": 2.0, "az_kp": 4.0, "az_kd": 3.5, "ay_abs_max_m_s2": 2.0, "az_abs_max_m_s2": 2.0, "axis_slew_max_m_s2_per_update": 0.25},
        "all_methods_share_yz_scaffold": True, "advanced_methods_may_not_retune_yz": True,
    }, indent=2) + "\n", encoding="utf-8", newline="\n")
    (OUT / "shared_yz_grid.json").write_text(json.dumps({
        "selection_rule": "separate y and z development-only tuning; no joint 81-grid",
        "y": [{"kp": kp, "kd": kd} for kp in [1.0, 1.5, 2.5] for kd in [1.5, 2.0, 3.0]],
        "z": [{"kp": kp, "kd": kd} for kp in [2.5, 4.0, 6.0] for kd in [2.5, 3.5, 5.0]],
        "selection_samples": "calm +/-y or +/-z development targets only",
    }, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"development": len(dev), "holdout": len(holdout), "targets": len(targets), "model_sha256": model_sha}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
