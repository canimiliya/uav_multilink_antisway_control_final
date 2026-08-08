"""Build the V3-R1 development-only protocol and parameter grids."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility" / "v3" / "r0"
R1 = ROOT / "reproducibility" / "v3" / "r1"


def dump(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def targets() -> list[dict]:
    axis = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]
    diagonal = [[1, 1, 0], [-1, -1, 0], [1, -1, 0], [-1, 1, 0], [1, 0, 1], [-1, 0, -1], [1, 0, -1], [-1, 0, 1], [0, 1, 1], [0, -1, -1], [0, 1, -1], [0, -1, 1]]
    result = []
    for index, vector in enumerate(axis):
        direction = np.asarray(vector, dtype=float) / np.linalg.norm(vector)
        result.append({"target_id": f"axis_{index:02d}", "direction": direction.tolist(), "radius_m": 0.15, "delta_tip_m": (0.15 * direction).tolist()})
    for index, vector in enumerate(diagonal):
        direction = np.asarray(vector, dtype=float) / np.linalg.norm(vector)
        result.append({"target_id": f"face_diagonal_{index:02d}", "direction": direction.tolist(), "radius_m": 0.20, "delta_tip_m": (0.20 * direction).tolist()})
    return result


def development_manifest() -> dict:
    target_rows = targets()
    samples = []
    for target in target_rows:
        samples.append({"sample_id": f"calm_{target['target_id']}", "scenario": "CALM_3D_SETPOINT", "target": target, "wind": {"kind": "calm", "speed_m_s": 0.0}, "execution_allowed": True})
    for speed in (1.5, 3.0):
        for target in target_rows:
            samples.append({"sample_id": f"constant_{speed:g}_{target['target_id']}", "scenario": "WIND_3D_SETPOINT", "target": target, "wind": {"kind": "constant", "speed_m_s": speed}, "execution_allowed": True})
    for seed in range(2000, 2020):
        target = target_rows[(seed - 2000) % len(target_rows)]
        samples.append({"sample_id": f"stochastic_{seed}", "scenario": "WIND_3D_SETPOINT", "target": target, "wind": {"kind": "stochastic", "speed_m_s": 0.0, "seed": seed}, "execution_allowed": True})
    samples.append({"sample_id": "ramp_wind_equilibrium_hold", "scenario": "RAMP_WIND_EQUILIBRIUM_HOLD", "target": {"target_id": "equilibrium", "direction": [0.0, 0.0, 0.0], "radius_m": 0.0, "delta_tip_m": [0.0, 0.0, 0.0]}, "wind": {"kind": "ramp", "speed_m_s": 3.0}, "execution_allowed": True})
    if len(samples) != 75:
        raise AssertionError(len(samples))
    return {"split": "development", "duration_s": 12.0, "outer_period_s": 0.05, "log_period_s": 0.005, "target_issue_times_s": {"CALM_3D_SETPOINT": 1.0, "WIND_3D_SETPOINT": 3.0, "RAMP_WIND_EQUILIBRIUM_HOLD": None}, "targets": target_rows, "development_seeds": list(range(2000, 2020)), "samples": samples, "sample_count": len(samples)}


def grids() -> dict:
    pid_axes = {
        "x": {"kp": [0.4, 0.8, 1.6], "kd": [2.0, 4.0, 6.0], "ki": [0.0, 0.1]},
        "y": {"kp": [0.05, 0.10, 0.20], "kd": [0.25, 0.50, 1.00], "ki": [0.0, 0.05]},
        "z": {"kp": [3.0, 6.0, 9.0], "kd": [2.0, 3.5, 5.0], "ki": [0.0, 0.1]},
    }
    axis_rows = {}
    for axis, values in pid_axes.items():
        axis_rows[axis] = [{"candidate_id": f"pid_{axis}_{i:02d}", "kp": kp, "kd": kd, "ki": ki} for i, (kp, kd, ki) in enumerate(( (kp, kd, ki) for kp in values["kp"] for kd in values["kd"] for ki in values["ki"] ))]
    pid_combinations = [{"candidate_id": f"pid_combo_{i:03d}", "x": x, "y": y, "z": z} for i, (x, y, z) in enumerate(((x, y, z) for x in range(2) for y in range(2) for z in range(2)))]
    full_lqr = []
    for i, (qp, qv, qa, qj, qjd, r) in enumerate(((qp, qv, qa, qj, qjd, r) for qp in (20.0, 80.0) for qv in (4.0, 12.0) for qa in (5.0, 20.0) for qj in (20.0, 80.0) for qjd in (3.0, 12.0) for r in (0.5, 2.0))):
        full_lqr.append({"candidate_id": f"full_lqr_{i:03d}", "q_position": qp, "q_velocity": qv, "q_attitude": qa, "q_joint_angle": qj, "q_joint_velocity": qjd, "r": r})
    task_lqr = [{"candidate_id": f"task_lqr_{i:03d}", "w_p": wp, "w_theta": wt, "r": r} for i, (wp, wt, r) in enumerate(((wp, wt, r) for wp in (20.0, 80.0, 320.0) for wt in (5.0, 20.0, 80.0) for r in (0.5, 1.0, 2.0)))]
    return {"pid_axis": axis_rows, "pid_combination": pid_combinations, "full_lqr": full_lqr, "task_lqr": task_lqr}


def main() -> int:
    R1.mkdir(parents=True, exist_ok=True)
    manifest = development_manifest()
    grid = grids()
    protocol = {
        "task": "V3-R1-THREE-FULL-3D-TRADITIONAL-BASELINES-DEVELOPMENT-AND-FREEZE-R1",
        "start_head": "f27a79d24689d2fd9d8444df58d3d7dae0d6ee64",
        "protected_paths": ["reproducibility/v2/**", "reproducibility/v3/r0/**", "main", "v1.0.0", "v2-research-final-2026-08-09"],
        "controller_output": {"frame": "world", "components": ["ax", "ay", "az"], "amplitude_limit_m_s2": 2.0, "slew_limit_m_s2_per_outer_update": 0.25, "outer_period_s": 0.05},
        "common_inner_loop": "frozen Udaan geometric inner loop; no per-method gain override",
        "target_mapping": "p_uav_ref = p_tip_star - r_tip_equilibrium; equilibrium geometry is read from the frozen model",
        "pid": {"controller": "3D Task PID", "integral_absolute_limit": 1.0, "anti_windup": "per-axis conditional", "axis_screen_cases": ["+axis calm", "-axis calm", "+axis wind 3.0", "-axis wind 3.0"], "axis_screen_runs": 72, "combination_grid_size": 8},
        "full_state_lqr": {"controller": "3D Full-State LQR", "state_dimension": 20, "input_dimension": 3, "gain_shape": [3, 20], "grid_size": 64, "stability_gate": "DARE finite, P positive definite, rho(A-BK)<1"},
        "task_weighted_lqr": {"controller": "3D Task-Weighted LQR", "grid_size": 27, "task_metric_outputs": ["C_pos", "C_vel", "C_dir", "C_omega_perp"], "stability_gate": "DARE finite, P positive definite, rho(A-BK)<1"},
        "stage1_core_case_count": 13,
        "stage1_selection": ["safety-valid count descending", "task-success count descending", "3D position RMSE ascending", "acquisition median ascending", "ramp steady error ascending", "ramp peak error ascending", "orientation RMSE ascending", "total acceleration effort ascending", "candidate ID ascending"],
        "stage2": {"top_k": 3, "evaluation_samples": 75, "raw_trace_policy": "aggregate plus diagnostics; retained selected representative cases only"},
        "holdout_execution_allowed": False,
        "advanced_self_started": False,
        "advanced_paper_selected": False,
    }
    dump(R1 / "r1_protocol.json", protocol)
    dump(R1 / "development_evaluation_manifest.json", manifest)
    dump(R1 / "pid_axis_grids.json", grid["pid_axis"])
    dump(R1 / "pid_combination_grid.json", {"grid": grid["pid_combination"], "grid_size": len(grid["pid_combination"])})
    dump(R1 / "full_lqr_grid.json", {"grid": grid["full_lqr"], "grid_size": len(grid["full_lqr"])})
    dump(R1 / "task_lqr_grid.json", {"grid": grid["task_lqr"], "grid_size": len(grid["task_lqr"])})
    dump(R1 / "protocol_sha256.json", {"files": {name: hashlib.sha256(path.read_bytes()).hexdigest() for name in ("r1_protocol.json", "development_evaluation_manifest.json", "pid_axis_grids.json", "pid_combination_grid.json", "full_lqr_grid.json", "task_lqr_grid.json") for path in [R1 / name]}})
    print(json.dumps({"result": "V3_R1_PROTOCOL_READY", "development_count": manifest["sample_count"], "pid_axis_grids": {axis: len(rows) for axis, rows in grid["pid_axis"].items()}, "pid_combinations": len(grid["pid_combination"]), "full_lqr": len(grid["full_lqr"]), "task_lqr": len(grid["task_lqr"])}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
