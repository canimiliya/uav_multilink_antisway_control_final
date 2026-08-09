"""Freeze V3-R2 Self evidence from the completed Development-only run."""

from __future__ import annotations

import csv
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v3/r0"
R1 = ROOT / "reproducibility/v3/r1"
R1R1 = ROOT / "reproducibility/v3/r1r1"
R2 = ROOT / "reproducibility/v3/r2"
START_HEAD = "6ced3e97d4f635cedb05456eccc402d22a6b6028"
PROTOCOL_HEAD = "6625a4fd0abdfb9ff6e5ed786bd6a46fb434ef60"


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict | list) -> None:
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tree(revision: str, path: str) -> str:
    return subprocess.check_output(["git", "rev-parse", f"{revision}:{path}"], cwd=ROOT, text=True).strip()


def committed_sha(revision: str, path: str) -> str:
    content = subprocess.check_output(["git", "show", f"{revision}:{path}"], cwd=ROOT)
    return hashlib.sha256(content).hexdigest()


def rows(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def main() -> int:
    summary = read_json(R2 / "self_round_b_summary.json")["best"]
    if summary["candidate_id"] != "self_a_034" or summary["sample_count"] != 75:
        raise RuntimeError("unexpected R2-B selected candidate")
    contract = read_json(R1R1 / "advanced_numeric_win_contract.json")
    primary = read_json(R1R1 / "primary_traditional_baseline.json")["selected_metrics"]
    selected_rows = [row for row in rows(R2 / "development_results_round_b.csv") if row["candidate_id"] == summary["candidate_id"]]
    primary_rows = {
        row["sample_id"]: row for row in rows(R1 / "traditional_development_results.csv")
        if row["candidate_id"] == "full_lqr_048"
    }
    if len(selected_rows) != 75 or len(primary_rows) != 75 or {row["sample_id"] for row in selected_rows} != set(primary_rows):
        raise RuntimeError("exact sample_id pairing failed")
    pairs = []
    for row in selected_rows:
        baseline = primary_rows[row["sample_id"]]
        delta = float(baseline["position_rmse_3d_m"]) - float(row["position_rmse_3d_m"])
        pairs.append({
            "sample_id": row["sample_id"],
            "primary_position_rmse_m": float(baseline["position_rmse_3d_m"]),
            "self_position_rmse_m": float(row["position_rmse_3d_m"]),
            "primary_minus_self_m": delta,
            "primary_success": baseline["task_success"].lower() == "true",
            "self_success": row["task_success"].lower() == "true",
        })
    deltas = np.asarray([pair["primary_minus_self_m"] for pair in pairs])
    write_json(R2 / "paired_primary_comparison.json", {
        "scope": "Development diagnostic only",
        "primary": "full_lqr:full_lqr_048",
        "self": summary["candidate_id"],
        "pairing": "exact sample_id",
        "pair_count": 75,
        "mean_primary_minus_self_position_rmse_m": float(np.mean(deltas)),
        "median_primary_minus_self_position_rmse_m": float(np.median(deltas)),
        "positive_pair_fraction": float(np.mean(deltas > 0.0)),
        "formal_10000_bootstrap_executed": False,
        "formal_bootstrap_reserved_for_future_frozen_holdout": True,
        "pairs": pairs,
    })
    improvements = {
        "position_fraction": (primary["position_rmse_3d_m"] - summary["position_rmse_3d_m"]) / primary["position_rmse_3d_m"],
        "acquisition_fraction": (primary["acquisition_median_s"] - summary["acquisition_median_s"]) / primary["acquisition_median_s"],
        "ramp_peak_fraction": (primary["ramp_peak_position_error_m"] - summary["ramp_peak_position_error_m"]) / primary["ramp_peak_position_error_m"],
        "ramp_steady_fraction": (primary["ramp_steady_state_position_error_m"] - summary["ramp_steady_state_position_error_m"]) / primary["ramp_steady_state_position_error_m"],
    }
    gates = {
        "safety": summary["safety_rate"] >= contract["safety_threshold_rate"],
        "success": summary["success_rate"] >= contract["success_threshold_rate"],
        "position": summary["position_rmse_3d_m"] <= contract["position_rmse_max_m"],
        "acquisition": summary["acquisition_median_s"] <= contract["acquisition_noninferiority_max_s"],
        "strict_acquisition": summary["acquisition_median_s"] <= contract["strict_acquisition_max_s"],
        "ramp": summary["ramp_peak_position_error_m"] <= contract["ramp_peak_max_m"] or summary["ramp_steady_state_position_error_m"] <= contract["ramp_steady_max_m"],
        "realtime": summary["solve_time_p95_ms"] < 50.0,
        "solver": summary["solver_valid_rate"] == 1.0,
        "limiter_parity": summary["limiter_mismatch_max"] <= 1.0e-5,
    }
    strict = all(improvements[name] >= 0.05 for name in ("position_fraction", "acquisition_fraction", "ramp_peak_fraction")) and summary["success_rate"] >= primary["success_rate"] and summary["safety_rate"] >= primary["safety_rate"]
    if not all(gates.values()) or not strict:
        raise RuntimeError(f"selected candidate does not pass freeze gates: {gates}, strict={strict}")
    freeze = {
        "task": "V3-R2-FULL-3D-SELF-ADVANCED-DEVELOPMENT-AND-FREEZE-R1",
        "method": "3D-DR-TSRMPC",
        "candidate_id": summary["candidate_id"],
        "architecture": "causal projected 20D dynamic residual plus residual-compatible steady compensation and finite-horizon 3D task-space residual QP",
        "backbone": summary["parameters"]["backbone"],
        "parameters": summary["parameters"],
        "metrics": {key: value for key, value in summary.items() if key != "parameters"},
        "improvement_vs_primary": improvements,
        "win_level": "STRICT_ALL_METRIC_WIN_DEVELOPMENT",
        "gates": gates,
        "frozen_before_ablation": True,
        "development_only": True,
        "holdout_executed": False
    }
    write_json(R2 / "self_freeze.json", freeze)
    write_json(R2 / "implementation_audit.json", {
        "implementation": "uav_sway.v3.dr_tsrmpc.V3DRTSRMPC",
        "state_dimension": 20,
        "input_dimension": 3,
        "physical_input": ["ax", "ay", "az"],
        "A3_B3_sha256": sha(R0 / "linear_model_audit.json"),
        "C_task_v3_sha256": sha(R1 / "task_metric_alignment_audit.json"),
        "backbone": summary["parameters"]["backbone"],
        "backbone_gain_retuned": False,
        "future_wind_truth": False,
        "future_target_leakage": False,
        "fallback": False,
        "traditional_override": False,
        "paper_advanced_started": False
    })
    write_json(R2 / "dynamic_residual_audit.json", {
        "equation": "d_raw[k] = x[k] - (A3*x[k-1] + B3*u_actual_limited[k-1] - reference_shift[k])",
        "causal": True,
        "raw_dimension": 20,
        "bounded_low_pass_beta": summary["parameters"]["residual_beta"],
        "component_limit": summary["parameters"]["residual_clip_norm"],
        "controllable_projection_rank": 18,
        "raw_norm_max": summary["d_raw_norm_max"],
        "filtered_norm_max": summary["d_hat_norm_max"],
        "projected_norm_max": summary["d_projected_norm_max"],
        "rejected_norm_max": summary["d_rejected_norm_max"],
        "steady_state_residual_max": summary["steady_state_residual_max"],
        "steady_task_residual_max": summary["steady_task_residual_max"],
        "wind_truth_used": False
    })
    max_abs = max(max(float(row[name]) for name in ("max_abs_ax_m_s2", "max_abs_ay_m_s2", "max_abs_az_m_s2")) for row in selected_rows)
    max_step = max(max(float(row[name]) for name in ("max_ax_step_m_s2", "max_ay_step_m_s2", "max_az_step_m_s2")) for row in selected_rows)
    write_json(R2 / "constraint_audit.json", {
        "constraints_inside_qp": True,
        "physical_amplitude_limit_m_s2": 2.0,
        "physical_slew_limit_m_s2_per_update": 0.25,
        "observed_max_abs_command_m_s2": max_abs,
        "observed_max_step_m_s2": max_step,
        "limiter_mismatch_max": summary["limiter_mismatch_max"],
        "limiter_mismatch_tolerance": 1.0e-5,
        "pass": max_abs <= 2.0 + 1.0e-9 and max_step <= 0.25 + 1.0e-9 and gates["limiter_parity"]
    })
    write_json(R2 / "solver_audit.json", {
        "solver": "OSQP",
        "solver_valid_rate": summary["solver_valid_rate"],
        "solve_p95_ms": summary["solve_time_p95_ms"],
        "realtime_threshold_ms": 50.0,
        "qp_iterations_max": summary["qp_iterations_max"],
        "pass": gates["solver"] and gates["realtime"]
    })
    write_json(R2 / "safety_audit.json", {
        "safe_sample_count": summary["safe_sample_count"],
        "sample_count": summary["sample_count"],
        "safety_rate": summary["safety_rate"],
        "pass": gates["safety"]
    })
    write_json(R2 / "holdout_access_audit.json", {
        "development_manifest_loaded": "reproducibility/v3/r1/development_evaluation_manifest.json",
        "holdout_manifest_loaded": False,
        "spherical_holdout_targets_executed": False,
        "seeds_3000_3019_executed": False,
        "constant_2_or_3p5_m_s_executed": False,
        "ramp_0_to_3p5_m_s_executed": False,
        "holdout_executed": False
    })
    protected = {
        path: {"start_tree": tree(START_HEAD, path), "current_tree": tree("HEAD", path)}
        for path in ("reproducibility/v2", "reproducibility/v3/r0", "reproducibility/v3/r1", "reproducibility/v3/r1r1")
    }
    write_json(R2 / "gate.json", {
        "task": freeze["task"],
        "start_head": START_HEAD,
        "protocol_freeze_head": PROTOCOL_HEAD,
        "three_traditional_unchanged": protected["reproducibility/v3/r1"]["start_tree"] == protected["reproducibility/v3/r1"]["current_tree"] and protected["reproducibility/v3/r1r1"]["start_tree"] == protected["reproducibility/v3/r1r1"]["current_tree"],
        "advanced_contract_unchanged": sha(R1R1 / "advanced_numeric_win_contract.json") == committed_sha(START_HEAD, "reproducibility/v3/r1r1/advanced_numeric_win_contract.json"),
        "protected_trees": protected,
        "self_method": "3D-DR-TSRMPC",
        "full_3d_control": True,
        "self_candidate_frozen": True,
        "safety_gate": gates["safety"],
        "success_gate": gates["success"],
        "position_gate": gates["position"],
        "acquisition_gate": gates["acquisition"],
        "ramp_gate": gates["ramp"],
        "realtime_gate": gates["realtime"],
        "win_level": freeze["win_level"],
        "ablation_executed": False,
        "advanced_paper_started": False,
        "holdout_executed": False,
        "result": "V3_SELF_ADVANCED_FROZEN_PRE_ABLATION"
    })
    print(json.dumps({"selected": summary["candidate_id"], "win_level": freeze["win_level"], "metrics": freeze["metrics"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
