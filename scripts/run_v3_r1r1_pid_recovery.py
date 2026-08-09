"""Development-only tuning for the V3-R1R1 classical cascaded PID."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from run_v3_r1_baselines import aggregate, case_subset, read_json, run_candidates, selection_key, summary_without_rows, write_csv, write_json


ROOT = Path(__file__).resolve().parents[1]
R1 = ROOT / "reproducibility/v3/r1"
R1R1 = ROOT / "reproducibility/v3/r1r1"
SEED = 20260809


RANGES = (
    ("uav_kp_xy", 0.4, 4.0), ("uav_kp_z", 0.8, 8.0),
    ("uav_kd_xy", 0.8, 5.0), ("uav_kd_z", 1.0, 6.0),
    ("uav_ki_xy", 0.0, 0.3), ("uav_ki_z", 0.0, 0.4),
    ("tip_kp_xy", 0.0, 1.5), ("tip_kp_z", 0.0, 1.0),
    ("tip_kd_xy", 0.0, 0.8), ("tip_kd_z", 0.0, 0.6),
    ("correction_limit_xy", 0.03, 0.30), ("correction_limit_z", 0.02, 0.20),
    ("correction_slew", 0.01, 0.10), ("integral_limit", 0.05, 0.50),
)


def candidates(count: int = 64) -> list[dict]:
    rng = np.random.Generator(np.random.PCG64(SEED))
    dimensions: dict[str, np.ndarray] = {}
    for name, low, high in RANGES:
        strata = (np.arange(count, dtype=float) + rng.random(count)) / count
        dimensions[name] = low + (high - low) * strata[rng.permutation(count)]
    result = []
    for index in range(count):
        value = {name: float(dimensions[name][index]) for name, _, _ in RANGES}
        result.append({
            "candidate_id": f"cascaded_pid_{index:03d}",
            "uav_kp": [value["uav_kp_xy"], value["uav_kp_xy"], value["uav_kp_z"]],
            "uav_kd": [value["uav_kd_xy"], value["uav_kd_xy"], value["uav_kd_z"]],
            "uav_ki": [value["uav_ki_xy"], value["uav_ki_xy"], value["uav_ki_z"]],
            "tip_kp": [value["tip_kp_xy"], value["tip_kp_xy"], value["tip_kp_z"]],
            "tip_kd": [value["tip_kd_xy"], value["tip_kd_xy"], value["tip_kd_z"]],
            "correction_limit_m": [value["correction_limit_xy"], value["correction_limit_xy"], value["correction_limit_z"]],
            "correction_slew_m_per_update": value["correction_slew"],
            "integral_limit": value["integral_limit"],
        })
    return result


def enriched(summary: dict) -> dict:
    calm_axis = [row for row in summary["rows"] if row["sample_id"].startswith("calm_axis_")]
    value = dict(summary)
    value["calm_axis_acquired_count"] = sum(bool(row["task_success"]) for row in calm_axis)
    value["competence_pass"] = bool(
        value["safety_rate"] == 1.0
        and value["calm_axis_acquired_count"] >= 5
        and value["success_rate"] >= 0.5
        and value["position_rmse_3d_m"] <= 0.25
    )
    return value


def final_key(row: dict) -> tuple:
    return (not row["competence_pass"], -row["safe_sample_count"], -row["calm_axis_acquired_count"], -row["task_success_count"], row["position_rmse_3d_m"], float("inf") if row["acquisition_median_s"] is None else row["acquisition_median_s"], row["orientation_rmse_deg"], row["total_acceleration_effort"], row["candidate_id"])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=20)
    args = parser.parse_args()
    protocol = read_json(R1R1 / "pid_corrective_protocol.json")
    if protocol["holdout_execution_allowed"] is not False:
        raise RuntimeError("R1R1 protocol must forbid holdout execution")
    manifest = read_json(R1 / "development_evaluation_manifest.json")
    samples = manifest["samples"]
    if len(samples) != 75 or manifest["development_seeds"] != list(range(2000, 2020)):
        raise RuntimeError("frozen Development manifest mismatch")
    generated = candidates()
    write_json(R1R1 / "pid_round_a_candidates.json", {"written_before_performance": True, "seed": SEED, "candidate_count": len(generated), "candidates": generated})
    core = case_subset(samples, "core")
    round_a_path = R1R1 / "pid_round_a.csv"
    round_b_candidates_path = R1R1 / "pid_round_b_candidates.json"
    if round_a_path.exists() and round_b_candidates_path.exists():
        with round_a_path.open("r", encoding="utf-8", newline="") as stream:
            round_a_history = list(csv.DictReader(stream))
        top_parameters = read_json(round_b_candidates_path)["candidates"]
        if len(round_a_history) != 64 or len(top_parameters) != 8:
            raise RuntimeError("incomplete resume evidence")
    else:
        round_a = run_candidates("corrected_pid", generated, core, args.workers, "r1r1-pid-round-a")
        round_a_history = [summary_without_rows(row) for row in round_a]
        write_csv(round_a_path, round_a_history)
        top = sorted(round_a, key=selection_key)[:8]
        top_parameters = [row["parameters"] for row in top]
        write_json(round_b_candidates_path, {"written_before_performance": True, "source": "R1R1-A top 8 under frozen selection rule", "candidate_count": 8, "candidates": top_parameters})
    round_b_raw = run_candidates("corrected_pid", top_parameters, samples, args.workers, "r1r1-pid-round-b")
    round_b = [enriched(row) for row in round_b_raw]
    selected = sorted(round_b, key=final_key)[0]
    history_rows = []
    for row in round_a_history:
        history_rows.append({"round": "R1R1-A", **row, "calm_axis_acquired_count": None, "competence_pass": None})
    for row in round_b:
        history_rows.append({"round": "R1R1-B", **summary_without_rows(row)})
    write_csv(R1R1 / "pid_candidates.csv", history_rows)
    write_csv(R1R1 / "pid_development_results.csv", [{**sample, "candidate_id": row["candidate_id"]} for row in round_b for sample in row["rows"]])
    write_json(R1R1 / "pid_tuning_history.json", {"rounds": [{"round": "R1R1-A", "candidate_count": 64, "case_count": len(core)}, {"round": "R1R1-B", "candidate_count": 8, "case_count": 75}], "total_unique_candidates_evaluated": 72, "authoritative_case_executions": 64 * len(core) + len(round_b) * len(samples), "all_attempts_retained": True, "execution_audit": "pid_execution_audit.json", "selected": summary_without_rows(selected)})
    freeze = {"controller": "corrected_pid", "architecture": "equilibrium-anchored cascaded tip-reference correction plus UAV PID/PD", "implementation": "uav_sway.v3.controllers.V3CascadedTaskPID", "selected": summary_without_rows(selected), "parameters": selected["parameters"], "common_limits": read_json(R1 / "r1_protocol.json")["controller_output"], "competence_gate": protocol["competence_gate"], "holdout_executed": False}
    write_json(R1R1 / "pid_freeze.json", freeze)
    write_json(R1R1 / "pid_competence_audit.json", {"thresholds": protocol["competence_gate"], "observed": {key: selected[key] for key in ("safety_rate", "success_rate", "calm_axis_acquired_count", "position_rmse_3d_m")}, "pass": selected["competence_pass"], "selected_candidate": selected["candidate_id"], "holdout_executed": False})
    print(json.dumps({"selected": selected["candidate_id"], "competence_pass": selected["competence_pass"], "safety_rate": selected["safety_rate"], "success_rate": selected["success_rate"], "calm_axis_acquired_count": selected["calm_axis_acquired_count"], "position_rmse_3d_m": selected["position_rmse_3d_m"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
