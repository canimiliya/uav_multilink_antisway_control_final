"""Execute the preregistered V3-R1R1 relative-velocity PID refinement."""

from __future__ import annotations

import argparse
import ast
import csv
import json
from pathlib import Path

import numpy as np

from run_v3_r1_baselines import case_subset, read_json, run_candidates, selection_key, summary_without_rows, write_csv, write_json
from run_v3_r1r1_pid_recovery import enriched, final_key


ROOT = Path(__file__).resolve().parents[1]
R1 = ROOT / "reproducibility/v3/r1"
R1R1 = ROOT / "reproducibility/v3/r1r1"
SEED = 20260810
RANGES = (
    ("uav_kp_x", .3, 1.5), ("uav_kp_y", .15, 1.2), ("uav_kp_z", .8, 5.0),
    ("uav_kd_x", 1.5, 5.0), ("uav_kd_y", 2.0, 7.0), ("uav_kd_z", 2.0, 6.0),
    ("uav_ki_x", .03, .3), ("uav_ki_y", .03, .3), ("uav_ki_z", 0.0, .3),
    ("tip_kp_x", 0.0, .8), ("tip_kp_y", 0.0, .8), ("tip_kp_z", 0.0, .7),
    ("tip_kd_x", 0.0, .6), ("tip_kd_y", 0.0, .8), ("tip_kd_z", 0.0, .4),
    ("limit_x", .03, .16), ("limit_y", .02, .14), ("limit_z", .02, .12),
    ("correction_slew", .01, .08), ("integral_limit", .1, .5),
)


def generate(count: int = 48) -> list[dict]:
    rng = np.random.Generator(np.random.PCG64(SEED))
    dimensions = {}
    for name, low, high in RANGES:
        strata = (np.arange(count) + rng.random(count)) / count
        dimensions[name] = low + (high - low) * strata[rng.permutation(count)]
    rows = []
    for i in range(count):
        value = {name: float(dimensions[name][i]) for name, _, _ in RANGES}
        rows.append({
            "candidate_id": f"cascaded_relative_pid_{i:03d}",
            "uav_kp": [value[f"uav_kp_{axis}"] for axis in "xyz"],
            "uav_kd": [value[f"uav_kd_{axis}"] for axis in "xyz"],
            "uav_ki": [value[f"uav_ki_{axis}"] for axis in "xyz"],
            "tip_kp": [value[f"tip_kp_{axis}"] for axis in "xyz"],
            "tip_kd": [value[f"tip_kd_{axis}"] for axis in "xyz"],
            "correction_limit_m": [value[f"limit_{axis}"] for axis in "xyz"],
            "correction_slew_m_per_update": value["correction_slew"],
            "integral_limit": value["integral_limit"],
            "tip_velocity_mode": "relative_to_uav",
        })
    return rows


def write_union_csv(path: Path, rows: list[dict]) -> None:
    columns = []
    for row in rows:
        for key in row:
            if key not in columns:
                columns.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--workers", type=int, default=8); args = parser.parse_args()
    amendment = read_json(R1R1 / "pid_refinement_protocol.json")
    if amendment["holdout_execution_allowed"] or amendment["advanced_execution_allowed"]:
        raise RuntimeError("refinement permissions invalid")
    manifest = read_json(R1 / "development_evaluation_manifest.json")
    samples = manifest["samples"]
    candidates = generate()
    write_json(R1R1 / "pid_round_c_candidates.json", {"written_before_performance": True, "seed": SEED, "candidate_count": 48, "candidates": candidates})
    core = case_subset(samples, "core")
    round_c = run_candidates("corrected_pid", candidates, core, args.workers, "r1r1-pid-round-c")
    write_csv(R1R1 / "pid_round_c.csv", [summary_without_rows(row) for row in round_c])
    finalists = sorted(round_c, key=selection_key)[:6]
    write_json(R1R1 / "pid_round_d_candidates.json", {"written_before_performance": True, "source": "R1R1-C top 6", "candidate_count": 6, "candidates": [row["parameters"] for row in finalists]})
    round_d = [enriched(row) for row in run_candidates("corrected_pid", [row["parameters"] for row in finalists], samples, args.workers, "r1r1-pid-round-d")]
    selected = sorted(round_d, key=final_key)[0]
    write_csv(R1R1 / "pid_round_d.csv", [summary_without_rows(row) for row in round_d])
    write_csv(R1R1 / "pid_development_results_refinement.csv", [{**sample, "candidate_id": row["candidate_id"]} for row in round_d for sample in row["rows"]])
    with (R1R1 / "pid_candidates.csv").open("r", encoding="utf-8", newline="") as stream:
        history = list(csv.DictReader(stream))
    history.extend({"round": "R1R1-C", **summary_without_rows(row), "calm_axis_acquired_count": None, "competence_pass": None} for row in round_c)
    history.extend({"round": "R1R1-D", **summary_without_rows(row)} for row in round_d)
    write_union_csv(R1R1 / "pid_candidates.csv", history)
    tuning = read_json(R1R1 / "pid_tuning_history.json")
    tuning["rounds"].extend([{"round": "R1R1-C", "candidate_count": 48, "case_count": len(core)}, {"round": "R1R1-D", "candidate_count": 6, "case_count": 75}])
    tuning["total_unique_candidates_evaluated"] = 126
    tuning["authoritative_case_executions"] += 48 * len(core) + 6 * len(samples)
    tuning["selected"] = summary_without_rows(selected)
    write_json(R1R1 / "pid_tuning_history.json", tuning)
    protocol = read_json(R1R1 / "pid_corrective_protocol.json")
    write_json(R1R1 / "pid_freeze.json", {"controller": "corrected_pid", "architecture": "equilibrium-anchored cascaded relative-tip-motion correction plus UAV PID/PD", "implementation": "uav_sway.v3.controllers.V3CascadedTaskPID", "selected": summary_without_rows(selected), "parameters": selected["parameters"], "common_limits": read_json(R1 / "r1_protocol.json")["controller_output"], "competence_gate": protocol["competence_gate"], "holdout_executed": False})
    write_json(R1R1 / "pid_competence_audit.json", {"thresholds": protocol["competence_gate"], "observed": {key: selected[key] for key in ("safety_rate", "success_rate", "calm_axis_acquired_count", "position_rmse_3d_m")}, "pass": selected["competence_pass"], "selected_candidate": selected["candidate_id"], "holdout_executed": False})
    execution = read_json(R1R1 / "pid_execution_audit.json"); execution["attempts"][2]["status"] = "COMPLETE"; execution["attempts"][2]["completed_case_count"] = 600; execution["attempts"].append({"attempt": 4, "workers": args.workers, "status": "COMPLETE", "scope": "Preregistered Rounds C and D", "completed_case_count": 48 * len(core) + 6 * len(samples), "candidate_set_changed": True, "change_authorized_by": "pid_refinement_protocol.json", "holdout_executed": False}); write_json(R1R1 / "pid_execution_audit.json", execution)
    print(json.dumps({"selected": selected["candidate_id"], "competence_pass": selected["competence_pass"], "safety_rate": selected["safety_rate"], "success_rate": selected["success_rate"], "calm_axis_acquired_count": selected["calm_axis_acquired_count"], "position_rmse_3d_m": selected["position_rmse_3d_m"]}, indent=2))
    return 0


if __name__ == "__main__": raise SystemExit(main())
