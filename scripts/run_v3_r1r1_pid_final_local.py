"""Prepare and execute the final finite V3-R1R1 PID local composition."""

from __future__ import annotations

import argparse
import ast
import csv
import json
from itertools import product
from pathlib import Path

from run_v3_r1_baselines import read_json, run_candidates, summary_without_rows, write_csv, write_json
from run_v3_r1r1_pid_recovery import enriched, final_key
from run_v3_r1r1_pid_refinement import write_union_csv


ROOT = Path(__file__).resolve().parents[1]
R1 = ROOT / "reproducibility/v3/r1"
R1R1 = ROOT / "reproducibility/v3/r1r1"
DONOR_IDS = ("cascaded_relative_pid_007", "cascaded_relative_pid_041", "cascaded_relative_pid_035")


def donors() -> dict[str, dict]:
    with (R1R1 / "pid_round_d.csv").open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    result = {row["candidate_id"]: ast.literal_eval(row["parameters"]) for row in rows if row["candidate_id"] in DONOR_IDS}
    if set(result) != set(DONOR_IDS):
        raise RuntimeError("final local donor evidence incomplete")
    return result


def prepare() -> list[dict]:
    source = donors()
    short = {key.rsplit("_", 1)[-1]: value for key, value in source.items()}
    rows = []
    for x_name, y_name, z_name in product(("007", "041", "035"), ("041", "035"), ("007", "041", "035")):
        selected = (short[x_name], short[y_name], short[z_name])
        parameter = {"candidate_id": f"hybrid_x{x_name}_y{y_name}_z{z_name}"}
        for field in ("uav_kp", "uav_kd", "uav_ki", "tip_kp", "tip_kd", "correction_limit_m"):
            parameter[field] = [float(selected[axis][field][axis]) for axis in range(3)]
        parameter["correction_slew_m_per_update"] = sum(float(value["correction_slew_m_per_update"]) for value in selected) / 3.0
        parameter["integral_limit"] = sum(float(value["integral_limit"]) for value in selected) / 3.0
        parameter["tip_velocity_mode"] = "relative_to_uav"
        parameter["axis_donors"] = {"x": x_name, "y": y_name, "z": z_name}
        rows.append(parameter)
    write_json(R1R1 / "pid_round_e_candidates.json", {"written_before_performance": True, "candidate_count": len(rows), "construction": "exhaustive preregistered axis-donor product", "candidates": rows})
    return rows


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--workers", type=int, default=8); parser.add_argument("--prepare-only", action="store_true"); args = parser.parse_args()
    protocol = read_json(R1R1 / "pid_final_local_protocol.json")
    if protocol["holdout_execution_allowed"] or protocol["advanced_execution_allowed"]:
        raise RuntimeError("final local permissions invalid")
    candidates_path = R1R1 / "pid_round_e_candidates.json"
    candidates = prepare() if not candidates_path.exists() else read_json(candidates_path)["candidates"]
    if args.prepare_only:
        print(json.dumps({"prepared": len(candidates)}, indent=2)); return 0
    samples = read_json(R1 / "development_evaluation_manifest.json")["samples"]
    round_e = [enriched(row) for row in run_candidates("corrected_pid", candidates, samples, args.workers, "r1r1-pid-round-e")]
    selected = sorted(round_e, key=final_key)[0]
    write_csv(R1R1 / "pid_round_e.csv", [summary_without_rows(row) for row in round_e])
    write_csv(R1R1 / "pid_development_results_final.csv", [{**sample, "candidate_id": row["candidate_id"]} for row in round_e for sample in row["rows"]])
    with (R1R1 / "pid_candidates.csv").open("r", encoding="utf-8", newline="") as stream:
        history = list(csv.DictReader(stream))
    history.extend({"round": "R1R1-E", **summary_without_rows(row)} for row in round_e)
    write_union_csv(R1R1 / "pid_candidates.csv", history)
    tuning = read_json(R1R1 / "pid_tuning_history.json")
    tuning["rounds"].append({"round": "R1R1-E", "candidate_count": 18, "case_count": 75})
    tuning["total_unique_candidates_evaluated"] = 144
    tuning["authoritative_case_executions"] += 18 * 75
    tuning["selected"] = summary_without_rows(selected)
    write_json(R1R1 / "pid_tuning_history.json", tuning)
    corrective = read_json(R1R1 / "pid_corrective_protocol.json")
    write_json(R1R1 / "pid_freeze.json", {"controller": "corrected_pid", "architecture": "equilibrium-anchored cascaded relative-tip-motion correction plus UAV PID/PD", "implementation": "uav_sway.v3.controllers.V3CascadedTaskPID", "selected": summary_without_rows(selected), "parameters": selected["parameters"], "common_limits": read_json(R1 / "r1_protocol.json")["controller_output"], "competence_gate": corrective["competence_gate"], "holdout_executed": False})
    write_json(R1R1 / "pid_competence_audit.json", {"thresholds": corrective["competence_gate"], "observed": {key: selected[key] for key in ("safety_rate", "success_rate", "calm_axis_acquired_count", "position_rmse_3d_m")}, "pass": selected["competence_pass"], "selected_candidate": selected["candidate_id"], "holdout_executed": False})
    execution = read_json(R1R1 / "pid_execution_audit.json"); execution["attempts"].append({"attempt": 5, "workers": args.workers, "status": "COMPLETE", "scope": "Preregistered final Round E", "completed_case_count": 18 * 75, "candidate_set_changed": True, "change_authorized_by": "pid_final_local_protocol.json", "holdout_executed": False}); write_json(R1R1 / "pid_execution_audit.json", execution)
    print(json.dumps({"selected": selected["candidate_id"], "competence_pass": selected["competence_pass"], "safety_rate": selected["safety_rate"], "success_rate": selected["success_rate"], "calm_axis_acquired_count": selected["calm_axis_acquired_count"], "position_rmse_3d_m": selected["position_rmse_3d_m"]}, indent=2))
    return 0


if __name__ == "__main__": raise SystemExit(main())
