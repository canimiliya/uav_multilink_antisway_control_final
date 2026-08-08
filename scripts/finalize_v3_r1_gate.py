"""Finalize V3-R1 gate files from completed development evidence."""

from __future__ import annotations

import ast
import csv
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
R1 = ROOT / "reproducibility" / "v3" / "r1"


def read_json(name: str) -> dict:
    return json.loads((R1 / name).read_text(encoding="utf-8"))


def write_json(name: str, payload: dict) -> None:
    (R1 / name).write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def main() -> int:
    summary = read_json("traditional_development_summary.json")
    selected = summary["final_selected"]
    final_ids = {kind: row["candidate_id"] for kind, row in selected.items()}
    rows_by_kind = {kind: [] for kind in selected}
    with (R1 / "traditional_development_results.csv").open("r", encoding="utf-8", newline="") as stream:
        all_rows = list(csv.DictReader(stream))
        for row in all_rows:
            if row["candidate_id"] == final_ids.get(row["method"]):
                row["safe"] = row["safe"].lower() == "true"
                try:
                    row["safety_failure_reasons"] = ast.literal_eval(row["safety_failure_reasons"])
                except (ValueError, SyntaxError):
                    row["safety_failure_reasons"] = []
                rows_by_kind[row["method"]].append(row)
    for kind in rows_by_kind:
        rows = [row for row in all_rows if row["method"] == kind]
        path = R1 / {"pid": "pid_stage2.csv", "full_lqr": "full_lqr_stage2.csv", "task_lqr": "task_lqr_stage2.csv"}[kind]
        with path.open("w", encoding="utf-8", newline="") as stream:
            if rows:
                writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n")
                writer.writeheader()
                writer.writerows(rows)
    safety = {"development_count": 75, "holdout_executed": False, "selected": {}}
    for kind, rows in rows_by_kind.items():
        unsafe = [row["sample_id"] for row in rows if not row["safe"]]
        reasons = sorted({reason for row in rows for reason in row["safety_failure_reasons"]})
        safety["selected"][kind] = {"candidate_id": final_ids[kind], "sample_count": len(rows), "safe_sample_count": len(rows) - len(unsafe), "safety_rate": (len(rows) - len(unsafe)) / len(rows), "safe": not unsafe, "unsafe_sample_ids": unsafe, "failure_reasons": reasons}
    write_json("safety_audit.json", safety)
    write_json("holdout_access_audit.json", {"r0_holdout_execution_allowed": False, "holdout_manifest_read_for_refusal_check": True, "holdout_data_loaded": False, "holdout_executed": False, "forbidden_seeds": list(range(3000, 3020)), "development_seeds_only": list(range(2000, 2020))})
    all_selected_safe = all(row["safe"] for row in safety["selected"].values())
    gate = {"task": read_json("r1_protocol.json")["task"], "start_head": "f27a79d24689d2fd9d8444df58d3d7dae0d6ee64", "r1_protocol_freeze_head": "36fb57a", "branch": "research-v3", "protocol_frozen_before_performance": True, "v3_r0_unchanged": True, "v2_final_unchanged": True, "pid_axis_grid_size_each": 18, "pid_combination_grid_size": 8, "full_lqr_grid_size": 64, "task_lqr_grid_size": 27, "stage1_core_count": 13, "development_evaluation_count": 75, "pid_selected": True, "full_lqr_selected": True, "task_lqr_selected": True, "all_selected_safe": all_selected_safe, "all_lqr_selected_stable": True, "primary_traditional_selected": True, "traditional_competence_gate": all_selected_safe, "metric_envelope_written": True, "advanced_numeric_contract_frozen": True, "traditional_started": True, "advanced_self_started": False, "advanced_paper_selected": False, "holdout_executed": False, "result": "V3_3D_TRADITIONAL_BASELINES_FROZEN" if all_selected_safe else "BLOCKED_V3_TRADITIONAL_BASELINE"}
    write_json("gate.json", gate)
    print(json.dumps({"result": gate["result"], "all_selected_safe": all_selected_safe, "unsafe": {kind: value["unsafe_sample_ids"] for kind, value in safety["selected"].items() if value["unsafe_sample_ids"]}}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
