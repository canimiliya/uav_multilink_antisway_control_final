"""Freeze the highest-ranked independently confirmed V5 Self candidate."""

from __future__ import annotations

import csv
import hashlib
import json
import subprocess
from collections import defaultdict
from pathlib import Path

try:
    from scripts import evaluate_v5_self as ev
except ImportError:
    import evaluate_v5_self as ev


ROOT = Path(__file__).resolve().parents[1]
SELF = ROOT / "reproducibility/v5/self"
PROTOCOL_HEAD = "a728f4566b3965a6fb73531e9c81b8a63b3ac6ac"


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(name: str, payload: object) -> None:
    path = SELF / name
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def main() -> int:
    if git("branch", "--show-current") != "research-v5": raise RuntimeError("wrong branch")
    if git("rev-parse", "HEAD") != PROTOCOL_HEAD: raise RuntimeError("Self Development must remain on frozen protocol head")
    baseline: dict[str, list[dict]] = defaultdict(list)
    for row in ev.rows(SELF / "baseline_development_results.csv"): baseline[row["candidate_id"]].append(row)
    stage_c: dict[str, list[dict]] = defaultdict(list)
    for row in ev.rows(SELF / "stage_c_results.csv"): stage_c[row["candidate_id"]].append(row)
    audits = sorted((ev.audit(name, rows, baseline) for name, rows in stage_c.items()), key=ev.rank_key)
    stage_b = read(SELF / "stage_b_gate_audit.json")["ranked"]
    stage_b_by_id = {row["candidate_id"]: row for row in stage_b}
    confirmed = [row for row in audits if row["final_pass"] and stage_b_by_id[row["candidate_id"]]["final_pass"]]
    if not confirmed:
        write("near_miss.json", {"status": "V5_SELF_NO_DEVELOPMENT_QUALIFICATION", "ranked": audits, "paper_executed": False, "holdout_executed": False})
        raise SystemExit("No confirmed SATC candidate")
    selected = confirmed[0]
    candidate_id = selected["candidate_id"]
    protocol = read(SELF / "development_protocol.json")
    parameters = next(row for row in protocol["search"]["stage_b"]["candidates"] if row["candidate_id"] == candidate_id)
    write("stage_c_gate_audit.json", {"candidate_count": len(audits), "ranked": audits, "confirmed_pass_ids": [row["candidate_id"] for row in confirmed], "selection_rule": protocol["search"]["stage_b"]["selection_rule"], "selected": candidate_id, "holdout_executed": False})
    write("self_freeze.json", {
        "status": "V5_SELF_DEVELOPMENT_QUALIFIED_FROZEN", "method": "SATC-OFMPC", "candidate_id": candidate_id,
        "protocol_head": PROTOCOL_HEAD, "parameters": parameters, "metrics": selected,
        "stage_b_pass": stage_b_by_id[candidate_id]["final_pass"], "stage_c_confirmation_pass": selected["final_pass"],
        "unique_configurations_used": 48, "maximum_unique_configurations": 64,
        "implementation_sha256": digest(ROOT / "src/uav_sway/v5/satc_ofmpc.py"),
        "parameters_and_implementation_mutable_after_freeze": False, "holdout_executed": False,
    })
    write("development_summary.json", {"selected": candidate_id, "metrics": selected, "stage_b_pass_count": sum(row["final_pass"] for row in stage_b), "stage_c_confirmed_pass_count": len(confirmed), "authoritative_selected_source": "stage_c_results.csv", "holdout_executed": False})
    write("directional_analysis.json", {"candidate_id": candidate_id, "strata": selected["directional"], "all_directional_hard_gates_pass": all(value for data in selected["directional"].values() for value in data["gates"].values()), "coordinate_sign_rule_used": False})
    write("paired_bootstrap.json", selected["bootstrap"])
    selected_rows = stage_c[candidate_id]
    write("safety_audit.json", {"candidate_id": candidate_id, "sample_count": len(selected_rows), "safe_count": sum(row["safe"] for row in selected_rows), "safety_rate": selected["overall"]["safety_rate"], "failure_ids": [row["sample_id"] for row in selected_rows if not row["safe"]]})
    write("constraint_audit.json", {"candidate_id": candidate_id, "acceleration_limit_m_s2": 2.0, "slew_limit_m_s2_per_update": 0.25, "max_observed_acceleration_m_s2": max(float(row["max_abs_command_m_s2"]) for row in selected_rows), "max_observed_step_m_s2": max(float(row["max_command_step_m_s2"]) for row in selected_rows), "within_frozen_authority": all(float(row["max_abs_command_m_s2"]) <= 2.0 + 1e-9 and float(row["max_command_step_m_s2"]) <= .25 + 1e-9 for row in selected_rows)})
    output = SELF / "development_results.csv"
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(selected_rows[0]), lineterminator="\n")
        writer.writeheader(); writer.writerows(selected_rows)
    evidence = ["development_protocol.json", "baseline_development_results.csv", "stage_a_results.csv", "stage_b_results.csv", "stage_c_results.csv", "stage_b_gate_audit.json", "stage_c_gate_audit.json", "self_freeze.json", "development_results.csv", "directional_analysis.json", "paired_bootstrap.json", "safety_audit.json", "constraint_audit.json"]
    write("evidence_sha256.json", {name: digest(SELF / name) for name in evidence})
    print(json.dumps({"selected": candidate_id, "stage_b_passes": sum(row["final_pass"] for row in stage_b), "stage_c_confirmed": len(confirmed), "gates": f"{selected['hard_gate_pass_count']}/{selected['hard_gate_count']}"}, indent=2))
    return 0


if __name__ == "__main__": raise SystemExit(main())
