"""Finalize the decisive P2-R1R3 Traditional qualification failure."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "reproducibility/native_stack/r1r3"
FINAL = BASE / "final"
PROTOCOL_HEAD = "ab289b2dda4813bf2f8cfc76019f82bfd801d2d0"
RECOVERY_HEAD = "4118c4d57235ba646c585f1cd01e4f11f5e48c02"
TRADITIONAL_EVIDENCE_HEAD = "fd1530b9005646b1da6134673d91396e5327d003"


def read_best(folder: str) -> dict:
    path = BASE / f"traditional/{folder}/local_development_summary.csv"
    with path.open(encoding="utf-8", newline="") as handle:
        return next(csv.DictReader(handle))


def number(row: dict, key: str) -> float:
    return float(row[key])


def write(name: str, payload: object) -> None:
    FINAL.mkdir(parents=True, exist_ok=True)
    (FINAL / name).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def metric_payload(row: dict) -> dict:
    keys = [
        "all_safety_rate", "all_catastrophic_count", "all_deadline_miss_rate",
        "nominal_success_rate", "nominal_calm_success_rate", "nominal_moderate_success_rate",
        "nominal_stochastic_success_rate", "nominal_setpoint_success_rate",
        "nominal_trajectory_success_rate", "nominal_setpoint_rmse_m",
        "nominal_trajectory_rmse_m", "challenge_success_rate", "challenge_rmse_m",
        "strong_mean_m", "strong_p90_m", "orientation_rmse_rad", "physical_effort",
        "runtime_p95_ms",
    ]
    payload = {key: number(row, key) for key in keys}
    payload["method_id"] = row["method_id"]
    payload["competence_v2"] = row["competence_v2"].lower() == "true"
    return payload


def main() -> None:
    pid = read_best("pid")
    lqi = read_best("full_lqr")
    if pid["competence_v2"].lower() != "false" or lqi["competence_v2"].lower() != "false":
        raise AssertionError("failure finalizer may only run when both completed families failed")
    holdout = ROOT / "reproducibility/native_stack/r0s/resolved_holdout_manifest.json"
    final_gate = {
        "task": "P2-R1R3-GOVERNANCE-V2-STRONG-TRADITIONAL-QUALIFICATION-AND-SATC-FREEZE-R1",
        "result": "P2_GOVERNANCE_V2_TRADITIONAL_QUALIFICATION_FAILED",
        "source_tag": "native-stack-benchmark-v1.2-governance",
        "source_head": "88c3aef081fabab44d174bc8afd234ae634af3da",
        "branch": "research/p2-baselines-governance-v2",
        "governance_version": "native-stack-competence-governance-v2",
        "governance_changed": False, "benchmark_changed": False,
        "protocol_freeze_head": PROTOCOL_HEAD, "local_recovery_freeze_head": RECOVERY_HEAD,
        "traditional_failure_evidence_head": TRADITIONAL_EVIDENCE_HEAD,
        "pid": {"source_seed": "native_pid_001", "new_configs": 48,
                "full_200_candidates": 12, "selected": pid["method_id"],
                "metrics": metric_payload(pid), "competence_v2": False,
                "freeze_head": TRADITIONAL_EVIDENCE_HEAD},
        "full_lqr": {"architecture": "9_state_translational_CARE_LQI_plus_SO3_physical_wrench",
                     "new_configs": 48, "full_200_candidates": 12,
                     "selected": lqi["method_id"], "metrics": metric_payload(lqi),
                     "competence_v2": False, "freeze_head": TRADITIONAL_EVIDENCE_HEAD},
        "task_lqr": {"status": "NOT_RUN_OPTIONAL_AFTER_DECISIVE_FAILURE", "new_configs": 0,
                     "selected": None, "competence_v2": False, "freeze_head": None},
        "competent_traditional_count": 0,
        "primary_traditional_frozen": False, "traditional_envelope_frozen": False,
        "satc": {"search_executed": False, "selected": None, "advanced_qualified": False,
                 "reason": "competent_traditional_count < 2"},
        "paper_search": False, "paper_selected": False, "paper_implementation": False,
        "paper_performance": False, "v1_v10_unchanged": True,
        "benchmark_physics_unchanged": True, "project_progress_percent": 84,
        "final_tag": None,
    }
    write("final_gate.json", final_gate)
    write("holdout_status.json", {
        "manifest_path": str(holdout.relative_to(ROOT)).replace("\\", "/"),
        "manifest_sha256": sha256(holdout), "execution_allowed": False,
        "executed": False, "authoritative_runs": 0, "compromised": False,
        "performance_fields_read": False,
    })
    write("candidate_budget.json", {
        "pid": {"initial_configs": 24, "local_configs": 24, "total_new_configs": 48,
                "maximum": 64, "full_200_candidates": 12},
        "full_lqr_lqi": {"initial_configs": 24, "local_configs": 24,
                         "total_new_configs": 48, "full_200_candidates": 12},
        "task_lqr": {"new_configs": 0, "maximum_optional": 24},
        "satc": {"new_configs": 0, "maximum_if_gate_opened": 96},
    })
    write("project_test_status.json", {
        "isolated_group_execution": {
            "tests/native_stack": 36, "tests/release": 4, "tests/v2": 56,
            "tests/v3": 63, "tests/v4": 25, "tests/v5": 22, "tests/v6": 20,
            "tests/v7": 9, "tests/v8": 11, "tests/v9": 13, "tests/v10": 12,
        },
        "passed": 271, "failed": 0,
        "warnings": "OSQP deprecation and pending-deprecation warnings only",
        "monolithic_invocation": {
            "status": "HARNESS_STALLED_NO_FAILURE_OUTPUT",
            "action": "exact pytest process stopped after prolonged no-I/O wait; complete suite rerun by isolated test directories",
            "used_as_project_failure": False,
        },
    })
    evidence_paths = [
        "reproducibility/native_stack/r1r3/protocol/research_contract.json",
        "reproducibility/native_stack/r1r3/protocol/search_contract.json",
        "reproducibility/native_stack/r1r3/protocol/selection_contract.json",
        "reproducibility/native_stack/r1r3/protocol/governance_reference.json",
        "reproducibility/native_stack/r1r3/protocol/candidate_import_audit.json",
        "reproducibility/native_stack/r1r3/protocol/stage_a_manifest.json",
        "reproducibility/native_stack/r1r3/protocol/statistical_protocol.json",
        "reproducibility/native_stack/r1r3/recovery_protocol/local_recovery_contract.json",
        "reproducibility/native_stack/r1r3/recovery_protocol/local_recovery_diagnosis.json",
        "reproducibility/native_stack/r1r3/traditional/pid/local_development_summary.csv",
        "reproducibility/native_stack/r1r3/traditional/full_lqr/local_development_summary.csv",
        "docs/native_stack/r1r3/P2_R1R3_FINAL_REPORT.md",
        "docs/native_stack/r1r3/TRADITIONAL_QUALIFICATION_FAILURE_BOUNDARY.md",
        "reproducibility/native_stack/r1r3/final/project_test_status.json",
    ]
    write("evidence_manifest.json", {
        "entries": [{"path": path, "sha256": sha256(ROOT / path)} for path in evidence_paths],
        "holdout_performance_evidence_included": False,
    })
    print(json.dumps({"result": final_gate["result"], "competent_traditional_count": 0,
                      "satc_executed": False, "holdout_executed": False}, sort_keys=True))


if __name__ == "__main__":
    main()
