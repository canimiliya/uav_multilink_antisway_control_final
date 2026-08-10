from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
FINAL = ROOT / "reproducibility/v10/final"


def read(name: str) -> dict:
    return json.loads((FINAL / name).read_text(encoding="utf-8"))


def test_failure_closes_route_without_holdout() -> None:
    gate = read("final_gate.json")
    assert gate["result"] == "V10_FXTDO_MPC_NOT_STRONGER_THAN_TRADITIONAL"
    assert gate["RECENT_PAPER_STRONG_BASELINE_VALIDATED"] is False
    assert gate["PAPER_GT_TRADITIONAL"] is False
    assert gate["PROJECT_RESEARCH_COMPLETE"] is False
    assert gate["FINAL_EXTERNAL_PAPER_SEARCH_CLOSED"] is True
    assert gate["NO_V11"] is True
    assert gate["PAPER_FREEZE_HEAD"] is None and gate["V10_HOLDOUT_PROTOCOL_HEAD"] is None


def test_holdout_is_untouched_and_uncompromised() -> None:
    status = read("holdout_status.json")
    assert status["status"] == "NOT_RUN_DEVELOPMENT_INELIGIBLE"
    assert status["executed"] is False and status["authoritative_runs"] == 0
    assert status["retries"] == 0 and status["compromised"] is False
    path = ROOT / status["inherited_manifest"]
    assert hashlib.sha256(path.read_bytes()).hexdigest() == status["manifest_sha256"]


def test_protected_evidence_and_claim_boundary() -> None:
    protected = read("protected_evidence_audit.json")
    assert protected["V1_V9_UNCHANGED"] is True
    assert all(row["unchanged"] for row in protected["protected"].values())
    claims = read("claim_matrix.json")
    assert claims["highest_project_claim"] == "V5_SELF_OVERALL_HOLDOUT_WIN"
    assert "Recent Paper > Traditional" in claims["not_supported"]
    assert not (ROOT / "docs/final_resume").exists()


def test_observer_truth_remained_offline() -> None:
    audit = json.loads((ROOT / "reproducibility/v10/development/observer_offline_truth_audit.json").read_text(encoding="utf-8"))
    assert audit["causal"] is True and audit["used_for_selection"] is False
    assert audit["estimate_finite"] is True and audit["no_sustained_divergence"] is True
    assert audit["fixed_time_claim_validated"] is False
