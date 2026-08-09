from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
V9 = ROOT / "reproducibility/v9"


def read(relative: str) -> dict:
    return json.loads((V9 / relative).read_text(encoding="utf-8"))


def test_final_gate_stops_before_controller_performance() -> None:
    gate = read("final/final_gate.json")
    assert gate["result"] == "BLOCKED_V9_NEURAL_PREDICTOR_NOT_VALIDATED"
    assert gate["predictor"]["competence_pass"] is False
    assert gate["np_mpc_configurations_executed"] == 0
    assert gate["development_executed"] is False
    assert gate["holdout_executed"] is False
    assert gate["PROJECT_RESEARCH_COMPLETE"] is False
    assert gate["automatic_v10_started"] is False


def test_locked_benchmarks_remain_unexecuted() -> None:
    development = read("development/status.json")
    holdout = read("holdout/status.json")
    assert development["status"] == "NOT_RUN_PREDICTOR_INELIGIBLE"
    assert development["authoritative_runs"] == 0
    assert holdout["status"] == "NOT_RUN_PREDICTOR_INELIGIBLE"
    assert holdout["manifest_unchanged"] is True
    assert holdout["sample_count"] == 112
    assert holdout["authoritative_runs"] == 0


def test_completion_resume_claims_are_not_generated() -> None:
    resume = read("final/resume_outputs.json")
    claims = read("final/claim_matrix.json")
    assert resume["status"] == "NOT_GENERATED_PROJECT_INCOMPLETE"
    assert resume["generation_allowed"] is False
    assert claims["predictor_qualified"] is False
    assert claims["paper_controller_created"] is False


def test_final_evidence_manifest_matches_files() -> None:
    manifest = read("final/evidence_manifest.json")
    assert manifest["result"] == "BLOCKED_V9_NEURAL_PREDICTOR_NOT_VALIDATED"
    assert manifest["development_executed"] is False
    assert manifest["holdout_executed"] is False
    for relative, record in manifest["files"].items():
        content = (ROOT / relative).read_bytes()
        try:
            content = content.decode("utf-8").replace("\r\n", "\n").encode("utf-8")
        except UnicodeDecodeError:
            pass
        assert len(content) == record["bytes"]
        assert hashlib.sha256(content).hexdigest() == record["sha256"]
