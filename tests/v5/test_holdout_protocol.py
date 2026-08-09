import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "reproducibility/v5/holdout"


def read(name): return json.loads((OUT / name).read_text(encoding="utf-8"))


def test_unlock_is_final_and_original_contract_is_unchanged():
    value = read("unlock_audit.json")
    assert value["result"] == "V5_HOLDOUT_UNLOCKED"
    assert value["v5_self_finalized_and_qualified"] is True
    assert value["paper_selection_process_finalized"] is True
    assert value["paper_qualified_candidate_exists"] is False
    assert value["original_v5_contract_modified"] is False


def test_one_shot_manifest_is_exact_and_paper_excluded():
    manifest = read("execution_manifest.json"); protocol = read("holdout_protocol_freeze.json")
    assert manifest["sample_count"] == 96
    assert manifest["participant_count"] == 5
    assert manifest["authoritative_run_count"] == 480
    assert all(row["execution_allowed"] for row in manifest["samples"])
    assert manifest["paper"]["status"] == "NOT_RUN_DEVELOPMENT_INELIGIBLE"
    assert protocol["one_shot"] is True
    assert protocol["written_before_first_holdout_trajectory"] is True
    assert protocol["holdout_executed_at_freeze"] is False
