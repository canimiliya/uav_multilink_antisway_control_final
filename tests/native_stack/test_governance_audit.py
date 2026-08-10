import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
GOV = ROOT / "reproducibility/native_stack/governance"


def load(name):
    return json.loads((GOV / name).read_text(encoding="utf-8"))


def test_70_percent_origin_is_not_overclaimed():
    audit = load("competence_origin_audit.json")
    assert audit["SUCCESS_70_GATE_PHYSICALLY_DERIVED"] is False
    assert audit["classification"] == {
        "ENGINEERING_REQUIREMENT": False,
        "HEURISTIC_RESEARCH_GATE": True,
        "PHYSICALLY_DERIVED_THRESHOLD": False,
    }


def test_cohort_partition_is_identity_preserving_and_complete():
    audit = load("cohort_audit.json")
    assert audit["case_identity_changed"] is False
    assert audit["balanced_100_100"] is True
    assert set(audit["nominal"]["wind_kinds"]) == {"calm", "moderate", "stochastic"}
    assert set(audit["challenge"]["wind_kinds"]) == {"strong_sustained", "strong_transient", "ramp"}


def test_holdout_is_hash_only_and_unexecuted():
    status = load("holdout_status.json")
    assert status["fingerprint_count"] == status["unique_fingerprint_count"] == 140
    assert status["performance_fields_read"] is False
    assert status["executed"] is False
    assert status["authoritative_runs"] == 0
    assert status["compromised"] is False


def test_oracle_has_no_selection_authority():
    protocol = load("oracle_protocol.json")
    assert protocol["split"] == "Development only"
    assert protocol["search_or_tuning_allowed"] is False
    assert protocol["selection_authority"] == "NONE"
    assert protocol["holdout_allowed"] is False
