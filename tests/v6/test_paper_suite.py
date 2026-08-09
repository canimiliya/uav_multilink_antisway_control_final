import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PAPERS = ROOT / "reproducibility/v6/papers"


def read(name: str) -> dict:
    return json.loads((PAPERS / name).read_text(encoding="utf-8"))


def test_suite_is_complete_and_frozen_before_performance():
    freeze = read("suite_freeze.json")
    assert freeze["suite_size"] == 2
    assert freeze["replacement_after_freeze_allowed"] is False
    assert freeze["paper_performance_executed"] is False
    assert freeze["holdout_accessed"] is False
    assert "YU2026-FT-CFO-ADAPTED-5LINK" in freeze["methods"]
    assert "SEP2026-PASSIVITY-NMPC-ADAPTED-5LINK" in freeze["methods"]


def test_priority_b_is_not_guessed_and_fallback_is_primary_complete():
    rows = {row["paper_id"]: row for row in read("literature_review.json")["candidates"]}
    assert rows["KANG2026"]["selected"] is False
    assert rows["KANG2026"]["full_equations_available"] is False
    assert rows["SEP2026"]["selected"] is True
    assert rows["SEP2026"]["full_equations_available"] is True
    assert rows["YU2026"]["supplementary_audit"].endswith("executable simulation source code.")


def test_adaptations_preserve_core_and_common_authority():
    for name in ("yu2026_adaptation_contract.json", "sep2026_adaptation_contract.json"):
        contract = read(name)
        assert contract["core_innovation_retained"] is True
        assert contract["authority"]["output"] == "world acceleration"
        assert contract["authority"]["absolute_limit"] == 2.0
        assert contract["authority"]["slew_per_update"] == 0.25


def test_search_budgets_and_smoke_ids_are_fixed():
    protocol = read("development_protocol.json")
    assert len(protocol["stage_a_sample_ids"]) == len(set(protocol["stage_a_sample_ids"])) == 24
    for paper in protocol["papers"].values():
        assert len(paper["stage_a"]) == 8
        assert len(paper["stage_b"]) == 24
        assert paper["unique_configuration_count"] == 32
        assert paper["maximum"] == 48
    assert protocol["holdout_manifest_loaded"] is False
