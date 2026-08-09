import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PROTOCOL = ROOT / "reproducibility/v5/self/development_protocol.json"


def test_self_protocol_is_frozen_before_performance():
    value = json.loads(PROTOCOL.read_text(encoding="utf-8"))
    search = value["search"]
    ids = [row["candidate_id"] for stage in ("stage_a", "stage_b") for row in search[stage]["candidates"]]
    assert value["written_before_first_satc_performance"] is True
    assert value["satc_performance_executed_at_freeze"] is False
    assert len(ids) == len(set(ids)) == 48
    assert search["maximum_unique_configurations"] == 64
    assert search["stage_a"]["selection_authority"] is False
    assert search["stage_b"]["candidate_count"] == 36
    assert value["holdout_executed"] is False


def test_protocol_forbids_noncausal_shortcuts():
    value = json.loads(PROTOCOL.read_text(encoding="utf-8"))
    forbidden = set(value["architecture"]["forbidden"])
    assert {"wind truth", "future wind", "target-x sign branch", "V5 Holdout"} <= forbidden
