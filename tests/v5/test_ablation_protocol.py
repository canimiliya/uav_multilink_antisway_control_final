import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_ablation_cannot_change_frozen_self_or_select():
    value = json.loads((ROOT / "reproducibility/v5/self/ablation_protocol.json").read_text(encoding="utf-8"))
    assert value["self_candidate_unchanged"] == "satc_b_027"
    assert value["selection_authority"] is False
    assert value["return_to_tuning_forbidden"] is True
    assert len(value["cumulative_levels"]) == 6
    assert len(value["representative_traces"]) == 4
    assert value["holdout_executed"] is False
