import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SELF = ROOT / "reproducibility/v5/self"


def read(name): return json.loads((SELF / name).read_text(encoding="utf-8"))


def test_ablation_is_diagnostic_and_complete():
    value = read("ablation.json")
    assert value["status"] == "POST_FREEZE_DIAGNOSTIC_COMPLETE"
    assert value["selection_authority"] is False
    assert value["return_to_tuning"] is False
    assert len(value["levels"]) == 6
    assert value["protocol_source_label_issue"]["resolution"].startswith("ran the already-preregistered")
    assert value["technical_retries"][0]["completed_on_retry"] is True


def test_mechanism_traces_cover_all_preregistered_strata():
    value = read("mechanism_report.json")
    assert set(value["traces"]) == {"normal", "strong_aligned", "strong_opposed", "strong_cross"}
    assert all(row["updates"] == 241 for row in value["traces"].values())
    assert value["holdout_executed"] is False
