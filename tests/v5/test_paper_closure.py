import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PAPER = ROOT / "reproducibility/v5/paper"


def read(name): return json.loads((PAPER / name).read_text(encoding="utf-8"))


def test_paper_route_is_closed_without_replacement_or_holdout():
    status = read("paper_final_status.json"); near = read("near_miss.json"); audit = read("development_gate_audit.json")
    assert status["development_status"] == "PAPER_DEVELOPMENT_INELIGIBLE"
    assert status["holdout_status"] == "NOT_RUN_DEVELOPMENT_INELIGIBLE"
    assert status["claims_about_original_authors_on_v5_benchmark"] is False
    assert near["no_replacement_paper"] is True
    assert not audit["qualified_ids"]
    assert audit["candidate_count"] == 24
    assert max(row["gate_pass_count"] for row in audit["ranked"]) < 5
