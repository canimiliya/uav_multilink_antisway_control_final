import csv
import json
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
V6 = ROOT / "reproducibility/v6"


def read(path: str) -> dict:
    return json.loads((V6 / path).read_text(encoding="utf-8"))


def test_every_preregistered_full_bank_candidate_ran_exactly_once():
    for paper in ("yu2026", "sep2026"):
        with (V6 / f"papers/{paper}/stage_b_results.csv").open(encoding="utf-8", newline="") as stream:
            counts = Counter(row["candidate_id"] for row in csv.DictReader(stream))
        assert len(counts) == 24
        assert set(counts.values()) == {120}


def test_zero_qualified_papers_closes_holdout():
    final = read("development/development_final.json")
    holdout = read("holdout_status.json")
    assert final["result"] == "V6_NO_QUALIFIED_RECENT_PAPER_BASELINE"
    assert final["qualified_paper_count"] == 0
    assert final["holdout_execution_allowed"] is False
    assert final["holdout_executed"] is False
    assert holdout["executed"] is False
    assert holdout["reason"] == "ZERO_QUALIFIED_PAPERS"


def test_both_papers_are_ineligible_and_all_gates_are_machine_audited():
    for paper in ("yu2026", "sep2026"):
        audit = read(f"papers/{paper}/development_gate_audit.json")
        status = read(f"papers/{paper}/paper_final_status.json")
        assert audit["candidate_count"] == 24
        assert audit["qualified_ids"] == []
        assert status["development_status"] == "PAPER_DEVELOPMENT_INELIGIBLE"
        assert status["holdout_status"] == "NOT_RUN_DEVELOPMENT_INELIGIBLE"
        assert all(row["gate_count"] == 8 for row in audit["ranked"])


def test_development_accounting_and_frozen_references():
    final = read("development/development_final.json")
    comparator = read("development/comparator_gate.json")
    assert final["authoritative_runs"]["total"] == 6624
    assert set(comparator["participants"]) == {"hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009", "satc_b_027"}
    assert comparator["primary"] == "full_lqr_048"
    assert comparator["satc_role"] == "FROZEN_REFERENCE_NOT_QUALIFICATION_BASELINE"
