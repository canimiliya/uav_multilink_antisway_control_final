import csv
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R3 = ROOT / "reproducibility" / "v3" / "r3"


def _read(name: str) -> dict:
    return json.loads((R3 / name).read_text(encoding="utf-8"))


def test_paper_selection_precedes_performance_and_is_not_exact() -> None:
    selection = _read("paper_selection.json")
    assert selection["selected_before_paper_performance"] is True
    assert selection["performance_considered"] is False
    assert selection["adaptation_name"] == "LV2026-SPACC-ADAPTED-3D"
    assert selection["exact_reproduction"] is False


def test_search_budget_and_holdout_barrier_are_frozen() -> None:
    history = _read("paper_search_history.json")
    contract = _read("paper_adaptation_contract.json")
    assert history["written_before_any_paper_performance"] is True
    assert history["total_unique_candidate_budget"] == 48
    assert history["holdout_executed"] is False
    assert contract["holdout_execution_allowed"] is False
    assert contract["self_module_used"] is False


def test_candidate_matrix_contains_required_comparison() -> None:
    with (R3 / "paper_candidate_matrix.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) >= 5
    assert any("10.1109/TITS.2025.3594288" in row["primary_source"] for row in rows)
    assert sum(row["decision"] == "SELECT" for row in rows) == 1
