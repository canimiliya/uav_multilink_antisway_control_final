from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
FINAL = ROOT / "reproducibility/native_stack/r1r3/final"


def test_failed_gate_stops_before_satc_paper_and_holdout() -> None:
    gate = json.loads((FINAL / "final_gate.json").read_text(encoding="utf-8"))
    holdout = json.loads((FINAL / "holdout_status.json").read_text(encoding="utf-8"))
    assert gate["result"] == "P2_GOVERNANCE_V2_TRADITIONAL_QUALIFICATION_FAILED"
    assert gate["competent_traditional_count"] == 0
    assert gate["primary_traditional_frozen"] is False
    assert gate["traditional_envelope_frozen"] is False
    assert gate["satc"]["search_executed"] is False
    assert gate["paper_search"] is False
    assert holdout["executed"] is False
    assert holdout["authoritative_runs"] == 0
    assert holdout["performance_fields_read"] is False


def test_platform_and_governance_remain_unchanged() -> None:
    gate = json.loads((FINAL / "final_gate.json").read_text(encoding="utf-8"))
    assert gate["source_head"] == "88c3aef081fabab44d174bc8afd234ae634af3da"
    assert gate["governance_changed"] is False
    assert gate["benchmark_changed"] is False
    assert gate["benchmark_physics_unchanged"] is True
    assert gate["v1_v10_unchanged"] is True
    assert gate["project_progress_percent"] == 84

