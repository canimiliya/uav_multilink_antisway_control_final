"""V3-R2 frozen Self candidate and evidence gates."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R2 = ROOT / "reproducibility/v3/r2"


def read(name: str) -> dict:
    return json.loads((R2 / name).read_text(encoding="utf-8"))


def test_selected_self_passes_every_frozen_development_gate() -> None:
    freeze = read("self_freeze.json")
    metrics = freeze["metrics"]
    assert freeze["method"] == "3D-DR-TSRMPC"
    assert freeze["candidate_id"] == "self_a_034"
    assert freeze["backbone"] == "task_lqr_009"
    assert metrics["safe_sample_count"] == metrics["sample_count"] == 75
    assert metrics["task_success_count"] == 75
    assert freeze["win_level"] == "STRICT_ALL_METRIC_WIN_DEVELOPMENT"
    assert all(freeze["gates"].values())


def test_pairing_and_solver_constraint_evidence_are_complete() -> None:
    paired = read("paired_primary_comparison.json")
    solver = read("solver_audit.json")
    constraint = read("constraint_audit.json")
    assert paired["pair_count"] == len(paired["pairs"]) == 75
    assert len({row["sample_id"] for row in paired["pairs"]}) == 75
    assert paired["formal_10000_bootstrap_executed"] is False
    assert solver["pass"] is True and solver["solve_p95_ms"] < 50.0
    assert constraint["pass"] is True
    assert constraint["observed_max_abs_command_m_s2"] <= 2.0 + 1.0e-9
    assert constraint["observed_max_step_m_s2"] <= 0.25 + 1.0e-9


def test_frozen_boundaries_and_holdout_remain_untouched() -> None:
    gate = read("gate.json")
    holdout = read("holdout_access_audit.json")
    assert gate["three_traditional_unchanged"] is True
    assert gate["advanced_contract_unchanged"] is True
    assert all(item["start_tree"] == item["current_tree"] for item in gate["protected_trees"].values())
    assert gate["advanced_paper_started"] is False
    assert gate["holdout_executed"] is False
    assert holdout["holdout_manifest_loaded"] is False
    assert holdout["holdout_executed"] is False
