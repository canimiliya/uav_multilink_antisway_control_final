"""V3-R2 Self-Advanced preregistration and boundary tests."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R2 = ROOT / "reproducibility/v3/r2"


def read(name: str) -> dict:
    return json.loads((R2 / name).read_text(encoding="utf-8"))


def test_method_family_is_frozen_before_performance() -> None:
    contract = read("self_method_contract.json")
    assert contract["method"] == "3D-DR-TSRMPC"
    assert contract["written_before_any_self_performance"] is True
    assert contract["state_model"]["dimension"] == 20
    assert contract["control"]["components"] == ["ax", "ay", "az"]
    assert contract["control"]["physical_commands_constrained_inside_qp"] is True


def test_first_search_is_preregistered_and_bounded() -> None:
    history = read("self_search_history.json")
    first = history["rounds"][0]
    assert first["SEARCH_ROUND"] == "R2-A"
    assert first["ARCHITECTURE_VERSION"] == "3D-DR-TSRMPC-A1"
    assert first["MAX_EVALUATIONS"] == 48
    assert history["written_before_any_self_performance"] is True
    assert first["performance_started"] is True
    assert first["performance_completed"] is True
    assert first["authoritative_case_executions"] == 672
    assert first["holdout_executed"] is False


def test_prohibited_data_and_methods_are_explicitly_absent() -> None:
    contract = read("self_method_contract.json")
    residual = contract["dynamic_residual"]
    assert residual["future_wind_truth"] is False
    assert contract["steady_compensation"]["external_tip_target_moves"] is False
    assert contract["predictive_controller"]["fallback_or_traditional_override"] is False
    assert contract["paper_advanced_started"] is False
    assert contract["holdout_executed"] is False
