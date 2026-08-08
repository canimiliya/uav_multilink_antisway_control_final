"""Contract tests for the DR-TSRMPC preregistration artifact."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "reproducibility/v2/r3r1"


def read(name: str):
    return json.loads((OUT / name).read_text(encoding="utf-8"))


def test_dr_preregistration_gate_passes_without_performance():
    gate = read("gate.json")
    assert gate["result"] == "V2_DR_TSRMPC_PREREGISTERED"
    assert gate["new_self_performance_executed"] is False
    assert gate["paper_advanced_executed"] is False
    assert gate["holdout_executed"] is False
    assert gate["parameter_grid_size"] == 36
    assert gate["grid_identical_to_r3"] is True


def test_dynamic_residual_contract_is_causal_and_16_dimensional():
    contract = read("dynamic_residual_contract.json")
    assert contract["residual_dimension"] == 16
    assert contract["causality_pass"] is True
    assert contract["matched_wind_model_used"] is False
    assert contract["beta_grid"] == [0.05, 0.15, 0.3]


def test_reference_shift_and_steady_state_audits_pass():
    reference = read("reference_shift_audit.json")
    steady = read("steady_state_feasibility_audit.json")
    assert reference["pass"] is True
    assert reference["reference_only_false_residual"] <= reference["tolerance"]
    assert steady["pass"] is True
    assert steady["max_equality_residual_norm"] <= steady["equality_tolerance"]


def test_of_route_is_preserved_as_structural_negative():
    failure = read("of_tsrmpc_failure_mechanism.json")
    assert failure["of_tsrmpc_status"] == "CLOSED_WITH_NO_DEVELOPMENT_WIN"
    assert failure["task_lqr_baseline"]["candidate_id"] == "task_lqr_001"
    assert failure["pid_baseline"]["candidate_id"] == "pid_005"
    assert failure["implementation_bug"] is False
    assert failure["structural_limitation"] is True
    assert failure["performance_rerun"] is False


def test_shared_yz_and_model_interface_remain_frozen():
    parity = read("advanced_interface_parity.json")
    shared = json.loads((ROOT / "reproducibility/v2/r1r1/shared_task_yz_freeze.json").read_text(encoding="utf-8"))
    assert parity["shared_yz"] == shared["selected"]
    assert parity["A_shape"] == [16, 16]
    assert parity["B_shape"] == [16, 1]
    assert parity["C_task_shape"] == [4, 16]
    assert parity["pass"] is True
