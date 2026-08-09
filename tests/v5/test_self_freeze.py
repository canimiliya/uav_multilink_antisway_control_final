import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SELF = ROOT / "reproducibility/v5/self"


def read(name): return json.loads((SELF / name).read_text(encoding="utf-8"))


def test_self_is_confirmed_and_frozen_without_holdout():
    value = read("self_freeze.json")
    assert value["status"] == "V5_SELF_DEVELOPMENT_QUALIFIED_FROZEN"
    assert value["stage_b_pass"] is True
    assert value["stage_c_confirmation_pass"] is True
    assert value["metrics"]["hard_gate_pass_count"] == value["metrics"]["hard_gate_count"] == 20
    assert value["parameters_and_implementation_mutable_after_freeze"] is False
    assert value["holdout_executed"] is False


def test_directional_tail_and_authority_gates_pass():
    directional = read("directional_analysis.json")
    constraint = read("constraint_audit.json")
    assert directional["all_directional_hard_gates_pass"] is True
    assert directional["coordinate_sign_rule_used"] is False
    assert constraint["within_frozen_authority"] is True
    assert constraint["max_observed_acceleration_m_s2"] <= 2.0 + 1e-9
    assert constraint["max_observed_step_m_s2"] <= 0.25 + 1e-9
