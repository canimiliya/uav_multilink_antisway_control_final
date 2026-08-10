import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PATH = ROOT / "reproducibility/native_stack/governance/competence_governance_v2.json"


def test_v2_changes_governance_only():
    contract = json.loads(PATH.read_text(encoding="utf-8"))
    assert contract["frozen_before_retrospective_existing_method_diagnostic"] is True
    for key in ("case_manifests_changed", "case_semantics_changed", "plant_changed", "wind_changed", "metric_definitions_changed", "success_definition_changed"):
        assert contract[key] is False


def test_v2_separates_eligibility_from_challenge_robustness():
    contract = json.loads(PATH.read_text(encoding="utf-8"))
    assert contract["cohorts"]["nominal"] == ["calm", "moderate", "stochastic"]
    assert contract["cohorts"]["challenge"] == ["strong_sustained", "strong_transient", "ramp"]
    assert contract["traditional_eligibility"]["nominal_success_rate_min"] == 0.70
    assert contract["challenge_robustness"]["eligibility_role"] == "REPORT_ONLY_EXCEPT_ALL_CASE_SAFETY"


def test_v2_forbids_posthoc_rescue():
    rules = json.loads(PATH.read_text(encoding="utf-8"))["anti_posthoc_rules"]
    assert rules["existing_controller_result_used_to_set_thresholds"] is False
    assert rules["native_pid_001_targeted_to_pass"] is False
    assert rules["thresholds_may_not_be_changed_after_retrospective"] is True
    assert rules["v1_history_may_not_be_relabelled"] is True
