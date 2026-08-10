import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
GOV = ROOT / "reproducibility/native_stack/governance"


def test_clean_governance_freeze_preserves_physics_and_success_semantics():
    contract = json.loads((GOV / "competence_governance_v2.json").read_text(encoding="utf-8"))
    for key in ("case_manifests_changed", "case_semantics_changed", "plant_changed", "wind_changed", "metric_definitions_changed", "success_definition_changed"):
        assert contract[key] is False


def test_clean_cohort_roles_cover_all_frozen_wind_kinds():
    roles = json.loads((GOV / "cohort_role_definition.json").read_text(encoding="utf-8"))
    winds = roles["nominal"]["wind_kinds"] + roles["challenge"]["wind_kinds"]
    assert len(winds) == len(set(winds)) == 6
    assert roles["case_identity_changed"] is False


def test_clean_governance_has_no_existing_controller_rescue():
    contract = json.loads((GOV / "competence_governance_v2.json").read_text(encoding="utf-8"))
    assert contract["anti_posthoc_rules"]["native_pid_001_targeted_to_pass"] is False
    assert contract["anti_posthoc_rules"]["future_qualification_requires_new_task_protocol_and_freeze"] is True
