from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from uav_sway.native_stack.r1r3_evaluation import competence_v2


ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "reproducibility/native_stack/r1r3/protocol"


def test_protocol_preserves_platform_and_holdout_barriers() -> None:
    contract = json.loads((BASE / "research_contract.json").read_text(encoding="utf-8"))
    assert contract["source_head"] == "88c3aef081fabab44d174bc8afd234ae634af3da"
    assert contract["benchmark_changed"] is False
    assert contract["governance_changed"] is False
    assert contract["holdout_execution_allowed"] is False
    assert contract["holdout_performance_access_allowed"] is False
    assert contract["paper_search"] is False


def test_stage_a_and_search_budgets_are_frozen() -> None:
    manifest = json.loads((BASE / "stage_a_manifest.json").read_text(encoding="utf-8"))
    search = json.loads((BASE / "search_contract.json").read_text(encoding="utf-8"))
    assert manifest["case_count"] == 42
    assert manifest["nominal_case_count"] == 36
    assert len(set(manifest["case_ids"])) == 42
    assert search["pid"]["total_max"] == 64
    assert search["satc_native"]["maximum_new_configs"] == 96


def test_competence_v2_requires_every_frozen_subgate() -> None:
    summary = {
        "case_count": 200, "nominal_case_count": 100, "challenge_case_count": 100,
        "all_safety_rate": 0.98, "all_catastrophic_count": 4, "all_deadline_miss_rate": 0.01,
        "nominal_success_rate": 0.70, "nominal_calm_success_rate": 0.50,
        "nominal_moderate_success_rate": 0.50, "nominal_stochastic_success_rate": 0.50,
        "nominal_setpoint_success_rate": 0.60, "nominal_trajectory_success_rate": 0.60,
        "nominal_setpoint_rmse_m": 1.25, "nominal_trajectory_rmse_m": 1.50,
    }
    assert competence_v2(summary)
    for key in ("all_safety_rate", "nominal_success_rate", "nominal_moderate_success_rate",
                "nominal_setpoint_success_rate"):
        failed = dict(summary)
        failed[key] = np.nextafter(failed[key], -np.inf)
        assert not competence_v2(failed)

