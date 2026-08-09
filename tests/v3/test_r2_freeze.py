"""V3-R2 frozen Self candidate and evidence gates."""

from __future__ import annotations

import json
import hashlib
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


def test_ablation_contract_is_post_freeze_and_cannot_retune() -> None:
    contract = read("self_ablation_contract.json")
    assert contract["written_after_committed_self_freeze"] is True
    assert contract["self_freeze_commit"] == "58b6544ab6a123f08d1cf04f3b9a7b38a40647e5"
    assert contract["frozen_candidate"] == "self_a_034"
    assert contract["ablation_retuning_allowed"] is False
    assert contract["ablation_may_modify_self_freeze"] is False
    assert contract["holdout_executed"] is False


def test_ablation_is_post_freeze_and_full_method_is_necessary() -> None:
    ablation = read("self_ablation.json")
    gate = read("gate.json")
    assert ablation["executed_after_committed_self_freeze"] is True
    assert ablation["self_freeze_head"] == "58b6544ab6a123f08d1cf04f3b9a7b38a40647e5"
    assert ablation["no_ablation_retuning"] is True
    assert ablation["backbone"]["success_rate"] == 44 / 75
    assert ablation["predictive_only"]["success_rate"] == 39 / 75
    assert ablation["residual_only"]["success_rate"] == 25 / 75
    assert ablation["full"]["success_rate"] == 1.0
    assert ablation["holdout_executed"] is False
    assert gate["ablation_executed"] is True
    assert gate["ablation_after_committed_freeze"] is True
    assert gate["result"] == "V3_SELF_ADVANCED_FROZEN"


def test_representative_development_artifacts_match_manifest_hashes() -> None:
    manifest = read("visual_manifest.json")
    entries = manifest["traces"] + manifest["plots"]
    assert len(manifest["traces"]) == 4 and len(manifest["plots"]) == 1
    for entry in entries:
        path = ROOT / entry["path"]
        assert path.is_file()
        assert hashlib.sha256(path.read_bytes()).hexdigest() == entry["sha256"]
    assert manifest["holdout_executed"] is False
