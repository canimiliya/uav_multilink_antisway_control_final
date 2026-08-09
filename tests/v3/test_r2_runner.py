"""Isolation and grid tests for the V3-R2 Development runner."""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

from run_v3_r2_self_advanced import core_samples, round_a_candidates  # noqa: E402
from run_v3_r1_baselines import read_json  # noqa: E402


def test_round_a_grid_matches_preregistered_factorial() -> None:
    candidates = round_a_candidates()
    assert len(candidates) == 48
    assert len({row["candidate_id"] for row in candidates}) == 48
    assert {row["backbone"] for row in candidates} == {"full_lqr_048", "task_lqr_009"}
    assert {row["horizon_updates"] for row in candidates} == {8, 12}
    assert {row["residual_beta"] for row in candidates} == {0.05, 0.15, 0.30}


def test_runner_uses_only_frozen_development_cases() -> None:
    manifest = read_json(ROOT / "reproducibility/v3/r1/development_evaluation_manifest.json")
    assert manifest["split"] == "development"
    core = core_samples(manifest["samples"])
    assert len(core) == 14
    assert all(sample["sample_id"] not in {"spherical", "holdout"} for sample in core)
    source = (ROOT / "scripts/run_v3_r2_self_advanced.py").read_text(encoding="utf-8").lower()
    assert "holdout_manifest" not in source
    assert "3000" not in source and "3019" not in source and "3.5" not in source


def test_round_b_protocol_uses_only_round_a_candidates() -> None:
    protocol = read_json(ROOT / "reproducibility/v3/r2/self_round_b_protocol.json")
    grid_ids = {row["candidate_id"] for row in round_a_candidates()}
    ids = protocol["SEARCH_SPACE"]["candidate_ids"]
    assert len(ids) == 6 and len(set(ids)) == 6
    assert set(ids) <= grid_ids
    assert protocol["written_before_R2_B_performance"] is True
    assert protocol["holdout_executed"] is False
