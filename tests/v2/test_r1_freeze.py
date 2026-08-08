"""Post-freeze V2-R1 audit tests."""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

import numpy as np

from uav_sway.task_space.v2_reference import CutterTargetMapper, Shared3DControlLimits


ROOT = Path(__file__).resolve().parents[2]
R1 = ROOT / "reproducibility/v2/r1"


def read(name: str) -> dict:
    return json.loads((R1 / name).read_text(encoding="utf-8"))


def test_external_target_does_not_change_internal_equilibrium_geometry() -> None:
    mapper = CutterTargetMapper(np.array([0.225, 0.0, -2.81]), np.array([1.0, 0.0, 0.0]))
    first = mapper.tip_relative_equilibrium_m.copy()
    _ = mapper.uav_reference_from_tip_target(np.array([0.6, 0.2, 0.8]))
    assert np.array_equal(first, mapper.tip_relative_equilibrium_m)


def test_shared_yz_limits_are_common_to_all_three_frozen_methods() -> None:
    selected = [read(name)["shared_yz"] for name in ("pid_freeze.json", "lqr_freeze.json", "task_lqr_freeze.json")]
    assert selected[0] == selected[1] == selected[2]
    assert read("shared_yz_freeze.json")["frozen_limits"] == {"ay_abs_max_m_s2": 2.0, "az_abs_max_m_s2": 2.0, "axis_slew_max_m_s2_per_update": 0.25}


def test_all_required_grid_sizes_are_exact() -> None:
    assert read("pid_grid.json")["grid_size"] == 18
    assert read("lqr_grid.json")["grid_size"] == 64
    assert read("task_lqr_grid.json")["grid_size"] == 27


def test_development_and_holdout_namespaces_are_disjoint() -> None:
    development = read("development_manifest.json")["samples"]
    holdout = read("holdout_manifest.json")["samples"]
    dev_seeds = {row["wind"]["seed"] for row in development if "seed" in row["wind"]}
    holdout_seeds = {row["wind"]["seed"] for row in holdout if "seed" in row["wind"]}
    assert dev_seeds == set(range(20))
    assert holdout_seeds == set(range(1000, 1020))
    assert dev_seeds.isdisjoint(holdout_seeds)


def test_holdout_targets_are_manifest_only_and_never_in_raw_runs() -> None:
    assert read("holdout_manifest.json")["execution_allowed"] is False
    assert not list((R1 / "runs").rglob("*holdout*"))
    assert not list((R1 / "runs").rglob("*1000*"))
    assert not list((R1 / "runs").rglob("*1001*"))


def test_lexicographic_score_is_monotone_in_safety_and_success() -> None:
    def key(row: dict) -> tuple:
        return (-row["safe_sample_count"], -row["task_success_rate"], row["acquisition_median_s"], row["position_rmse_3d_m"], row["orientation_rmse_deg"], row["control_effort"])
    safe = {"safe_sample_count": 57, "task_success_rate": 0.1, "acquisition_median_s": 9.0, "position_rmse_3d_m": 9.0, "orientation_rmse_deg": 9.0, "control_effort": 9.0}
    unsafe = {**safe, "safe_sample_count": 56, "task_success_rate": 1.0, "acquisition_median_s": 0.0, "position_rmse_3d_m": 0.0, "orientation_rmse_deg": 0.0, "control_effort": 0.0}
    assert key(safe) < key(unsafe)
    high_success = {**safe, "task_success_rate": 0.2}
    assert key(high_success) < key(safe)


def test_advanced_contract_exists_before_any_advanced_run() -> None:
    contract = read("advanced_win_contract.json")
    gate = read("gate.json")
    assert contract["written_before_any_advanced_performance"] is True
    assert gate["advanced_self_started"] is False
    assert gate["advanced_paper_selected"] is False


def test_three_baselines_and_primary_are_frozen() -> None:
    assert read("pid_freeze.json")["selected"]["sample_count"] == 57
    assert read("lqr_freeze.json")["selected"]["sample_count"] == 57
    assert read("task_lqr_freeze.json")["selected"]["sample_count"] == 57
    assert read("primary_traditional_baseline.json")["primary_controller"] == "lqr_041"
    assert read("traditional_metric_envelope.json")["best_success_rate"]["value"] > 0.0


def test_main_and_v1_tag_are_unchanged() -> None:
    main = subprocess.check_output(["git", "rev-parse", "main"], cwd=ROOT, text=True).strip()
    tag = subprocess.check_output(["git", "rev-list", "-n", "1", "v1.0.0^{}"], cwd=ROOT, text=True).strip()
    assert main == "62769122b6b75cd124c9cabc48aee2976a159f6b"
    assert tag == main
