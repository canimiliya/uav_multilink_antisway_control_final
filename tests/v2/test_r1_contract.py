"""Unit tests for the pre-performance V2-R1 contract freeze."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from uav_sway.task_space.v2_reference import CutterTargetMapper, Shared3DControlLimits


ROOT = Path(__file__).resolve().parents[2]
R1 = ROOT / "reproducibility/v2/r1"


def load(name: str) -> dict:
    return json.loads((R1 / name).read_text(encoding="utf-8"))


def test_cutter_target_maps_to_nominal_uav_reference_without_mutation() -> None:
    mapper = CutterTargetMapper(np.array([0.225, 0.0, -2.81]), np.array([1.0, 0.0, 0.0]))
    target = np.array([0.30, -0.10, 0.50])
    copy = target.copy()
    reference = mapper.uav_reference_from_tip_target(target)
    assert np.allclose(reference, target - [0.225, 0.0, -2.81])
    assert np.array_equal(target, copy)
    assert np.allclose(mapper.tip_target_from_uav_reference(reference), target)


def test_shared_yz_limits_and_slew() -> None:
    limits = Shared3DControlLimits()
    ay, az = limits.apply(9.0, -9.0)
    assert (ay, az) == (0.25, -0.25)
    ay, az = limits.apply(9.0, -9.0, ay, az)
    assert (ay, az) == (0.5, -0.5)
    ay, az = limits.apply(9.0, -9.0, 1.9, -1.9)
    assert abs(ay) <= 2.0 and abs(az) <= 2.0


def test_sample_bank_namespaces_and_holdout_are_not_executable() -> None:
    contract = load("sample_bank_contract.json")
    development = load("development_manifest.json")
    holdout = load("holdout_manifest.json")
    assert contract["development_target_count"] == 12
    assert len(development["samples"]) == 57
    assert len(holdout["samples"]) == 23
    assert all(row["execution_allowed"] for row in development["samples"])
    assert not holdout["execution_allowed"]
    assert all(not row["execution_allowed"] for row in holdout["samples"])
    assert {row["wind"].get("seed") for row in development["samples"] if "seed" in row["wind"]} == set(range(20))
    assert {row["wind"].get("seed") for row in holdout["samples"] if "seed" in row["wind"]} == set(range(1000, 1020))


def test_r1_grid_sizes_are_exact() -> None:
    grid = load("shared_yz_grid.json")
    assert len(grid["y"]) == 9
    assert len(grid["z"]) == 9
