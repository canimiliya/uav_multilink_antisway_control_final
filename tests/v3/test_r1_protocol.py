"""Pre-performance tests for the V3-R1 traditional-baseline protocol."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from uav_sway.control.base import ReferenceState
from uav_sway.v3.contracts import V3AccelerationCommand, V3AccelerationLimiter
from uav_sway.v3.observation import map_tip_target_to_reference


ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility" / "v3" / "r0"
R1 = ROOT / "reproducibility" / "v3" / "r1"


def read(name: str) -> dict:
    return json.loads((R1 / name).read_text(encoding="utf-8"))


def test_v3_command_and_shared_limiter_contract() -> None:
    command = V3AccelerationCommand.from_array([0.1, -0.2, 0.3])
    assert command.as_array().shape == (3,)
    limiter = V3AccelerationLimiter()
    first = limiter.limit([9.0, -9.0, 9.0]).as_array()
    assert np.allclose(first, [0.25, -0.25, 0.25])
    second = limiter.limit([9.0, -9.0, 9.0]).as_array()
    assert np.allclose(second, [0.5, -0.5, 0.5])
    assert np.max(np.abs(second)) <= 2.0


def test_reference_mapping_and_all_step_directions_are_causal() -> None:
    relative = np.asarray([0.225, 0.0, -2.81])
    for delta in ([0.15, 0, 0], [0, 0.15, 0], [0, 0, 0.15], [0.15, 0.15, 0.15]):
        target = relative + np.asarray(delta)
        assert np.allclose(map_tip_target_to_reference(target, relative), delta)
    assert ReferenceState(0, 0, 0, 0, 3.2, 0).yaw_ref == 0


def test_exact_grid_sizes_and_manifest() -> None:
    assert {axis: len(values) for axis, values in read("pid_axis_grids.json").items()} == {"x": 18, "y": 18, "z": 18}
    assert read("pid_combination_grid.json")["grid_size"] == 8
    assert read("full_lqr_grid.json")["grid_size"] == 64
    assert read("task_lqr_grid.json")["grid_size"] == 27
    manifest = read("development_evaluation_manifest.json")
    assert manifest["sample_count"] == 75
    assert manifest["development_seeds"] == list(range(2000, 2020))
    assert {row["wind"]["kind"] for row in manifest["samples"]} == {"calm", "constant", "stochastic", "ramp"}


def test_r0_matrices_and_model_hash_are_read_only_inputs() -> None:
    linear = read_r0("linear_model_audit.json")
    assert linear["A_shape"] == [20, 20]
    assert linear["B_shape"] == [20, 3]
    assert linear["B_rank"] == 3
    assert linear["stabilizable"] is True
    assert read_r0("plant_parity.json")["model_sha256"] == "19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d"


def test_holdout_is_not_an_r1_input() -> None:
    holdout = read_r0("holdout_manifest.json")
    assert holdout["execution_allowed"] is False
    assert holdout["holdout_seeds"] == list(range(3000, 3020))


def read_r0(name: str) -> dict:
    return json.loads((R0 / name).read_text(encoding="utf-8"))


def test_controller_modules_are_importable_without_performance_execution() -> None:
    from uav_sway.task_space.state import CutterTaskState
    from uav_sway.v3.controllers import V3FullStateLQR, V3TaskPID, V3TaskWeightedLQR
    from uav_sway.v3.observation import V3Observation, V3Reference

    zero = np.zeros(3)
    task = CutterTaskState(zero, zero, [0, 0, 0], [1, 0, 0], np.eye(3))
    observation = V3Observation(np.zeros(20), zero, zero, task)
    reference = V3Reference(zero, zero, zero, 0.0)
    pid = V3TaskPID(np.ones(3), np.ones(3), zero)
    assert pid.command(observation, reference).shape == (3,)
    assert np.isfinite(pid.diagnostics.command).all()
    assert pid.limiter.previous.shape == (3,)
    with pytest.raises(ValueError):
        V3FullStateLQR(np.zeros((1, 20)))
    with pytest.raises(ValueError):
        V3TaskWeightedLQR(np.zeros((3, 19)))
