from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from uav_sway.task_space.state import CutterTaskState
from uav_sway.v3.observation import V3Observation, V3Reference
from uav_sway.v8.xu2025_cbs_ftdo import Xu2025CBSFTDO


ROOT = Path(__file__).resolve().parents[2]


def parameters() -> dict:
    modal = json.loads((ROOT / "reproducibility/v8/r0/modal_selection_contract.json").read_text())
    return {
        "candidate_id": "unit",
        "modal_basis": modal["modal_basis_mass_normalized"],
        "modal_projection": modal["joint_to_modal_projection"],
        "axis_scale": [1.0, 1.0, 1.0],
        "position_gain": 1.0,
        "velocity_gain": 2.4,
        "direction_gain": 4.0,
        "angular_gain": 8.0,
        "direction_filter_gain": 10.0,
        "angular_filter_gain": 10.0,
        "force_observer_k1": 1.0,
        "force_observer_k2": 1.0,
        "angular_observer_k1": 1.0,
        "angular_observer_k2": 1.0,
        "observer_boundary": 0.02,
        "observer_clip": 1.0,
        "effective_length_m": 2.81,
        "observer_compensation_scale": 0.25,
        "angular_coupling_scale": 0.5,
    }


def observation(position=(0.0, 0.0, 0.32), velocity=(0.0, 0.0, 0.0)) -> V3Observation:
    state = np.zeros(20)
    task = CutterTaskState(
        tip_position_world=np.asarray(position),
        tip_velocity_world=np.asarray(velocity),
        cutter_axis_world=np.asarray([1.0, 0.0, 0.0]),
        cutter_angular_velocity_world=np.zeros(3),
        cutter_rotation_world=np.eye(3),
    )
    return V3Observation(state, np.asarray([0.0, 0.0, 3.2]), np.zeros(3), task)


def reference() -> V3Reference:
    return V3Reference(np.asarray([0.0, 0.0, 3.2]), np.zeros(3), np.asarray([0.0, 0.0, 0.32]), 0.0)


def test_equilibrium_is_finite_and_near_zero() -> None:
    controller = Xu2025CBSFTDO(parameters())
    command = controller.command(observation(), reference())
    assert np.isfinite(command).all()
    assert np.linalg.norm(command) < 1.0e-9
    assert controller.diagnostics.modal_coordinates.shape == (2,)
    assert controller.diagnostics.reconstructed_joint_angles.shape == (5,)


def test_common_authority_and_slew_are_enforced() -> None:
    controller = Xu2025CBSFTDO(parameters())
    commands = [controller.command(observation(position=(5.0, -5.0, 5.0)), reference()) for _ in range(20)]
    assert all(np.max(np.abs(value)) <= 2.0 + 1.0e-12 for value in commands)
    assert all(np.max(np.abs(b - a)) <= 0.25 + 1.0e-12 for a, b in zip([np.zeros(3), *commands[:-1]], commands))


def test_reset_is_deterministic() -> None:
    controller = Xu2025CBSFTDO(parameters())
    first = controller.command(observation(position=(0.2, -0.1, 0.4)), reference())
    controller.command(observation(position=(0.1, 0.2, 0.5)), reference())
    controller.reset()
    repeated = controller.command(observation(position=(0.2, -0.1, 0.4)), reference())
    np.testing.assert_allclose(repeated, first, atol=0.0, rtol=0.0)


def test_wrong_outer_rate_is_rejected() -> None:
    controller = Xu2025CBSFTDO(parameters())
    try:
        controller.command(observation(), reference(), 0.1)
    except ValueError as error:
        assert "20 Hz" in str(error)
    else:
        raise AssertionError("wrong outer rate was accepted")
