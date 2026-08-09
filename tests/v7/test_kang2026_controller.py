"""Structural tests for the frozen Kang-2026 adaptation."""

from __future__ import annotations

import numpy as np

from uav_sway.task_space.state import CutterTaskState
from uav_sway.v3.observation import V3Observation, V3Reference
from uav_sway.v7.kang2026_fas_dob import Kang2026FASDOB


PARAMETERS = {
    "candidate_id": "test", "position_gain": 1.1, "velocity_gain": 1.4,
    "virtual_constraint_gain": 0.9, "virtual_length_scale": 0.8,
    "observer_lambda1": 0.8, "observer_lambda2": 0.6, "observer_rho": 0.8,
    "observer_clip_m_s2": 0.6, "observer_compensation_scale": 0.25,
    "observer_substeps": 5, "axis_scale": [1.0, 1.0, 0.85],
}


def observation(tip=(0.2, -0.1, 2.0), tip_velocity=(0.0, 0.0, 0.0)) -> V3Observation:
    task = CutterTaskState(
        tip_position_world=np.asarray(tip), tip_velocity_world=np.asarray(tip_velocity),
        cutter_axis_world=np.asarray([1.0, 0.0, 0.0]), cutter_angular_velocity_world=np.zeros(3),
        cutter_rotation_world=np.eye(3),
    )
    return V3Observation(np.zeros(20), np.asarray([0.0, 0.0, 3.0]), np.zeros(3), task)


def reference() -> V3Reference:
    return V3Reference(np.asarray([0.1, 0.0, 3.0]), np.zeros(3), np.asarray([0.1, 0.0, 2.0]), 0.0)


def test_command_is_finite_and_within_common_authority() -> None:
    controller = Kang2026FASDOB(PARAMETERS)
    previous = np.zeros(3)
    for _ in range(20):
        command = controller.command(observation(), reference())
        assert np.isfinite(command).all()
        assert np.max(np.abs(command)) <= 2.0 + 1.0e-12
        assert np.max(np.abs(command - previous)) <= 0.25 + 1.0e-12
        previous = command


def test_reset_is_deterministic_and_dob_is_causal() -> None:
    controller = Kang2026FASDOB(PARAMETERS)
    first = controller.command(observation(), reference())
    for _ in range(5):
        controller.command(observation(tip=(0.3, -0.2, 1.9)), reference())
    controller.reset()
    repeated = controller.command(observation(), reference())
    np.testing.assert_allclose(first, repeated, atol=0.0, rtol=0.0)
    assert np.max(np.abs(controller.diagnostics.disturbance_hat)) <= PARAMETERS["observer_clip_m_s2"]


def test_equivalent_swing_is_three_dimensional() -> None:
    controller = Kang2026FASDOB(PARAMETERS)
    controller.command(observation(tip=(0.3, -0.2, 1.8)), reference())
    value = controller.diagnostics.equivalent_swing_displacement
    assert value.shape == (3,)
    assert np.count_nonzero(np.abs(value) > 1.0e-12) == 3
