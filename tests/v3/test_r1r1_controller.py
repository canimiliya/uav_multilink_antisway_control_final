"""Correctness and method-family checks for the V3-R1R1 PID."""

from __future__ import annotations

import inspect

import numpy as np

from uav_sway.task_space.state import CutterTaskState
from uav_sway.v3.controllers import V3CascadedTaskPID
from uav_sway.v3.observation import V3Observation, V3Reference


def make_controller() -> V3CascadedTaskPID:
    return V3CascadedTaskPID(
        uav_kp=[10, 10, 10], uav_kd=[0, 0, 0], uav_ki=[1, 1, 1],
        tip_kp=[0, 0, 0], tip_kd=[0, 0, 0], correction_limit_m=[0.2, 0.2, 0.2],
        correction_slew_m_per_update=0.05, integral_limit=0.5,
    )


def observation(uav_position: np.ndarray, tip_position: np.ndarray | None = None) -> V3Observation:
    zero = np.zeros(3)
    tip = np.asarray(uav_position if tip_position is None else tip_position, dtype=float)
    task = CutterTaskState(tip, zero, zero, [1, 0, 0], np.eye(3))
    return V3Observation(np.zeros(20), np.asarray(uav_position, dtype=float), zero, task)


def reference() -> V3Reference:
    zero = np.zeros(3)
    return V3Reference(zero, zero, zero, 0.0)


def test_positive_and_negative_amplitude_antiwindup_freezes_correct_direction() -> None:
    for sign in (-1.0, 1.0):
        controller = make_controller()
        controller.limiter.previous[:] = -2.0 * sign
        controller.command(observation(np.asarray([sign, 0, 0])), reference(), 0.05)
        assert controller.diagnostics.saturated[0]
        assert controller.integral[0] == 0.0


def test_positive_and_negative_slew_antiwindup_freezes_correct_direction() -> None:
    for sign in (-1.0, 1.0):
        controller = make_controller()
        controller.uav_kp[:] = 1.0
        controller.command(observation(np.asarray([sign, 0, 0])), reference(), 0.05)
        assert controller.diagnostics.slew_limited[0]
        assert controller.integral[0] == 0.0


def test_integral_is_allowed_when_it_unloads_a_constraint() -> None:
    controller = make_controller()
    controller.integral[0] = 0.5
    controller.limiter.previous[0] = 2.0
    controller.command(observation(np.asarray([-0.1, 0, 0])), reference(), 0.05)
    assert controller.integral[0] == 0.495


def test_reference_correction_is_causal_bounded_and_rate_limited() -> None:
    controller = make_controller()
    controller.tip_kp[:] = 2.0
    controller.command(observation(np.zeros(3), np.ones(3)), reference(), 0.05)
    assert np.allclose(controller.reference_correction, -0.05)
    for _ in range(10):
        controller.command(observation(np.zeros(3), np.ones(3)), reference(), 0.05)
    assert np.allclose(controller.reference_correction, -0.2)


def test_reset_is_bumpless_and_controller_has_no_wind_or_future_inputs() -> None:
    controller = make_controller()
    controller.command(observation(np.ones(3)), reference(), 0.05)
    controller.reset()
    assert np.allclose(controller.integral, 0.0)
    assert np.allclose(controller.reference_correction, 0.0)
    assert np.allclose(controller.limiter.previous, 0.0)
    signature = inspect.signature(controller.command)
    assert list(signature.parameters) == ["observation", "reference", "dt"]


def test_relative_tip_velocity_mode_removes_whole_uav_translation() -> None:
    controller = make_controller()
    controller.tip_velocity_mode = "relative_to_uav"
    controller.tip_kd[:] = 1.0
    zero = np.zeros(3)
    velocity = np.asarray([0.4, -0.2, 0.1])
    task = CutterTaskState(zero, velocity, zero, [1, 0, 0], np.eye(3))
    obs = V3Observation(np.zeros(20), zero, velocity, task)
    controller.uav_kd[:] = 0.0
    controller.command(obs, reference(), 0.05)
    assert np.allclose(controller.reference_correction, 0.0)
