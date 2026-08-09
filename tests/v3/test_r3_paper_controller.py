import numpy as np

from uav_sway.task_space.state import CutterTaskState
from uav_sway.v3.lv2026_spacc import (
    PAPER_REPORTED_SWING_GAIN,
    V3LV2026SPACC,
    paper_shaped_acceleration,
    suspension_acceleration_correction,
)
from uav_sway.v3.observation import V3Observation, V3Reference


def _observation(tip=(0.0, 0.0, -2.5), tip_velocity=(0.0, 0.0, 0.0)) -> V3Observation:
    task = CutterTaskState(
        tip_position_world=np.asarray(tip, dtype=float),
        tip_velocity_world=np.asarray(tip_velocity, dtype=float),
        cutter_axis_world=np.asarray([1.0, 0.0, 0.0]),
        cutter_angular_velocity_world=np.zeros(3),
        cutter_rotation_world=np.eye(3),
    )
    return V3Observation(np.zeros(20), np.zeros(3), np.zeros(3), task)


def _parameters() -> dict:
    return {
        "outer_position_gain": 0.25,
        "outer_velocity_gain": 0.5,
        "desired_tip_speed_limit_m_s": 0.5,
        "swing_gain_scale_of_reported_3_2": 0.125,
        "swing_correction_scale": 0.1,
        "swing_correction_clip_m_s2": 0.5,
    }


def test_paper_middle_loop_restores_positive_swing() -> None:
    gain = PAPER_REPORTED_SWING_GAIN * 0.125
    desired = paper_shaped_acceleration(0.1, 0.0, gain)
    correction = suspension_acceleration_correction(0.1, 0.0, 2.5, gain)
    assert desired < 0.0
    np.testing.assert_allclose(correction, -2.5 * desired - 9.81 * np.sin(0.1))


def test_controller_is_three_axis_causal_and_limited() -> None:
    controller = V3LV2026SPACC(np.zeros((3, 20)), _parameters())
    reference = V3Reference(np.zeros(3), np.zeros(3), np.asarray([0.2, 0.1, -2.5]), 1.0)
    first = controller.command(_observation(), reference)
    second = controller.command(_observation(), reference)
    assert first.shape == (3,)
    assert np.all(np.abs(first) <= 0.25 + 1.0e-12)
    assert np.all(np.abs(second - first) <= 0.25 + 1.0e-12)
    assert first[0] > 0.0 and first[1] > 0.0


def test_equilibrium_has_zero_paper_swing_correction() -> None:
    controller = V3LV2026SPACC(np.zeros((3, 20)), _parameters())
    reference = V3Reference(np.zeros(3), np.zeros(3), np.asarray([0.0, 0.0, -2.5]), 0.0)
    command = controller.command(_observation(), reference)
    np.testing.assert_allclose(command, np.zeros(3), atol=1.0e-12)
    np.testing.assert_allclose(controller.diagnostics.swing_correction, np.zeros(3), atol=1.0e-12)
