"""V3 adaptation of Lv et al.'s suspension-point acceleration cascade.

The controller preserves the paper's shaped swing-error middle loop while
respecting the frozen V3 acceleration interface.  It is an adaptation, not an
exact reproduction: a five-link cutter-tip vector replaces the single cable,
and a frozen Traditional LQR supplies the nominal plant stabilization.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .controllers import V3ControllerDiagnostics, _V3ControllerBase
from .observation import V3Observation, V3Reference


GRAVITY_M_S2 = 9.81
PAPER_REPORTED_SWING_GAIN = 3.2


@dataclass(frozen=True)
class EquivalentSwing3D:
    relative_vector_world: np.ndarray
    relative_velocity_world: np.ndarray
    length_m: float
    alpha_rad: float
    beta_rad: float
    alpha_rate_rad_s: float
    beta_rate_rad_s: float


@dataclass(frozen=True)
class V3LV2026Diagnostics(V3ControllerDiagnostics):
    nominal_command: np.ndarray
    outer_velocity_correction: np.ndarray
    swing_correction_raw: np.ndarray
    swing_correction: np.ndarray
    equivalent_length_m: float
    alpha_rad: float
    beta_rad: float
    alpha_rate_rad_s: float
    beta_rate_rad_s: float


def _angle_rate(position: float, vertical: float, velocity: float, vertical_velocity: float) -> tuple[float, float]:
    denominator = float(position * position + vertical * vertical)
    if denominator <= 1.0e-12:
        raise ValueError("equivalent swing projection is too short")
    angle = float(np.arctan2(position, -vertical))
    rate = float((-vertical * velocity + position * vertical_velocity) / denominator)
    return angle, rate


def equivalent_swing_3d(observation: V3Observation) -> EquivalentSwing3D:
    relative = observation.task_state.tip_position_world - observation.uav_position_world
    relative_velocity = observation.task_state.tip_velocity_world - observation.uav_velocity_world
    length = float(np.linalg.norm(relative))
    if length <= 1.0e-12 or not np.isfinite(np.r_[relative, relative_velocity]).all():
        raise ValueError("equivalent five-link swing state must be finite and nonzero")
    alpha, alpha_rate = _angle_rate(relative[1], relative[2], relative_velocity[1], relative_velocity[2])
    beta, beta_rate = _angle_rate(relative[0], relative[2], relative_velocity[0], relative_velocity[2])
    return EquivalentSwing3D(
        relative.copy(), relative_velocity.copy(), length,
        alpha, beta, alpha_rate, beta_rate,
    )


def paper_shaped_acceleration(angle: float, rate: float, gain: float) -> float:
    """Lv et al. Eq.18 for zero desired swing and equal shaped gains."""
    error = -float(angle)
    shaped_rate = -float(rate) + float(gain) * error
    return float((1.0 - gain * gain) * error + 2.0 * gain * shaped_rate)


def suspension_acceleration_correction(angle: float, rate: float, length: float, gain: float) -> float:
    desired_angle_acceleration = paper_shaped_acceleration(angle, rate, gain)
    return float(-length * desired_angle_acceleration - GRAVITY_M_S2 * np.sin(angle))


class V3LV2026SPACC(_V3ControllerBase):
    """LV2026-SPACC-ADAPTED-3D under the frozen V3 controller authority."""

    def __init__(self, gain: np.ndarray, parameters: dict) -> None:
        super().__init__()
        self.gain = np.asarray(gain, dtype=float).reshape(3, 20).copy()
        self.outer_position_gain = float(parameters["outer_position_gain"])
        self.outer_velocity_gain = float(parameters["outer_velocity_gain"])
        self.tip_speed_limit = float(parameters["desired_tip_speed_limit_m_s"])
        self.swing_gain = PAPER_REPORTED_SWING_GAIN * float(parameters["swing_gain_scale_of_reported_3_2"])
        self.swing_scale = float(parameters["swing_correction_scale"])
        self.swing_clip = float(parameters["swing_correction_clip_m_s2"])
        values = np.r_[self.gain.ravel(), self.outer_position_gain, self.outer_velocity_gain,
                       self.tip_speed_limit, self.swing_gain, self.swing_scale, self.swing_clip]
        if not np.isfinite(values).all() or np.any(values[-6:] <= 0.0):
            raise ValueError("LV2026 adaptation parameters must be finite and positive")
        self.diagnostics = self._paper_diagnostics(np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), None)

    def _paper_diagnostics(self, raw, nominal, outer, swing_raw, swing: EquivalentSwing3D | None):
        raw = np.asarray(raw, dtype=float).reshape(3)
        amplitude = np.clip(raw, -self.limiter.absolute_limit_m_s2, self.limiter.absolute_limit_m_s2)
        command = self.limiter.previous.copy()
        base = self._diagnostics(raw, amplitude, command, command, 0.0, np.zeros(3), self.limiter)
        if swing is None:
            length = alpha = beta = alpha_rate = beta_rate = 0.0
        else:
            length = swing.length_m
            alpha, beta = swing.alpha_rad, swing.beta_rad
            alpha_rate, beta_rate = swing.alpha_rate_rad_s, swing.beta_rate_rad_s
        swing_limited = self.swing_scale * np.clip(np.asarray(swing_raw, dtype=float), -self.swing_clip, self.swing_clip)
        return V3LV2026Diagnostics(
            **base.__dict__,
            nominal_command=np.asarray(nominal, dtype=float).reshape(3).copy(),
            outer_velocity_correction=np.asarray(outer, dtype=float).reshape(3).copy(),
            swing_correction_raw=np.asarray(swing_raw, dtype=float).reshape(3).copy(),
            swing_correction=swing_limited.copy(),
            equivalent_length_m=float(length), alpha_rad=float(alpha), beta_rad=float(beta),
            alpha_rate_rad_s=float(alpha_rate), beta_rate_rad_s=float(beta_rate),
        )

    def reset(self) -> None:
        super().reset()
        self.diagnostics = self._paper_diagnostics(np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), None)

    def command(self, observation: V3Observation, reference: V3Reference, dt: float = 0.05) -> np.ndarray:
        del dt
        swing = equivalent_swing_3d(observation)
        nominal = -self.gain @ observation.full_state_error
        position_error = reference.tip_position_world - observation.task_state.tip_position_world
        desired_velocity = np.clip(
            self.outer_position_gain * position_error,
            -self.tip_speed_limit,
            self.tip_speed_limit,
        )
        outer = self.outer_velocity_gain * (desired_velocity - observation.task_state.tip_velocity_world)
        swing_raw = np.asarray([
            suspension_acceleration_correction(swing.beta_rad, swing.beta_rate_rad_s, swing.length_m, self.swing_gain),
            suspension_acceleration_correction(swing.alpha_rad, swing.alpha_rate_rad_s, swing.length_m, self.swing_gain),
            0.0,
        ])
        swing_correction = self.swing_scale * np.clip(swing_raw, -self.swing_clip, self.swing_clip)
        raw = nominal + outer + swing_correction
        result = self._finish(raw, np.linalg.norm(observation.full_state_error), np.zeros(3))
        self.diagnostics = self._paper_diagnostics(raw, nominal, outer, swing_raw, swing)
        # _paper_diagnostics reads the limiter's accepted output after _finish.
        self.diagnostics = V3LV2026Diagnostics(
            **{**self.diagnostics.__dict__, "command": result.copy()}
        )
        return result
