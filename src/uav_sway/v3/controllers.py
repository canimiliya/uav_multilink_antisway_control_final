"""Three fair V3 traditional outer-loop controllers."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .contracts import V3AccelerationLimiter
from .observation import V3Observation, V3Reference


@dataclass(frozen=True)
class V3ControllerDiagnostics:
    raw_command: np.ndarray
    amplitude_limited: np.ndarray
    command: np.ndarray
    saturated: np.ndarray
    slew_limited: np.ndarray
    state_norm: float
    integral: np.ndarray


class _V3ControllerBase:
    def __init__(self) -> None:
        self.limiter = V3AccelerationLimiter()
        self.diagnostics = self._diagnostics(np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), 0.0, np.zeros(3))

    @staticmethod
    def _diagnostics(raw, amplitude, command, previous, state_norm, integral, limiter=None):
        raw = np.asarray(raw, dtype=float).reshape(3)
        amplitude = np.asarray(amplitude, dtype=float).reshape(3)
        command = np.asarray(command, dtype=float).reshape(3)
        previous = np.asarray(previous, dtype=float).reshape(3)
        if limiter is None:
            saturated = np.zeros(3, dtype=bool)
            slew = np.zeros(3, dtype=bool)
        else:
            saturated = np.abs(raw) > limiter.absolute_limit_m_s2 + 1.0e-12
            slew = np.abs(command - amplitude) > 1.0e-12
        return V3ControllerDiagnostics(raw.copy(), amplitude.copy(), command.copy(), saturated, slew, float(state_norm), np.asarray(integral, dtype=float).reshape(3).copy())

    def reset(self) -> None:
        self.limiter.reset()
        self.diagnostics = self._diagnostics(np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3), 0.0, np.zeros(3))

    def _finish(self, raw: np.ndarray, state_norm: float, integral: np.ndarray) -> np.ndarray:
        raw = np.asarray(raw, dtype=float).reshape(3)
        amplitude = np.clip(raw, -self.limiter.absolute_limit_m_s2, self.limiter.absolute_limit_m_s2)
        previous = self.limiter.previous.copy()
        result = self.limiter.limit(raw).as_array()
        self.diagnostics = self._diagnostics(raw, amplitude, result, previous, state_norm, integral, self.limiter)
        return result


class V3TaskPID(_V3ControllerBase):
    """Independent tip-space PID axes with conditional anti-windup."""

    def __init__(self, kp: np.ndarray, kd: np.ndarray, ki: np.ndarray, integral_limit: float = 1.0) -> None:
        super().__init__()
        self.kp = np.asarray(kp, dtype=float).reshape(3).copy()
        self.kd = np.asarray(kd, dtype=float).reshape(3).copy()
        self.ki = np.asarray(ki, dtype=float).reshape(3).copy()
        self.integral_limit = float(integral_limit)
        if not np.isfinite(np.r_[self.kp, self.kd, self.ki]).all() or self.integral_limit <= 0.0:
            raise ValueError("PID gains and integral limit must be finite and valid")
        self.integral = np.zeros(3, dtype=float)

    def reset(self) -> None:
        super().reset()
        self.integral = np.zeros(3, dtype=float)

    def command(self, observation: V3Observation, reference: V3Reference, dt: float = 0.05) -> np.ndarray:
        dt = float(dt)
        if dt <= 0.0 or not np.isfinite(dt):
            raise ValueError("dt must be positive and finite")
        position_error = observation.task_state.tip_position_world - reference.tip_position_world
        velocity_error = observation.task_state.tip_velocity_world - reference.uav_velocity_world
        proposed_integral = np.clip(self.integral + position_error * dt, -self.integral_limit, self.integral_limit)
        raw = -self.kp * position_error - self.kd * velocity_error - self.ki * proposed_integral
        amplitude = np.clip(raw, -self.limiter.absolute_limit_m_s2, self.limiter.absolute_limit_m_s2)
        candidate = self.limiter.previous + np.clip(amplitude - self.limiter.previous, -self.limiter.slew_limit_m_s2_per_update, self.limiter.slew_limit_m_s2_per_update)
        blocked = (np.abs(raw - candidate) > 1.0e-12) & (np.sign(position_error) == np.sign(raw - candidate))
        self.integral = np.where(blocked, self.integral, proposed_integral)
        return self._finish(-self.kp * position_error - self.kd * velocity_error - self.ki * self.integral, np.linalg.norm(observation.full_state_error), self.integral)


class V3FullStateLQR(_V3ControllerBase):
    """20D full-state discrete LQR with a common 3D limiter."""

    def __init__(self, gain: np.ndarray) -> None:
        super().__init__()
        self.gain = np.asarray(gain, dtype=float).reshape(3, 20).copy()
        if not np.isfinite(self.gain).all():
            raise ValueError("Full-State LQR gain must be finite")

    def command(self, observation: V3Observation, reference: V3Reference, dt: float = 0.05) -> np.ndarray:
        del reference, dt
        return self._finish(-self.gain @ observation.full_state_error, np.linalg.norm(observation.full_state_error), np.zeros(3))


class V3TaskWeightedLQR(V3FullStateLQR):
    """Full-state LQR whose Q matrix is weighted by measured cutter outputs."""
