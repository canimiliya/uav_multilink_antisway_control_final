"""Fair five-link adaptation of Kang and Shan's FAS-DOB outer loop."""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from uav_sway.v3.controllers import V3ControllerDiagnostics, _V3ControllerBase
from uav_sway.v3.observation import V3Observation, V3Reference


def signed_sqrt(value: np.ndarray) -> np.ndarray:
    value = np.asarray(value, dtype=float)
    return np.sign(value) * np.sqrt(np.abs(value))


@dataclass(frozen=True)
class Kang2026Diagnostics(V3ControllerDiagnostics):
    equivalent_swing_displacement: np.ndarray
    equivalent_swing_velocity: np.ndarray
    virtual_constraint: np.ndarray
    fas_coordinate: np.ndarray
    observer_innovation: np.ndarray
    disturbance_hat: np.ndarray
    nominal_fas_input: np.ndarray
    observer_substeps: int
    solve_time_ms: float
    limiter_mismatch: float


class Kang2026FASDOB(_V3ControllerBase):
    """Reduced-order FAS, virtual constraint, and causal finite-time DOB.

    Paper Eqs. (10)-(20) are retained structurally.  The point-load cable
    displacement is replaced by the measured UAV-to-cutter equivalent swing
    displacement.  The paper's desired-force output is normalized to the
    benchmark's common world-frame acceleration authority; the common
    geometric inner loop remains outside this class.
    """

    def __init__(self, parameters: dict) -> None:
        super().__init__()
        self.parameters = dict(parameters)
        self.axis_scale = np.asarray(parameters["axis_scale"], dtype=float).reshape(3)
        if np.any(self.axis_scale <= 0.0) or not np.isfinite(self.axis_scale).all():
            raise ValueError("axis_scale must be positive and finite")
        self.reset()

    def reset(self) -> None:
        super().reset()
        self.observer_state = np.zeros(3)
        self.disturbance_hat = np.zeros(3)
        self.previous_nominal = np.zeros(3)
        self.observer_initialized = False
        z3 = np.zeros(3)
        self.diagnostics = Kang2026Diagnostics(
            z3, z3, z3, np.zeros(3, dtype=bool), np.zeros(3, dtype=bool), 0.0, z3,
            z3, z3, z3, z3, z3, z3, z3, int(self.parameters["observer_substeps"]), 0.0, 0.0,
        )

    def _observer_update(self, coordinate: np.ndarray, dt: float) -> np.ndarray:
        if not self.observer_initialized:
            self.observer_state = coordinate.copy()
            self.observer_initialized = True
        substeps = int(self.parameters["observer_substeps"])
        h = float(dt) / substeps
        lambda1 = float(self.parameters["observer_lambda1"])
        lambda2 = float(self.parameters["observer_lambda2"])
        rho = float(self.parameters["observer_rho"])
        clip = float(self.parameters["observer_clip_m_s2"])
        for _ in range(substeps):
            innovation = self.observer_state - coordinate
            z_dot = -lambda1 * np.sqrt(rho) * signed_sqrt(innovation) + self.previous_nominal + self.disturbance_hat
            d_dot = -lambda2 * rho * np.sign(innovation)
            self.observer_state += h * z_dot
            self.disturbance_hat = np.clip(self.disturbance_hat + h * d_dot, -clip, clip)
        return self.observer_state - coordinate

    def command(self, observation: V3Observation, reference: V3Reference, dt: float = 0.05) -> np.ndarray:
        if abs(float(dt) - 0.05) > 1.0e-12:
            raise ValueError("KANG2026 adaptation requires the frozen 20 Hz outer rate")
        started = time.perf_counter_ns()
        uav_error = np.asarray(observation.uav_position_world - reference.uav_position_world, dtype=float)
        uav_velocity = np.asarray(observation.uav_velocity_world - reference.uav_velocity_world, dtype=float)
        tip_error = np.asarray(observation.task_state.tip_position_world - reference.tip_position_world, dtype=float)
        tip_velocity = np.asarray(observation.task_state.tip_velocity_world - reference.uav_velocity_world, dtype=float)
        swing_displacement = tip_error - uav_error
        swing_velocity = tip_velocity - uav_velocity

        k1 = float(self.parameters["position_gain"]) * self.axis_scale
        k2 = float(self.parameters["velocity_gain"]) * self.axis_scale
        virtual_scale = float(self.parameters["virtual_constraint_gain"]) * float(self.parameters["virtual_length_scale"])
        coupled_error = uav_error + virtual_scale * swing_displacement
        coupled_velocity = uav_velocity + virtual_scale * swing_velocity
        virtual_constraint = k1 * coupled_error
        virtual_constraint_rate = k1 * coupled_velocity
        coordinate = uav_velocity + virtual_constraint

        innovation = self._observer_update(coordinate, float(dt))
        nominal = -k2 * coordinate - virtual_constraint_rate
        compensation = float(self.parameters["observer_compensation_scale"]) * self.disturbance_hat
        raw = nominal - compensation
        amplitude = np.clip(raw, -2.0, 2.0)
        final = self.limiter.limit(raw).as_array()
        self.previous_nominal = final.copy()
        self.diagnostics = Kang2026Diagnostics(
            raw.copy(), amplitude.copy(), final.copy(), np.abs(raw) > 2.0 + 1.0e-12,
            np.abs(final - amplitude) > 1.0e-12, float(np.linalg.norm(observation.full_state_error)), np.zeros(3),
            swing_displacement.copy(), swing_velocity.copy(), virtual_constraint.copy(), coordinate.copy(),
            innovation.copy(), self.disturbance_hat.copy(), nominal.copy(), int(self.parameters["observer_substeps"]),
            (time.perf_counter_ns() - started) / 1.0e6, float(np.max(np.abs(final - raw))),
        )
        return final.copy()

