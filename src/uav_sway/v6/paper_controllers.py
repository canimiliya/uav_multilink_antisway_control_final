"""Faithful, fair five-link adaptations of the frozen V6 Paper suite."""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from uav_sway.mpc.osqp_solver import OSQPPreviewSolver
from uav_sway.mpc.qp_builder import QPData
from uav_sway.v3.controllers import V3ControllerDiagnostics, _V3ControllerBase
from uav_sway.v3.observation import V3Observation, V3Reference


def signed_power(value: np.ndarray, exponent: float) -> np.ndarray:
    value = np.asarray(value, dtype=float)
    return np.sign(value) * np.abs(value) ** float(exponent)


@dataclass(frozen=True)
class Yu2026Diagnostics(V3ControllerDiagnostics):
    disturbance_hat: np.ndarray
    position_innovation: np.ndarray
    velocity_innovation: np.ndarray
    swing_displacement: np.ndarray
    swing_velocity: np.ndarray
    finite_time_exponent: float
    solve_time_ms: float
    limiter_mismatch: float


class Yu2026FiniteTimeCFO(_V3ControllerBase):
    """Eq. (15)/(28)/(49) mapped to the five-link tip task.

    The measured cutter tip and its relative motion replace the original
    point-load cable coordinates.  The CFO remains an internal causal observer;
    no wind truth or future sample enters this controller.
    """

    def __init__(self, parameters: dict) -> None:
        super().__init__()
        self.parameters = dict(parameters)
        self.reset()

    def reset(self) -> None:
        super().reset()
        self.z1_hat = np.zeros(3)
        self.z2_hat = np.zeros(3)
        self.z0 = np.zeros(3)
        z3 = np.zeros(3)
        self.diagnostics = Yu2026Diagnostics(
            z3, z3, z3, np.zeros(3, dtype=bool), np.zeros(3, dtype=bool), 0.0, z3,
            z3, z3, z3, z3, z3, 2.0 * float(self.parameters["finite_time_power_r"]) - 1.0, 0.0, 0.0,
        )

    def command(self, observation: V3Observation, reference: V3Reference, dt: float = 0.05) -> np.ndarray:
        if abs(float(dt) - 0.05) > 1.0e-12:
            raise ValueError("YU2026 adaptation requires the frozen 20 Hz outer rate")
        started = time.perf_counter_ns()
        tip_error = np.asarray(observation.task_state.tip_position_world - reference.tip_position_world, dtype=float)
        tip_velocity = np.asarray(observation.task_state.tip_velocity_world - reference.uav_velocity_world, dtype=float)
        uav_error = np.asarray(observation.uav_position_world - reference.uav_position_world, dtype=float)
        uav_velocity = np.asarray(observation.uav_velocity_world - reference.uav_velocity_world, dtype=float)
        swing_displacement = tip_error - uav_error
        swing_velocity = tip_velocity - uav_velocity

        lambda1 = tip_error - self.z1_hat
        lambda2 = tip_velocity - self.z2_hat
        n1 = float(self.parameters["observer_n1"])
        n2 = float(self.parameters["observer_n2"])
        eta1 = float(self.parameters["observer_eta1"])
        eta2 = float(self.parameters["observer_eta2"])
        disturbance_hat = self.z0 + n1 * lambda1 + n2 * lambda2
        disturbance_hat = np.clip(
            disturbance_hat,
            -float(self.parameters["observer_clip_m_s2"]),
            float(self.parameters["observer_clip_m_s2"]),
        )
        # Forward-Euler realization of Eq. (15), using the previously applied
        # acceleration as the known nominal second-order channel.
        self.z1_hat += float(dt) * self.z2_hat
        self.z2_hat += float(dt) * (self.limiter.previous + disturbance_hat)
        self.z0 += float(dt) * (eta1 * n1 * lambda1 + eta2 * n2 * lambda2)

        exponent = 2.0 * float(self.parameters["finite_time_power_r"]) - 1.0
        raw = (
            -float(self.parameters["position_gain"]) * signed_power(tip_error, exponent)
            -float(self.parameters["velocity_gain"]) * signed_power(tip_velocity, exponent)
            -float(self.parameters["observer_scale"]) * disturbance_hat
            -float(self.parameters["swing_position_gain"]) * swing_displacement
            -float(self.parameters["swing_velocity_gain"]) * swing_velocity
        )
        amplitude = np.clip(raw, -2.0, 2.0)
        final = self.limiter.limit(raw).as_array()
        self.diagnostics = Yu2026Diagnostics(
            raw.copy(), amplitude.copy(), final.copy(), np.abs(raw) > 2.0 + 1.0e-12,
            np.abs(final - amplitude) > 1.0e-12, float(np.linalg.norm(observation.full_state_error)), np.zeros(3),
            disturbance_hat.copy(), lambda1.copy(), lambda2.copy(), swing_displacement.copy(), swing_velocity.copy(),
            exponent, (time.perf_counter_ns() - started) / 1.0e6, float(np.max(np.abs(final - raw))),
        )
        return final.copy()


def build_direct_mpc_qp(
    a: np.ndarray,
    b: np.ndarray,
    c_task: np.ndarray,
    state: np.ndarray,
    previous: np.ndarray,
    parameters: dict,
) -> QPData:
    """Condense Eq. (7a)-(7c) with the frozen linear five-link model."""
    horizon = int(parameters["horizon_updates"])
    variables = 3 * horizon
    affine = np.zeros((horizon + 1, 20))
    maps = np.zeros((horizon + 1, 20, variables))
    affine[0] = np.asarray(state, dtype=float).reshape(20)
    for k in range(horizon):
        affine[k + 1] = a @ affine[k]
        maps[k + 1] = a @ maps[k]
        maps[k + 1, :, 3 * k:3 * k + 3] += b
    weights = np.asarray([
        *([float(parameters["position_weight"])] * 3),
        *([float(parameters["velocity_weight"])] * 3),
        *([float(parameters["orientation_weight"])] * 3),
        *([float(parameters["angular_weight"])] * 3),
    ])
    output_weight = np.diag(weights)
    pmat = 2.0 * float(parameters["input_weight"]) * np.eye(variables)
    qvec = np.zeros(variables)
    for k in range(1, horizon + 1):
        output_affine = c_task @ affine[k]
        output_map = c_task @ maps[k]
        multiplier = 2.0 if k == horizon else 1.0
        pmat += 2.0 * multiplier * output_map.T @ output_weight @ output_map
        qvec += 2.0 * multiplier * output_map.T @ output_weight @ output_affine
    rows = []
    lower = []
    upper = []
    rate_weight = float(parameters["rate_weight"])
    for k in range(horizon):
        for axis in range(3):
            direct = np.zeros(variables)
            direct[3 * k + axis] = 1.0
            rows.append(direct.copy())
            lower.append(-2.0)
            upper.append(2.0)
            rate = direct.copy()
            rate_affine = -float(previous[axis]) if k == 0 else 0.0
            if k > 0:
                rate[3 * (k - 1) + axis] = -1.0
            rows.append(rate.copy())
            lower.append(-0.25 - rate_affine)
            upper.append(0.25 - rate_affine)
            pmat += 2.0 * rate_weight * np.outer(rate, rate)
            qvec += 2.0 * rate_weight * rate_affine * rate
    pmat = 0.5 * (pmat + pmat.T) + 1.0e-8 * np.eye(variables)
    return QPData(pmat, qvec, np.asarray(rows), np.asarray(lower), np.asarray(upper), affine, maps)


@dataclass(frozen=True)
class SEP2026Diagnostics(V3ControllerDiagnostics):
    shaped_input_before_filter: np.ndarray
    shaped_input_after_filter: np.ndarray
    collocated_velocity: np.ndarray
    passivity_residual_before: float
    passivity_residual_after: float
    passivity_projection_fraction: float
    qp_status: str
    qp_iterations: int
    solve_time_ms: float
    limiter_mismatch: float


class SEP2026PassivityNMPC(_V3ControllerBase):
    """Eq. (7)-(10) adapted to the frozen 20-state, three-input plant."""

    def __init__(self, a: np.ndarray, b: np.ndarray, c_task: np.ndarray, parameters: dict) -> None:
        super().__init__()
        self.a = np.asarray(a, dtype=float).reshape(20, 20)
        self.b = np.asarray(b, dtype=float).reshape(20, 3)
        self.c_task = np.asarray(c_task, dtype=float).reshape(12, 20)
        self.parameters = dict(parameters)
        self.solver = OSQPPreviewSolver(eps_abs=1.0e-5, eps_rel=1.0e-5, max_iter=4000, warm_start=True)
        self.reset()

    def reset(self) -> None:
        super().reset()
        z3 = np.zeros(3)
        self.diagnostics = SEP2026Diagnostics(
            z3, z3, z3, np.zeros(3, dtype=bool), np.zeros(3, dtype=bool), 0.0, z3,
            z3, z3, z3, 0.0, 0.0, 0.0, "not_run", 0, 0.0, 0.0,
        )

    def _residual(self, shaped: np.ndarray, velocity: np.ndarray) -> float:
        rho = float(self.parameters["passivity_rho"])
        epsilon = float(self.parameters["passivity_epsilon"])
        return float(shaped @ velocity + rho * (velocity @ velocity) + epsilon * (shaped @ shaped))

    def _passivity_projection(self, shaped: np.ndarray, velocity: np.ndarray) -> tuple[np.ndarray, float]:
        if self._residual(shaped, velocity) <= 0.0:
            return shaped.copy(), 0.0
        if float(np.linalg.norm(velocity)) <= 1.0e-10:
            return np.zeros(3), 1.0
        fallback = -0.60 * velocity
        if self._residual(fallback, velocity) > 1.0e-10:
            fallback = -velocity
        low = 0.0
        high = 1.0
        for _ in range(48):
            middle = 0.5 * (low + high)
            trial = fallback + middle * (shaped - fallback)
            if self._residual(trial, velocity) <= 0.0:
                low = middle
            else:
                high = middle
        return fallback + low * (shaped - fallback), float(1.0 - low)

    def command(self, observation: V3Observation, reference: V3Reference, dt: float = 0.05) -> np.ndarray:
        if abs(float(dt) - 0.05) > 1.0e-12:
            raise ValueError("SEP2026 adaptation requires the frozen 20 Hz outer rate")
        started = time.perf_counter_ns()
        state = np.asarray(observation.full_state_error, dtype=float)
        previous = self.limiter.previous.copy()
        qp = build_direct_mpc_qp(self.a, self.b, self.c_task, state, previous, self.parameters)
        solution, info = self.solver.solve(qp)
        nominal = np.asarray(solution[:3], dtype=float)
        position_error = np.asarray(observation.task_state.tip_position_world - reference.tip_position_world, dtype=float)
        velocity = np.asarray(observation.task_state.tip_velocity_world - reference.uav_velocity_world, dtype=float)
        storage = float(self.parameters["storage_position_gain"])
        shaped_before = nominal + storage * position_error
        residual_before = self._residual(shaped_before, velocity)
        shaped_after, fraction = self._passivity_projection(shaped_before, velocity)
        raw = shaped_after - storage * position_error
        amplitude = np.clip(raw, -2.0, 2.0)
        final = self.limiter.limit(raw).as_array()
        applied_shaped = final + storage * position_error
        residual_after = self._residual(applied_shaped, velocity)
        self.diagnostics = SEP2026Diagnostics(
            raw.copy(), amplitude.copy(), final.copy(), np.abs(raw) > 2.0 + 1.0e-12,
            np.abs(final - amplitude) > 1.0e-12, float(np.linalg.norm(state)), np.zeros(3),
            shaped_before.copy(), applied_shaped.copy(), velocity.copy(), residual_before, residual_after, fraction,
            str(info.status), int(info.iter), (time.perf_counter_ns() - started) / 1.0e6,
            float(np.max(np.abs(final - raw))),
        )
        return final.copy()
