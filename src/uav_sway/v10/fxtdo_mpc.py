"""Acceleration-level adaptation of Xu et al. FxTDO-MPC (arXiv:2408.15019)."""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from uav_sway.mpc.osqp_solver import OSQPPreviewSolver
from uav_sway.mpc.qp_builder import QPData
from uav_sway.v3.controllers import V3ControllerDiagnostics, _V3ControllerBase
from uav_sway.v3.observation import V3Observation, V3Reference


def _vector_signed_power(value: np.ndarray, power: float, boundary: float) -> np.ndarray:
    """Multivariable signed power: ||e||^(power-1) e with a causal boundary."""
    value = np.asarray(value, dtype=float).reshape(3)
    norm = float(np.linalg.norm(value))
    if norm <= boundary:
        if power == 0.0:
            return value / boundary
        return value * boundary ** (power - 1.0)
    return value * norm ** (power - 1.0)


def _phi(error: np.ndarray, parameters: dict, which: int) -> np.ndarray:
    d_inf = float(parameters["observer_d_infinity"])
    if which == 1:
        powers = (0.5, 1.0, 1.0 / (1.0 - d_inf))
    elif which == 2:
        powers = (0.0, 1.0, (1.0 + d_inf) / (1.0 - d_inf))
    else:
        raise ValueError(which)
    coefficients = (
        float(parameters[f"observer_k{which}"]),
        float(parameters[f"observer_k{which}_prime"]),
        float(parameters[f"observer_k{which}_double_prime"]),
    )
    boundary = float(parameters["observer_boundary"])
    return sum(coefficient * _vector_signed_power(error, power, boundary) for coefficient, power in zip(coefficients, powers))


def build_fxtdo_mpc_qp(
    a: np.ndarray,
    b: np.ndarray,
    c_task: np.ndarray,
    state: np.ndarray,
    previous_command: np.ndarray,
    disturbance_hat: np.ndarray,
    parameters: dict,
) -> QPData:
    """Condense Eqs. (23)-(27), holding FxTDO disturbance constant in-horizon."""
    a = np.asarray(a, dtype=float).reshape(20, 20)
    b = np.asarray(b, dtype=float).reshape(20, 3)
    c_task = np.asarray(c_task, dtype=float).reshape(12, 20)
    state = np.asarray(state, dtype=float).reshape(20)
    previous = np.asarray(previous_command, dtype=float).reshape(3)
    disturbance = float(parameters["disturbance_compensation_scale"]) * np.asarray(disturbance_hat, dtype=float).reshape(3)
    horizon = int(parameters["horizon_updates"])
    variables = 3 * horizon
    affine = np.zeros((horizon + 1, 20))
    maps = np.zeros((horizon + 1, 20, variables))
    affine[0] = state
    for step in range(horizon):
        affine[step + 1] = a @ affine[step] + b @ disturbance
        maps[step + 1] = a @ maps[step]
        maps[step + 1, :, 3 * step:3 * step + 3] += b
    output_weights = np.asarray(
        [float(parameters["position_weight"])] * 3
        + [float(parameters["velocity_weight"])] * 3
        + [float(parameters["orientation_weight"])] * 3
        + [float(parameters["angular_weight"])] * 3
    )
    weighted_output = np.diag(output_weights)
    pmat = np.zeros((variables, variables))
    qvec = np.zeros(variables)
    input_weight = float(parameters["input_weight"])
    terminal = float(parameters["terminal_multiplier"])
    for step in range(1, horizon + 1):
        output_affine = c_task @ affine[step]
        output_map = c_task @ maps[step]
        multiplier = terminal if step == horizon else 1.0
        pmat += 2.0 * multiplier * output_map.T @ weighted_output @ output_map
        qvec += 2.0 * multiplier * output_map.T @ weighted_output @ output_affine
    pmat += 2.0 * input_weight * np.eye(variables)
    rows: list[np.ndarray] = []
    lower: list[float] = []
    upper: list[float] = []
    rate_weight = float(parameters["rate_weight"])
    for step in range(horizon):
        for axis in range(3):
            input_row = np.zeros(variables)
            input_row[3 * step + axis] = 1.0
            rows.append(input_row.copy())
            lower.append(-2.0)
            upper.append(2.0)
            rate_row = input_row.copy()
            if step == 0:
                rate_affine = -previous[axis]
            else:
                rate_row[3 * (step - 1) + axis] = -1.0
                rate_affine = 0.0
            rows.append(rate_row.copy())
            lower.append(-0.25 - rate_affine)
            upper.append(0.25 - rate_affine)
            pmat += 2.0 * rate_weight * np.outer(rate_row, rate_row)
            qvec += 2.0 * rate_weight * rate_affine * rate_row
    regularization = float(parameters["regularization"])
    pmat = 0.5 * (pmat + pmat.T) + regularization * np.eye(variables)
    return QPData(pmat, qvec, np.asarray(rows), np.asarray(lower), np.asarray(upper), affine, maps)


@dataclass(frozen=True)
class FxTDOMPCDiagnostics(V3ControllerDiagnostics):
    observer_state_hat: np.ndarray
    disturbance_hat: np.ndarray
    observer_error: np.ndarray
    phi_1: np.ndarray
    phi_2: np.ndarray
    qp_status: str
    qp_iterations: int
    solve_time_ms: float
    limiter_mismatch: float
    observer_stable: bool


class XuFxTDOMPC(_V3ControllerBase):
    """Paper-identity-preserving 3D FxTDO plus constrained 20D task MPC."""

    def __init__(self, a: np.ndarray, b: np.ndarray, c_task: np.ndarray, parameters: dict) -> None:
        super().__init__()
        self.a = np.asarray(a, dtype=float).reshape(20, 20)
        self.b = np.asarray(b, dtype=float).reshape(20, 3)
        self.c_task = np.asarray(c_task, dtype=float).reshape(12, 20)
        self.parameters = dict(parameters)
        self._validate_parameters()
        self.solver = OSQPPreviewSolver(eps_abs=1.0e-5, eps_rel=1.0e-5, max_iter=4000, warm_start=True)
        self.reset()

    def _validate_parameters(self) -> None:
        if int(self.parameters["horizon_updates"]) < 1:
            raise ValueError("MPC horizon must be positive")
        if not 0.0 < float(self.parameters["observer_d_infinity"]) < 1.0:
            raise ValueError("paper requires 0 < d_infinity < 1")
        positive = [
            "position_weight", "velocity_weight", "orientation_weight", "angular_weight",
            "input_weight", "rate_weight", "terminal_multiplier", "observer_l1", "observer_l2",
            "observer_k1", "observer_k1_prime", "observer_k1_double_prime", "observer_k2",
            "observer_k2_prime", "observer_k2_double_prime", "observer_boundary",
            "observer_clip_m_s2", "observer_substeps", "regularization",
        ]
        values = np.asarray([float(self.parameters[name]) for name in positive])
        if not np.isfinite(values).all() or np.any(values <= 0.0):
            raise ValueError("FxTDO-MPC parameters must be positive and finite")

    def _empty_diagnostics(self) -> None:
        z3 = np.zeros(3)
        self.diagnostics = FxTDOMPCDiagnostics(
            z3, z3, z3, np.zeros(3, dtype=bool), np.zeros(3, dtype=bool), 0.0, z3,
            z3, z3, z3, z3, z3, "not_run", 0, 0.0, 0.0, True,
        )

    def reset(self) -> None:
        super().reset()
        # Reset the numerical warm start as well as the physical/controller state.
        self.solver = OSQPPreviewSolver(eps_abs=1.0e-5, eps_rel=1.0e-5, max_iter=4000, warm_start=True)
        self.observer_state_hat = np.zeros(3)
        self.disturbance_hat = np.zeros(3)
        self.previous_actual_command = np.zeros(3)
        self.observer_initialized = False
        self._empty_diagnostics()

    def _update_observer(self, measured_velocity: np.ndarray, dt: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        velocity = np.asarray(measured_velocity, dtype=float).reshape(3)
        if not self.observer_initialized:
            self.observer_state_hat = velocity.copy()
            self.observer_initialized = True
        substeps = int(self.parameters["observer_substeps"])
        sub_dt = dt / substeps
        phi_1 = np.zeros(3)
        phi_2 = np.zeros(3)
        for _ in range(substeps):
            error = velocity - self.observer_state_hat
            phi_1 = _phi(error, self.parameters, 1)
            phi_2 = _phi(error, self.parameters, 2)
            self.observer_state_hat += sub_dt * (
                self.disturbance_hat + self.previous_actual_command
                + float(self.parameters["observer_l1"]) * phi_1
            )
            self.disturbance_hat += sub_dt * float(self.parameters["observer_l2"]) * phi_2
            clip = float(self.parameters["observer_clip_m_s2"])
            self.disturbance_hat = np.clip(self.disturbance_hat, -clip, clip)
        return velocity - self.observer_state_hat, phi_1, phi_2

    def command(self, observation: V3Observation, reference: V3Reference, dt: float = 0.05) -> np.ndarray:
        del reference
        dt = float(dt)
        if abs(dt - 0.05) > 1.0e-12:
            raise ValueError("V10 FxTDO-MPC requires the frozen 20 Hz outer rate")
        started = time.perf_counter_ns()
        error, phi_1, phi_2 = self._update_observer(observation.uav_velocity_world, dt)
        qp = build_fxtdo_mpc_qp(
            self.a, self.b, self.c_task, observation.full_state_error,
            self.previous_actual_command, self.disturbance_hat, self.parameters,
        )
        solution, info = self.solver.solve(qp)
        raw = np.asarray(solution[:3], dtype=float)
        amplitude = np.clip(raw, -2.0, 2.0)
        final = self.limiter.limit(raw).as_array()
        self.previous_actual_command = final.copy()
        stable = bool(np.isfinite(np.r_[self.observer_state_hat, self.disturbance_hat, error, phi_1, phi_2]).all())
        elapsed_ms = (time.perf_counter_ns() - started) / 1.0e6
        self.diagnostics = FxTDOMPCDiagnostics(
            raw.copy(), amplitude.copy(), final.copy(), np.abs(raw) > 2.0 + 1.0e-12,
            np.abs(final - amplitude) > 1.0e-12, float(np.linalg.norm(observation.full_state_error)), np.zeros(3),
            self.observer_state_hat.copy(), self.disturbance_hat.copy(), error.copy(), phi_1.copy(), phi_2.copy(),
            str(info.status), int(info.iter), elapsed_ms, float(np.max(np.abs(final - raw))), stable,
        )
        return final.copy()
