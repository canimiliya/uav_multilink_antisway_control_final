"""Constraint-aware residual-trust offset-free MPC for V4.

The controller uses only the measured error state, the current/past reference,
and the previously applied limited acceleration.  The frozen task-LQR gain is
the nominal backbone.  A bounded steady-target problem and a physical-command
QP prevent the V3 residual-to-equilibrium saturation feedback mechanism.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product
import time

import numpy as np

from uav_sway.mpc.osqp_solver import OSQPPreviewSolver
from uav_sway.mpc.qp_builder import QPData
from uav_sway.v3.controllers import V3ControllerDiagnostics, _V3ControllerBase
from uav_sway.v3.dr_tsrmpc import controllability_basis
from uav_sway.v3.observation import V3Observation, V3Reference


@dataclass(frozen=True)
class CARTOFMPCDiagnostics(V3ControllerDiagnostics):
    residual_raw: np.ndarray
    residual_clipped: np.ndarray
    residual_represented: np.ndarray
    residual_unrepresented: np.ndarray
    residual_debt: np.ndarray
    residual_raw_norm: float
    residual_clipped_norm: float
    residual_represented_norm: float
    residual_unrepresented_norm: float
    residual_debt_norm: float
    residual_clip_fraction: float
    trust: float
    requested_steady_input: np.ndarray
    feasible_steady_input: np.ndarray
    steady_equality_residual: float
    steady_task_residual: float
    steady_feasible: bool
    steady_constraint_active: bool
    reachable_constraint_active: bool
    backbone_command: np.ndarray
    qp_correction: np.ndarray
    qp_status: str
    qp_iterations: int
    solve_time_ms: float
    limiter_mismatch: float


@dataclass(frozen=True)
class FeasibleSteadyTarget:
    state: np.ndarray
    command: np.ndarray
    requested_command: np.ndarray
    unrepresented: np.ndarray
    equality_residual: float
    task_residual: float
    feasible: bool
    constraint_active: bool
    reachable_active: bool


class CausalResidualEstimator:
    """One-step residual estimator driven by the previous actual command."""

    def __init__(self, a: np.ndarray, b: np.ndarray, beta: float, component_limit: float) -> None:
        self.a = np.asarray(a, dtype=float).reshape(20, 20)
        self.b = np.asarray(b, dtype=float).reshape(20, 3)
        self.beta = float(beta)
        self.component_limit = float(component_limit)
        if not 0.0 < self.beta <= 1.0 or self.component_limit <= 0.0:
            raise ValueError("invalid residual-estimator parameters")
        self.basis = controllability_basis(self.a, self.b)
        self.reset()

    def reset(self) -> None:
        self.previous_state: np.ndarray | None = None
        self.previous_reference: np.ndarray | None = None
        self.previous_actual_command = np.zeros(3)
        self.filtered = np.zeros(20)

    def update(self, state: np.ndarray, reference: V3Reference) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        state = np.asarray(state, dtype=float).reshape(20)
        reference_position = np.asarray(reference.uav_position_world, dtype=float).reshape(3)
        if self.previous_state is None:
            self.previous_state = state.copy()
            self.previous_reference = reference_position.copy()
            return np.zeros(20), np.zeros(20), np.zeros(20), 0.0
        reference_delta = reference_position - self.previous_reference
        reference_shift = np.zeros(20)
        reference_shift[[0, 2, 4]] = reference_delta
        prediction = self.a @ self.previous_state + self.b @ self.previous_actual_command - reference_shift
        raw = state - prediction
        clipped = np.clip(raw, -self.component_limit, self.component_limit)
        self.filtered = (1.0 - self.beta) * self.filtered + self.beta * clipped
        represented = self.basis @ (self.basis.T @ self.filtered)
        clip_fraction = float(np.linalg.norm(raw - clipped) / max(np.linalg.norm(raw), 1.0e-12))
        self.previous_state = state.copy()
        self.previous_reference = reference_position.copy()
        return raw, clipped, represented, clip_fraction

    def accept(self, command: np.ndarray) -> None:
        command = np.asarray(command, dtype=float).reshape(3)
        if not np.isfinite(command).all():
            raise ValueError("actual command must be finite")
        self.previous_actual_command = command.copy()


def _box_qp(hessian: np.ndarray, gradient: np.ndarray, lower: np.ndarray, upper: np.ndarray) -> np.ndarray:
    """Solve a strictly convex three-variable box QP by active-set enumeration."""
    hessian = np.asarray(hessian, dtype=float).reshape(3, 3)
    gradient = np.asarray(gradient, dtype=float).reshape(3)
    lower = np.asarray(lower, dtype=float).reshape(3)
    upper = np.asarray(upper, dtype=float).reshape(3)
    best: np.ndarray | None = None
    best_value = float("inf")
    for status in product((-1, 0, 1), repeat=3):
        active = [index for index, value in enumerate(status) if value]
        free = [index for index, value in enumerate(status) if not value]
        candidate = np.zeros(3)
        for index in active:
            candidate[index] = lower[index] if status[index] < 0 else upper[index]
        if free:
            rhs = -gradient[free]
            if active:
                rhs -= hessian[np.ix_(free, active)] @ candidate[active]
            candidate[free] = np.linalg.solve(hessian[np.ix_(free, free)], rhs)
        if np.any(candidate < lower - 1.0e-10) or np.any(candidate > upper + 1.0e-10):
            continue
        value = float(0.5 * candidate @ hessian @ candidate + gradient @ candidate)
        if value < best_value:
            best = candidate.copy()
            best_value = value
    if best is None:
        raise RuntimeError("feasible steady-target box QP unexpectedly failed")
    return np.clip(best, lower, upper)


class ConstraintFeasibleSteadySolver:
    """Soft residual balance with acceleration and finite-slew reachability."""

    def __init__(self, a: np.ndarray, b: np.ndarray, c_task: np.ndarray, parameters: dict) -> None:
        self.e = np.eye(20) - np.asarray(a, dtype=float).reshape(20, 20)
        self.b = np.asarray(b, dtype=float).reshape(20, 3)
        self.c_task = np.asarray(c_task, dtype=float).reshape(12, 20)
        self.eq_weight = float(parameters["steady_equality_weight"])
        self.input_weight = float(parameters["steady_input_weight"])
        self.state_regularization = float(parameters["steady_state_regularization"])
        self.reach_updates = int(parameters["steady_reach_updates"])
        self.feasibility_tolerance = float(parameters["steady_feasibility_tolerance"])
        task_weights = np.asarray(parameters["steady_task_weights"], dtype=float).reshape(12)
        weighted_task = np.sqrt(task_weights)[:, None] * self.c_task
        design = np.vstack((
            np.sqrt(self.eq_weight) * self.e,
            weighted_task,
            np.sqrt(self.state_regularization) * np.eye(20),
        ))
        rhs_d = np.vstack((np.sqrt(self.eq_weight) * np.eye(20), np.zeros((32, 20))))
        rhs_u = np.vstack((np.sqrt(self.eq_weight) * self.b, np.zeros((32, 3))))
        design_pinv = np.linalg.pinv(design)
        self.x_from_d = design_pinv @ rhs_d
        self.x_from_u = design_pinv @ rhs_u
        residual_d = design @ self.x_from_d - rhs_d
        residual_u = design @ self.x_from_u - rhs_u
        self.hessian = residual_u.T @ residual_u + self.input_weight * np.eye(3)
        self.gradient_map = residual_u.T @ residual_d

    def solve(self, residual: np.ndarray, previous_command: np.ndarray) -> FeasibleSteadyTarget:
        residual = np.asarray(residual, dtype=float).reshape(20)
        previous = np.asarray(previous_command, dtype=float).reshape(3)
        gradient = self.gradient_map @ residual
        requested = -np.linalg.solve(self.hessian, gradient)
        reach = 0.25 * self.reach_updates
        lower = np.maximum(-2.0, previous - reach)
        upper = np.minimum(2.0, previous + reach)
        command = _box_qp(self.hessian, gradient, lower, upper)
        state = self.x_from_d @ residual + self.x_from_u @ command
        unrepresented = residual - (self.e @ state - self.b @ command)
        equality_residual = float(np.linalg.norm(unrepresented))
        task_residual = float(np.linalg.norm(self.c_task[:3] @ state))
        amplitude_active = bool(np.any(np.abs(command) >= 2.0 - 1.0e-8))
        reachable_active = bool(np.any(np.isclose(command, lower, atol=1.0e-8) | np.isclose(command, upper, atol=1.0e-8)))
        normalized = equality_residual / max(float(np.linalg.norm(residual)), 1.0e-9)
        return FeasibleSteadyTarget(
            state=state,
            command=command,
            requested_command=requested,
            unrepresented=unrepresented,
            equality_residual=equality_residual,
            task_residual=task_residual,
            feasible=bool(normalized <= self.feasibility_tolerance),
            constraint_active=amplitude_active,
            reachable_active=reachable_active,
        )


def build_cart_qp(
    a: np.ndarray,
    b: np.ndarray,
    c_task: np.ndarray,
    gain: np.ndarray,
    state: np.ndarray,
    steady: FeasibleSteadyTarget,
    previous_command: np.ndarray,
    horizon: int,
    weights: tuple[float, float, float, float, float, float],
) -> QPData:
    """Condense the predictive correction and constrain every physical move."""
    h = int(horizon)
    variables = 3 * h
    acl = a - b @ gain
    affine = np.zeros((h + 1, 20))
    maps = np.zeros((h + 1, 20, variables))
    affine[0] = state - steady.state
    # The unrepresented steady residual is an explicit model-error term.
    for k in range(h):
        affine[k + 1] = acl @ affine[k] + steady.unrepresented
        maps[k + 1] = acl @ maps[k]
        maps[k + 1, :, 3 * k:3 * k + 3] += b
    position_w, velocity_w, orientation_w, angular_w, correction_w, rate_w = weights
    output_weight = np.diag([position_w] * 3 + [velocity_w] * 3 + [orientation_w] * 3 + [angular_w] * 3)
    pmat = np.zeros((variables, variables))
    qvec = np.zeros(variables)
    for k in range(1, h + 1):
        output_affine = c_task @ (steady.state + affine[k])
        output_map = c_task @ maps[k]
        multiplier = 2.0 if k == h else 1.0
        pmat += 2.0 * multiplier * output_map.T @ output_weight @ output_map
        qvec += 2.0 * multiplier * output_map.T @ output_weight @ output_affine
    physical_affine = np.zeros((h, 3))
    physical_map = np.zeros((h, 3, variables))
    for k in range(h):
        physical_affine[k] = steady.command - gain @ affine[k]
        physical_map[k] = -gain @ maps[k]
        physical_map[k, :, 3 * k:3 * k + 3] += np.eye(3)
    rate_affine = np.zeros((h, 3))
    rate_map = np.zeros((h, 3, variables))
    rate_affine[0] = physical_affine[0] - previous_command
    rate_map[0] = physical_map[0]
    for k in range(1, h):
        rate_affine[k] = physical_affine[k] - physical_affine[k - 1]
        rate_map[k] = physical_map[k] - physical_map[k - 1]
    pmat += 2.0 * correction_w * np.eye(variables)
    for k in range(h):
        pmat += 2.0 * rate_w * rate_map[k].T @ rate_map[k]
        qvec += 2.0 * rate_w * rate_map[k].T @ rate_affine[k]
    rows: list[np.ndarray] = []
    lower: list[float] = []
    upper: list[float] = []
    for k in range(h):
        for axis in range(3):
            rows.append(physical_map[k, axis].copy())
            lower.append(-2.0 - physical_affine[k, axis])
            upper.append(2.0 - physical_affine[k, axis])
            rows.append(rate_map[k, axis].copy())
            lower.append(-0.25 - rate_affine[k, axis])
            upper.append(0.25 - rate_affine[k, axis])
    pmat = 0.5 * (pmat + pmat.T) + 1.0e-8 * np.eye(variables)
    return QPData(pmat, qvec, np.asarray(rows), np.asarray(lower), np.asarray(upper), affine, maps)


class CARTOFMPC(_V3ControllerBase):
    """CART-OFMPC around the immutable task_lqr_009 nominal gain."""

    def __init__(self, a: np.ndarray, b: np.ndarray, c_task: np.ndarray, gain: np.ndarray, parameters: dict) -> None:
        super().__init__()
        self.a = np.asarray(a, dtype=float).reshape(20, 20)
        self.b = np.asarray(b, dtype=float).reshape(20, 3)
        self.c_task = np.asarray(c_task, dtype=float).reshape(12, 20)
        self.gain = np.asarray(gain, dtype=float).reshape(3, 20)
        self.parameters = dict(parameters)
        self.horizon = int(parameters["horizon_updates"])
        self.estimator = CausalResidualEstimator(self.a, self.b, float(parameters["residual_beta"]), float(parameters["residual_component_limit"]))
        self.steady_solver = ConstraintFeasibleSteadySolver(self.a, self.b, self.c_task, parameters)
        self.solver = OSQPPreviewSolver(eps_abs=1.0e-5, eps_rel=1.0e-5, max_iter=4000, warm_start=True)
        self.debt = np.zeros(20)
        self.trust = 1.0
        self._empty_diagnostics()

    def _empty_diagnostics(self) -> None:
        z3 = np.zeros(3); z20 = np.zeros(20)
        self.diagnostics = CARTOFMPCDiagnostics(
            z3, z3, z3, np.zeros(3, dtype=bool), np.zeros(3, dtype=bool), 0.0, z3,
            z20, z20, z20, z20, z20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            z3, z3, 0.0, 0.0, True, False, False, z3, z3,
            "not_run", 0, 0.0, 0.0,
        )

    def reset(self) -> None:
        super().reset()
        self.estimator.reset()
        self.debt = np.zeros(20)
        self.trust = 1.0
        self._empty_diagnostics()

    def command(self, observation: V3Observation, reference: V3Reference, dt: float = 0.05) -> np.ndarray:
        if abs(float(dt) - 0.05) > 1.0e-12:
            raise ValueError("CART-OFMPC requires the frozen 0.05 s outer period")
        started = time.perf_counter_ns()
        state = np.asarray(observation.full_state_error, dtype=float)
        raw, clipped, projected, clip_fraction = self.estimator.update(state, reference)
        debt_feedback = float(self.parameters["debt_feedback"])
        residual_for_steady = projected - debt_feedback * self.debt
        trial = self.steady_solver.solve(residual_for_steady, self.limiter.previous)
        normalized_infeasibility = trial.equality_residual / max(float(np.linalg.norm(residual_for_steady)), 1.0e-9)
        debt_ratio = float(np.linalg.norm(self.debt) / max(float(self.parameters["debt_limit"]), 1.0e-9))
        trust_target = np.exp(
            -float(self.parameters["trust_clip_scale"]) * clip_fraction
            -float(self.parameters["trust_feasibility_scale"]) * normalized_infeasibility
            -float(self.parameters["trust_debt_scale"]) * debt_ratio
        )
        trust_target = float(np.clip(trust_target, float(self.parameters["trust_floor"]), 1.0))
        trust_beta = float(self.parameters["trust_beta"])
        self.trust = float((1.0 - trust_beta) * self.trust + trust_beta * trust_target)
        steady = self.steady_solver.solve(self.trust * residual_for_steady, self.limiter.previous)
        debt_decay = float(self.parameters["debt_decay"])
        antiwindup_gain = float(self.parameters["antiwindup_gain"])
        self.debt = (1.0 - debt_decay) * self.debt + antiwindup_gain * steady.unrepresented
        debt_limit = float(self.parameters["debt_limit"])
        self.debt = np.clip(self.debt, -debt_limit, debt_limit)
        weights = (
            float(self.parameters["task_position_weight"]),
            float(self.parameters["task_velocity_weight"]),
            float(self.parameters["orientation_weight"]),
            float(self.parameters["angular_velocity_weight"]),
            float(self.parameters["correction_weight"]),
            float(self.parameters["rate_weight"]),
        )
        qp = build_cart_qp(self.a, self.b, self.c_task, self.gain, state, steady, self.limiter.previous, self.horizon, weights)
        correction, info = self.solver.solve(qp)
        backbone = -self.gain @ state
        raw_command = steady.command - self.gain @ (state - steady.state) + correction[:3]
        amplitude = np.clip(raw_command, -2.0, 2.0)
        limited = self.limiter.limit(raw_command).as_array()
        mismatch = float(np.max(np.abs(limited - raw_command)))
        self.estimator.accept(limited)
        self.diagnostics = CARTOFMPCDiagnostics(
            raw_command.copy(), amplitude.copy(), limited.copy(), np.abs(raw_command) > 2.0 + 1.0e-8,
            np.abs(limited - amplitude) > 1.0e-8, float(np.linalg.norm(state)), self.debt[:3].copy(),
            raw.copy(), clipped.copy(), projected.copy(), steady.unrepresented.copy(), self.debt.copy(),
            float(np.linalg.norm(raw)), float(np.linalg.norm(clipped)), float(np.linalg.norm(projected)),
            float(np.linalg.norm(steady.unrepresented)), float(np.linalg.norm(self.debt)), clip_fraction, self.trust,
            steady.requested_command.copy(), steady.command.copy(), steady.equality_residual, steady.task_residual,
            steady.feasible, steady.constraint_active, steady.reachable_active, backbone.copy(), correction[:3].copy(),
            str(info.status), int(info.iter), (time.perf_counter_ns() - started) / 1.0e6, mismatch,
        )
        return limited
