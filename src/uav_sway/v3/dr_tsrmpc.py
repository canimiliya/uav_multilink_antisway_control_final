"""Full-3D dynamic-residual task-space residual MPC for V3-R2."""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from uav_sway.mpc.osqp_solver import OSQPPreviewSolver
from uav_sway.mpc.qp_builder import QPData

from .controllers import V3ControllerDiagnostics, _V3ControllerBase
from .observation import V3Observation, V3Reference


@dataclass(frozen=True)
class V3DRTSRMPCDiagnostics(V3ControllerDiagnostics):
    d_raw: np.ndarray
    d_hat: np.ndarray
    d_projected: np.ndarray
    d_raw_norm: float
    d_hat_norm: float
    d_projected_norm: float
    d_rejected_norm: float
    steady_state_residual: float
    steady_task_residual: float
    steady_input: np.ndarray
    qp_status: str
    qp_iterations: int
    solve_time_ms: float
    limiter_mismatch: float


@dataclass(frozen=True)
class SteadyCompensation:
    state: np.ndarray
    command: np.ndarray
    equality_residual: float
    task_residual: float


def controllability_basis(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=float).reshape(20, 20)
    b = np.asarray(b, dtype=float).reshape(20, 3)
    matrix = np.column_stack([np.linalg.matrix_power(a, k) @ b for k in range(20)])
    u, _, _ = np.linalg.svd(matrix, full_matrices=False)
    # Match the rank convention used by the frozen R0 linear-model audit.
    rank = int(np.linalg.matrix_rank(matrix, tol=1.0e-10))
    return u[:, :rank]


class CausalDynamicResidual:
    """One-step causal residual estimator using the previous actual command."""

    def __init__(self, a: np.ndarray, b: np.ndarray, beta: float, component_limit: float) -> None:
        self.a = np.asarray(a, dtype=float).reshape(20, 20)
        self.b = np.asarray(b, dtype=float).reshape(20, 3)
        self.beta = float(beta)
        self.component_limit = float(component_limit)
        if not 0.0 < self.beta <= 1.0 or self.component_limit <= 0.0:
            raise ValueError("residual estimator parameters must be positive and finite")
        self.basis = controllability_basis(self.a, self.b)
        self.reset()

    def reset(self) -> None:
        self.previous_state: np.ndarray | None = None
        self.previous_reference: np.ndarray | None = None
        self.previous_actual_command = np.zeros(3, dtype=float)
        self.filtered = np.zeros(20, dtype=float)

    def update(self, state: np.ndarray, reference: V3Reference) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        state = np.asarray(state, dtype=float).reshape(20)
        reference_position = np.asarray(reference.uav_position_world, dtype=float).reshape(3)
        if self.previous_state is None:
            self.previous_state = state.copy()
            self.previous_reference = reference_position.copy()
            return np.zeros(20), self.filtered.copy(), self.filtered.copy()
        reference_delta = reference_position - self.previous_reference
        reference_shift = np.zeros(20, dtype=float)
        reference_shift[[0, 2, 4]] = reference_delta
        prediction = self.a @ self.previous_state + self.b @ self.previous_actual_command - reference_shift
        raw = state - prediction
        bounded = np.clip(raw, -self.component_limit, self.component_limit)
        self.filtered = (1.0 - self.beta) * self.filtered + self.beta * bounded
        projected = self.basis @ (self.basis.T @ self.filtered)
        self.previous_state = state.copy()
        self.previous_reference = reference_position.copy()
        return raw, self.filtered.copy(), projected

    def accept(self, actual_command: np.ndarray) -> None:
        value = np.asarray(actual_command, dtype=float).reshape(3)
        if not np.isfinite(value).all():
            raise ValueError("actual command must be finite")
        self.previous_actual_command = value.copy()


class SteadyCompensator:
    """Exact residual-compatible equilibrium with task-optimal nullspace."""

    def __init__(self, a: np.ndarray, b: np.ndarray, c_task: np.ndarray) -> None:
        self.a = np.asarray(a, dtype=float).reshape(20, 20)
        self.b = np.asarray(b, dtype=float).reshape(20, 3)
        self.c_task = np.asarray(c_task, dtype=float).reshape(12, 20)
        self.equality = np.column_stack((np.eye(20) - self.a, -self.b))
        u, singular, vh = np.linalg.svd(self.equality, full_matrices=True)
        tolerance = np.finfo(float).eps * max(self.equality.shape) * singular[0]
        rank = int(np.sum(singular > tolerance))
        if rank != 20:
            raise ValueError("steady-state equality must have full row rank")
        self.pinv = np.linalg.pinv(self.equality)
        self.null = vh[rank:].T
        task_weight = np.diag([1.0] * 3 + [0.15] * 3 + [0.05] * 3 + [0.02] * 3)
        selector = np.zeros((3, 23), dtype=float)
        selector[:, 20:] = np.eye(3)
        state_task = np.zeros((12, 23), dtype=float)
        state_task[:, :20] = self.c_task
        regularizer = 1.0e-7 * np.eye(23)
        self.weight = state_task.T @ task_weight @ state_task + 0.02 * selector.T @ selector + regularizer

    def solve(self, residual: np.ndarray) -> SteadyCompensation:
        residual = np.asarray(residual, dtype=float).reshape(20)
        particular = self.pinv @ residual
        reduced_hessian = self.null.T @ self.weight @ self.null
        reduced_linear = self.null.T @ self.weight @ particular
        alpha = -np.linalg.solve(reduced_hessian, reduced_linear)
        solution = particular + self.null @ alpha
        state = solution[:20]
        command = solution[20:]
        # The estimator is bounded so this should be inactive. Scaling both
        # state and command preserves the exact equilibrium if it is needed.
        scale = min(1.0, 1.9 / max(float(np.max(np.abs(command))), 1.0e-12))
        state = scale * state
        command = scale * command
        represented_residual = scale * residual
        equality_residual = float(np.linalg.norm((np.eye(20) - self.a) @ state - self.b @ command - represented_residual))
        return SteadyCompensation(
            state=state,
            command=command,
            equality_residual=equality_residual,
            task_residual=float(np.linalg.norm(self.c_task[:3] @ state)),
        )


def build_qp(
    a: np.ndarray,
    b: np.ndarray,
    c_task: np.ndarray,
    gain: np.ndarray,
    state: np.ndarray,
    steady: SteadyCompensation,
    previous_command: np.ndarray,
    horizon: int,
    position_weight: float,
    velocity_weight: float,
    orientation_weight: float,
    angular_velocity_weight: float,
    correction_weight: float,
    rate_weight: float,
) -> QPData:
    """Condense the residual MPC and constrain predicted physical commands."""
    a = np.asarray(a, dtype=float).reshape(20, 20)
    b = np.asarray(b, dtype=float).reshape(20, 3)
    c_task = np.asarray(c_task, dtype=float).reshape(12, 20)
    gain = np.asarray(gain, dtype=float).reshape(3, 20)
    state = np.asarray(state, dtype=float).reshape(20)
    previous_command = np.asarray(previous_command, dtype=float).reshape(3)
    h = int(horizon)
    variables = 3 * h
    acl = a - b @ gain
    affine = np.zeros((h + 1, 20), dtype=float)
    maps = np.zeros((h + 1, 20, variables), dtype=float)
    affine[0] = state - steady.state
    for k in range(h):
        affine[k + 1] = acl @ affine[k]
        maps[k + 1] = acl @ maps[k]
        maps[k + 1, :, 3 * k:3 * k + 3] += b
    output_weight = np.diag(
        [position_weight] * 3
        + [velocity_weight] * 3
        + [orientation_weight] * 3
        + [angular_velocity_weight] * 3
    )
    pmat = np.zeros((variables, variables), dtype=float)
    qvec = np.zeros(variables, dtype=float)
    for k in range(1, h + 1):
        output_affine = c_task @ (steady.state + affine[k])
        output_map = c_task @ maps[k]
        multiplier = 2.0 if k == h else 1.0
        pmat += 2.0 * multiplier * output_map.T @ output_weight @ output_map
        qvec += 2.0 * multiplier * output_map.T @ output_weight @ output_affine
    physical_affine = np.zeros((h, 3), dtype=float)
    physical_map = np.zeros((h, 3, variables), dtype=float)
    for k in range(h):
        physical_affine[k] = steady.command - gain @ affine[k]
        physical_map[k] = -gain @ maps[k]
        physical_map[k, :, 3 * k:3 * k + 3] += np.eye(3)
    rate_affine = np.zeros((h, 3), dtype=float)
    rate_map = np.zeros((h, 3, variables), dtype=float)
    rate_affine[0] = physical_affine[0] - previous_command
    rate_map[0] = physical_map[0]
    for k in range(1, h):
        rate_affine[k] = physical_affine[k] - physical_affine[k - 1]
        rate_map[k] = physical_map[k] - physical_map[k - 1]
    pmat += 2.0 * float(correction_weight) * np.eye(variables)
    for k in range(h):
        pmat += 2.0 * float(rate_weight) * rate_map[k].T @ rate_map[k]
        qvec += 2.0 * float(rate_weight) * rate_map[k].T @ rate_affine[k]
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
    pmat = 0.5 * (pmat + pmat.T) + 1.0e-9 * np.eye(variables)
    return QPData(pmat, qvec, np.asarray(rows), np.asarray(lower), np.asarray(upper), affine, maps)


class V3DRTSRMPC(_V3ControllerBase):
    """Causal 20D/3-input DR-TSRMPC around a frozen LQR backbone."""

    def __init__(self, a: np.ndarray, b: np.ndarray, c_task: np.ndarray, gain: np.ndarray, parameters: dict) -> None:
        super().__init__()
        self.a = np.asarray(a, dtype=float).reshape(20, 20)
        self.b = np.asarray(b, dtype=float).reshape(20, 3)
        self.c_task = np.asarray(c_task, dtype=float).reshape(12, 20)
        self.gain = np.asarray(gain, dtype=float).reshape(3, 20)
        self.parameters = dict(parameters)
        if not all(np.isfinite(value).all() for value in (self.a, self.b, self.c_task, self.gain)):
            raise ValueError("DR-TSRMPC arrays must be finite")
        self.horizon = int(parameters["horizon_updates"])
        self.residual_enabled = bool(parameters.get("residual_enabled", True))
        self.predictive_enabled = bool(parameters.get("predictive_enabled", True))
        self.estimator = CausalDynamicResidual(
            self.a, self.b, float(parameters["residual_beta"]), float(parameters["residual_clip_norm"])
        )
        self.steady = SteadyCompensator(self.a, self.b, self.c_task)
        self.solver = OSQPPreviewSolver(eps_abs=1.0e-5, eps_rel=1.0e-5, max_iter=4000, warm_start=True)
        self._set_empty_diagnostics()

    def _set_empty_diagnostics(self) -> None:
        zero3 = np.zeros(3)
        zero20 = np.zeros(20)
        self.diagnostics = V3DRTSRMPCDiagnostics(
            zero3, zero3, zero3, np.zeros(3, dtype=bool), np.zeros(3, dtype=bool), 0.0, zero3,
            zero20, zero20, zero20, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, zero3,
            "not_run", 0, 0.0, 0.0,
        )

    def reset(self) -> None:
        super().reset()
        self.estimator.reset()
        self._set_empty_diagnostics()

    def command(self, observation: V3Observation, reference: V3Reference, dt: float = 0.05) -> np.ndarray:
        if abs(float(dt) - 0.05) > 1.0e-12:
            raise ValueError("V3 DR-TSRMPC requires the frozen 0.05 s outer period")
        started = time.perf_counter_ns()
        state = observation.full_state_error
        raw_residual, filtered_residual, projected_residual = self.estimator.update(state, reference)
        if not self.residual_enabled:
            projected_residual = np.zeros(20, dtype=float)
        steady = self.steady.solve(projected_residual)
        if self.predictive_enabled:
            qp = build_qp(
                self.a, self.b, self.c_task, self.gain, state, steady, self.limiter.previous,
                self.horizon, float(self.parameters["task_position_weight"]),
                float(self.parameters["task_velocity_weight"]), float(self.parameters["orientation_weight"]),
                float(self.parameters["angular_velocity_weight"]), float(self.parameters["residual_correction_weight"]),
                float(self.parameters["command_rate_weight"]),
            )
            correction, info = self.solver.solve(qp)
            qp_status = str(info.status)
            qp_iterations = int(info.iter)
        else:
            correction = np.zeros(3 * self.horizon, dtype=float)
            qp_status = "not_run_residual_only_ablation"
            qp_iterations = 0
        raw_command = steady.command - self.gain @ (state - steady.state) + correction[:3]
        amplitude = np.clip(raw_command, -2.0, 2.0)
        limited = self.limiter.limit(raw_command).as_array()
        mismatch = float(np.max(np.abs(limited - raw_command)))
        self.estimator.accept(limited)
        self.diagnostics = V3DRTSRMPCDiagnostics(
            raw_command.copy(), amplitude.copy(), limited.copy(), np.abs(raw_command) > 2.0 + 1.0e-9,
            np.abs(limited - amplitude) > 1.0e-9, float(np.linalg.norm(state)), np.zeros(3),
            raw_residual.copy(), filtered_residual.copy(), projected_residual.copy(),
            float(np.linalg.norm(raw_residual)), float(np.linalg.norm(filtered_residual)),
            float(np.linalg.norm(projected_residual)), float(np.linalg.norm(filtered_residual - projected_residual)),
            steady.equality_residual, steady.task_residual, steady.command.copy(), qp_status, qp_iterations,
            (time.perf_counter_ns() - started) / 1.0e6, mismatch,
        )
        return limited
