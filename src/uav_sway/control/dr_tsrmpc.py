"""Dynamic-residual task-space residual MPC for the V2 Self route.

This module is the implementation frozen by V2-R3R2.  The disturbance model
is a causal 16-state one-step prediction residual.  It is absorbed by an
exact constrained steady-state optimizer; the MPC then predicts only the
deviation dynamics and therefore never counts the residual twice.
"""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from uav_sway.control.acceleration_limiter import AccelerationLimiter
from uav_sway.mpc.osqp_solver import OSQPPreviewSolver
from uav_sway.mpc.preview_model import reference_vector
from uav_sway.mpc.qp_builder import QPData


AX_MIN = -2.0
AX_MAX = 2.0
AX_SLEW = 0.25
HORIZON = 20
DT = 0.05
WSS = np.diag([1.0, 0.25, 0.25, 0.0625])
REGULARIZATION = 1.0e-8
BETAS = (0.05, 0.15, 0.30)
POSITION_WEIGHTS = (40.0, 80.0, 160.0)
ORIENTATION_WEIGHTS = (5.0, 20.0)
RESIDUAL_WEIGHTS = (0.5, 2.0)


def _as_model_arrays(a, b, c_task, gain, p_task):
    arrays = (
        np.asarray(a, dtype=float).reshape(16, 16),
        np.asarray(b, dtype=float).reshape(16, 1),
        np.asarray(c_task, dtype=float).reshape(4, 16),
        np.asarray(gain, dtype=float).reshape(1, 16),
        np.asarray(p_task, dtype=float).reshape(16, 16),
    )
    if not all(np.isfinite(value).all() for value in arrays):
        raise ValueError("DR-TSRMPC model arrays must be finite")
    if not np.allclose(arrays[4], arrays[4].T, atol=1.0e-10):
        raise ValueError("Task-LQR terminal P must be symmetric")
    if float(np.min(np.linalg.eigvalsh((arrays[4] + arrays[4].T) / 2.0))) <= 0.0:
        raise ValueError("Task-LQR terminal P must be positive definite")
    return arrays


def _reference_shift(a, previous_reference, current_reference) -> np.ndarray:
    return reference_vector(current_reference) - np.asarray(a, dtype=float) @ reference_vector(previous_reference)


@dataclass(frozen=True)
class ResidualUpdate:
    d_raw: np.ndarray
    d_hat: np.ndarray
    x_pred: np.ndarray | None
    first_sample: bool


class DynamicResidualEstimator:
    """Causal estimator using the previous measured state and actual ax."""

    def __init__(self, a, b, beta: float):
        self.a = np.asarray(a, dtype=float).reshape(16, 16)
        self.b = np.asarray(b, dtype=float).reshape(16, 1)
        if float(beta) not in BETAS:
            raise ValueError("beta is outside the frozen DR grid")
        self.beta = float(beta)
        self.reset()

    def reset(self) -> None:
        self.x_prev: np.ndarray | None = None
        self.reference_prev = None
        self.previous_actual_ax = 0.0
        self.d_hat = np.zeros(16, dtype=float)

    def update(self, x_current, reference_current) -> ResidualUpdate:
        x_current = np.asarray(x_current, dtype=float).reshape(16)
        if not np.isfinite(x_current).all():
            raise ValueError("current state must be finite")
        if self.x_prev is None:
            self.x_prev = x_current.copy()
            self.reference_prev = reference_current
            return ResidualUpdate(np.zeros(16), self.d_hat.copy(), None, True)
        c_ref = _reference_shift(self.a, self.reference_prev, reference_current)
        x_pred = self.a @ self.x_prev + self.b[:, 0] * self.previous_actual_ax - c_ref
        d_raw = x_current - x_pred
        self.d_hat = (1.0 - self.beta) * self.d_hat + self.beta * d_raw
        self.x_prev = x_current.copy()
        self.reference_prev = reference_current
        return ResidualUpdate(d_raw.copy(), self.d_hat.copy(), x_pred.copy(), False)

    def accept_actual_command(self, actual_ax: float) -> None:
        if not np.isfinite(actual_ax):
            raise ValueError("actual ax must be finite")
        self.previous_actual_ax = float(actual_ax)


@dataclass(frozen=True)
class ControllabilityProjection:
    d_controllable: np.ndarray
    d_uncontrollable: np.ndarray
    controllability_rank: int
    projection_residual: float


def project_residual(a, b, d_hat) -> ControllabilityProjection:
    """Diagnostic-only orthogonal projection onto span(C(A,B))."""
    a = np.asarray(a, dtype=float).reshape(16, 16)
    b = np.asarray(b, dtype=float).reshape(16, 1)
    d_hat = np.asarray(d_hat, dtype=float).reshape(16)
    controllability = np.column_stack([np.linalg.matrix_power(a, k) @ b[:, 0] for k in range(16)])
    u, singular_values, _ = np.linalg.svd(controllability, full_matrices=True)
    rank = int(np.linalg.matrix_rank(controllability))
    del singular_values
    basis = u[:, :rank]
    d_controllable = basis @ (basis.T @ d_hat) if rank else np.zeros(16)
    d_uncontrollable = d_hat - d_controllable
    return ControllabilityProjection(
        d_controllable=d_controllable,
        d_uncontrollable=d_uncontrollable,
        controllability_rank=rank,
        projection_residual=float(np.linalg.norm(d_hat - d_controllable - d_uncontrollable)),
    )


@dataclass(frozen=True)
class SteadyStateResult:
    x_s: np.ndarray
    u_s: float
    equality_residual: float
    task_residual: float
    x_s_norm: float
    objective: float
    bound_active: bool


def solve_steady_state(a, b, c_task, d_hat, u_limit: float = AX_MAX,
                       regularization: float = REGULARIZATION) -> SteadyStateResult:
    """Solve the exact constrained steady-state problem.

    The equality is parameterized by a least-squares particular solution and
    the one-dimensional nullspace of ``[(I-A), -B]``.  Clipping is performed
    on the nullspace coordinate, so the equality remains satisfied after the
    input bound is applied.
    """
    a, b, c_task, _, _ = _as_model_arrays(a, b, c_task, np.zeros((1, 16)), np.eye(16))
    d_hat = np.asarray(d_hat, dtype=float).reshape(16)
    if not np.isfinite(d_hat).all() or regularization != REGULARIZATION:
        raise ValueError("invalid DR steady-state inputs")
    equality = np.column_stack((np.eye(16) - a, -b[:, 0]))
    u_svd, singular_values, vh = np.linalg.svd(equality, full_matrices=True)
    rank = int(np.sum(singular_values > np.finfo(float).eps * max(equality.shape) * singular_values[0]))
    if rank != 16:
        raise ValueError("frozen steady-state equality is not full row rank")
    particular = np.linalg.lstsq(equality, d_hat, rcond=None)[0]
    null_basis = vh[rank:].T
    if null_basis.shape != (17, 1):
        raise ValueError("frozen steady-state equality must have one-dimensional nullspace")
    null = null_basis[:, 0]
    task_map = c_task.T @ WSS @ c_task
    hessian_scalar = float(null[:16] @ task_map @ null[:16] + regularization * (null @ null))
    linear_scalar = float(null[:16] @ task_map @ particular[:16] + regularization * (null @ particular))
    alpha = -linear_scalar / hessian_scalar
    input_particular = float(particular[16])
    input_null = float(null[16])
    if abs(input_null) <= 1.0e-10:
        if abs(input_particular) > float(u_limit) + 1.0e-10:
            raise ValueError("steady-state equality has no feasible bounded solution")
        alpha_bound = alpha
    else:
        lo = (-float(u_limit) - input_particular) / input_null
        hi = (float(u_limit) - input_particular) / input_null
        lo, hi = min(lo, hi), max(lo, hi)
        alpha_bound = float(np.clip(alpha, lo, hi))
    solution = particular + null * alpha_bound
    x_s = solution[:16]
    u_s = float(solution[16])
    equality_residual = float(np.linalg.norm((np.eye(16) - a) @ x_s - b[:, 0] * u_s - d_hat))
    task_residual = float(np.linalg.norm(c_task @ x_s))
    objective = float((c_task @ x_s) @ WSS @ (c_task @ x_s) + regularization * (x_s @ x_s + u_s * u_s))
    return SteadyStateResult(
        x_s=x_s.copy(), u_s=u_s, equality_residual=equality_residual,
        task_residual=task_residual, x_s_norm=float(np.linalg.norm(x_s)),
        objective=objective, bound_active=bool(abs(abs(u_s) - float(u_limit)) <= 1.0e-8),
    )


def build_dr_tsrmpc_qp(a, b, c_task, gain, p_task, x0, x_s, u_s,
                        w_p, w_theta, residual_r, previous_actual_ax,
                        horizon: int = HORIZON, ax_min: float = AX_MIN,
                        ax_max: float = AX_MAX, slew_limit: float = AX_SLEW) -> QPData:
    """Build the H=20 residual-input QP with physical command constraints."""
    a, b, c_task, gain, p_task = _as_model_arrays(a, b, c_task, gain, p_task)
    x0 = np.asarray(x0, dtype=float).reshape(16)
    x_s = np.asarray(x_s, dtype=float).reshape(16)
    if int(horizon) != HORIZON:
        raise ValueError("DR-TSRMPC freezes H=20")
    if float(residual_r) not in RESIDUAL_WEIGHTS:
        raise ValueError("R is outside the frozen grid")
    h = HORIZON
    xi0 = x0 - x_s
    a_cl = a - b @ gain
    f = np.zeros((h + 1, 16), dtype=float)
    g = np.zeros((h + 1, 16, h), dtype=float)
    f[0] = xi0
    for k in range(h):
        f[k + 1] = a_cl @ f[k]
        g[k + 1] = a_cl @ g[k]
        g[k + 1, :, k] += b[:, 0]
    weight = np.diag([float(w_p), 0.25 * float(w_p), float(w_theta), 0.25 * float(w_theta)])
    pmat = np.zeros((h, h), dtype=float)
    qvec = np.zeros(h, dtype=float)
    for k in range(1, h):
        affine_y = c_task @ (x_s + f[k])
        map_y = c_task @ g[k]
        pmat += 2.0 * (map_y.T @ weight @ map_y)
        qvec += 2.0 * (map_y.T @ weight @ affine_y)
    pmat += 2.0 * (g[h].T @ p_task @ g[h])
    qvec += 2.0 * (g[h].T @ p_task @ f[h])

    physical_affine = np.zeros(h, dtype=float)
    physical_map = np.zeros((h, h), dtype=float)
    for k in range(h):
        physical_affine[k] = float(u_s - (gain @ f[k])[0])
        physical_map[k] = (-gain @ g[k]).reshape(h)
        physical_map[k, k] += 1.0
    rate_maps = np.zeros((h, h), dtype=float)
    rate_affine = np.zeros(h, dtype=float)
    rate_maps[0] = physical_map[0]
    rate_affine[0] = physical_affine[0] - float(previous_actual_ax)
    for k in range(1, h):
        rate_maps[k] = physical_map[k] - physical_map[k - 1]
        rate_affine[k] = physical_affine[k] - physical_affine[k - 1]
    pmat += 2.0 * float(residual_r) * np.eye(h)
    pmat += 2.0 * float(residual_r) * (rate_maps.T @ rate_maps)
    qvec += 2.0 * float(residual_r) * (rate_maps.T @ rate_affine)
    pmat = (pmat + pmat.T) / 2.0
    rows, lower, upper = [], [], []
    for k in range(h):
        rows.append(physical_map[k].copy())
        lower.append(float(ax_min - physical_affine[k]))
        upper.append(float(ax_max - physical_affine[k]))
        rows.append(rate_maps[k].copy())
        lower.append(float(-slew_limit - rate_affine[k]))
        upper.append(float(slew_limit - rate_affine[k]))
    return QPData(pmat, qvec, np.asarray(rows), np.asarray(lower), np.asarray(upper), f, g)


def enumerate_grid() -> list[dict]:
    rows = []
    index = 0
    for beta in BETAS:
        for w_p in POSITION_WEIGHTS:
            for w_theta in ORIENTATION_WEIGHTS:
                for residual_r in RESIDUAL_WEIGHTS:
                    rows.append({"candidate_id": f"dr_tsrmpc_{index:03d}", "beta": beta,
                                 "w_p": w_p, "w_theta": w_theta, "R": residual_r, "H": HORIZON})
                    index += 1
    return rows


@dataclass(frozen=True)
class DRTSRMPCDiagnostics:
    d_raw: np.ndarray
    d_hat_vector: np.ndarray
    d_hat_norm: float
    d_controllable_norm: float
    d_uncontrollable_norm: float
    controllability_rank: int
    x_s_norm: float
    u_s: float
    steady_state_residual: float
    steady_task_residual: float
    u_s_bound_active: bool
    v_mpc: float
    ax_cmd_raw: float
    ax_cmd_amplitude_limited: float
    ax_cmd_limited: float
    ax_saturated: bool
    ax_slew_limited: bool
    qp_status: str
    qp_iterations: int
    steady_solve_ms: float
    mpc_solve_ms: float
    total_solve_ms: float
    limiter_mismatch: float

    @property
    def d_hat(self) -> float:
        """Legacy scalar access is intentionally unavailable."""
        raise AttributeError("DR-TSRMPC d_hat is a 16D vector; use d_hat_vector")


class DRTSRMPC:
    """Causal DR-TSRMPC controller for the x channel."""

    def __init__(self, a, b, c_task, gain, p_task, beta, w_p, w_theta, residual_r,
                 solver: OSQPPreviewSolver | None = None):
        self.a, self.b, self.c_task, self.gain, self.p_task = _as_model_arrays(a, b, c_task, gain, p_task)
        if float(beta) not in BETAS or float(w_p) not in POSITION_WEIGHTS or float(w_theta) not in ORIENTATION_WEIGHTS or float(residual_r) not in RESIDUAL_WEIGHTS:
            raise ValueError("DR-TSRMPC parameters are outside the frozen grid")
        self.beta, self.w_p, self.w_theta, self.residual_r = map(float, (beta, w_p, w_theta, residual_r))
        self.solver = solver or OSQPPreviewSolver(eps_abs=1e-5, eps_rel=1e-5, max_iter=4000, warm_start=True)
        self.estimator = DynamicResidualEstimator(self.a, self.b, self.beta)
        self.limiter = AccelerationLimiter(AX_MIN, AX_MAX, AX_SLEW)
        self.diagnostics = DRTSRMPCDiagnostics(np.zeros(16), np.zeros(16), 0.0, 0.0, 0.0, 13, 0.0, 0.0, 0.0, 0.0, False, 0.0, 0.0, 0.0, 0.0, False, False, "not_run", 0, 0.0, 0.0, 0.0, 0.0)

    def reset(self, state=None, reference=None):
        del state, reference
        self.estimator.reset()
        self.limiter.reset(0.0)

    def command(self, x, reference_current) -> float:
        started = time.perf_counter_ns()
        x = np.asarray(x, dtype=float).reshape(16)
        update = self.estimator.update(x, reference_current)
        projection = project_residual(self.a, self.b, update.d_hat)
        steady_started = time.perf_counter_ns()
        steady = solve_steady_state(self.a, self.b, self.c_task, update.d_hat)
        steady_ms = (time.perf_counter_ns() - steady_started) / 1.0e6
        qp = build_dr_tsrmpc_qp(self.a, self.b, self.c_task, self.gain, self.p_task, x, steady.x_s, steady.u_s,
                                 self.w_p, self.w_theta, self.residual_r, self.estimator.previous_actual_ax)
        mpc_started = time.perf_counter_ns()
        v, info = self.solver.solve(qp)
        mpc_ms = (time.perf_counter_ns() - mpc_started) / 1.0e6
        v0 = float(v[0])
        raw = float(steady.u_s - (self.gain @ (x - steady.x_s))[0, 0] + v0)
        limited = float(self.limiter.limit(raw))
        mismatch = abs(limited - raw)
        self.estimator.accept_actual_command(limited)
        limit_diag = self.limiter.diagnostics
        self.diagnostics = DRTSRMPCDiagnostics(
            update.d_raw.copy(), update.d_hat.copy(), float(np.linalg.norm(update.d_hat)),
            float(np.linalg.norm(projection.d_controllable)), float(np.linalg.norm(projection.d_uncontrollable)),
            projection.controllability_rank, steady.x_s_norm, steady.u_s, steady.equality_residual,
            steady.task_residual, steady.bound_active, v0, raw, float(limit_diag.amplitude_limited), limited,
            bool(limit_diag.saturated), bool(limit_diag.slew_limited), str(info.status), int(info.iter),
            steady_ms, mpc_ms, (time.perf_counter_ns() - started) / 1.0e6, mismatch,
        )
        return limited


def backbone_command(gain, x) -> float:
    return float((-np.asarray(gain, dtype=float).reshape(1, 16) @ np.asarray(x, dtype=float).reshape(16, 1))[0, 0])
