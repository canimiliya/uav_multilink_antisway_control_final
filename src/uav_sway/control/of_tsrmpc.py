"""Offset-free task-space residual MPC for the V2 advanced-self route.

The module deliberately contains only the x-channel method.  It does not
estimate wind and it does not alter the shared y/z controller.
"""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from uav_sway.control.acceleration_limiter import AccelerationLimiter
from uav_sway.mpc.osqp_solver import OSQPPreviewSolver
from uav_sway.mpc.qp_builder import QPData


AX_MIN = -2.0
AX_MAX = 2.0
AX_SLEW = 0.25
HORIZON = 20
DT = 0.05


@dataclass(frozen=True)
class SteadyStateResult:
    x_s: np.ndarray
    u_s: float
    residual: float
    objective: float


@dataclass(frozen=True)
class OFTSRMPCDiagnostics:
    lqr_feedback_ax: float
    ax_reference_feedforward: float
    ax_cmd_raw: float
    ax_cmd_amplitude_limited: float
    ax_cmd_limited: float
    ax_saturated: bool
    ax_slew_limited: bool
    lqr_state_norm: float
    d_raw: float
    d_hat: float
    x_s_norm: float
    u_s: float
    steady_state_residual: float
    v_mpc: float
    qp_status: str
    qp_iterations: int
    steady_solve_ms: float
    residual_solve_ms: float
    total_solve_ms: float
    limiter_mismatch: float


def _as_model_arrays(a, b, c_task, gain, p_task):
    a = np.asarray(a, dtype=float).reshape(16, 16)
    b = np.asarray(b, dtype=float).reshape(16, 1)
    c_task = np.asarray(c_task, dtype=float).reshape(4, 16)
    gain = np.asarray(gain, dtype=float).reshape(1, 16)
    p_task = np.asarray(p_task, dtype=float).reshape(16, 16)
    arrays = (a, b, c_task, gain, p_task)
    if not all(np.isfinite(item).all() for item in arrays):
        raise ValueError("OF-TSRMPC model arrays must be finite")
    if not np.allclose(p_task, p_task.T, atol=1e-10):
        raise ValueError("Task-LQR terminal P must be symmetric")
    if float(np.min(np.linalg.eigvalsh(p_task))) <= 0.0:
        raise ValueError("Task-LQR terminal P must be positive definite")
    return arrays


def solve_steady_state(a, b, c_pos, d_hat, u_limit=2.0, regularization=1e-8) -> SteadyStateResult:
    """Solve the preregistered output-bias steady-state problem.

    A is stable, so ``(I-A)`` is nonsingular for the frozen model.  The
    deterministic minimum-norm tie-break is the only numerical regularizer.
    """
    a = np.asarray(a, dtype=float).reshape(16, 16)
    b = np.asarray(b, dtype=float).reshape(16, 1)
    c_pos = np.asarray(c_pos, dtype=float).reshape(16)
    if regularization != 1e-8:
        raise ValueError("only the preregistered 1e-8 regularization is allowed")
    # Parameterize the exact steady-state equality with the nullspace of
    # [(I-A), -B].  The frozen 16D realization has one equilibrium degree of
    # freedom (tip translation); this avoids inventing a matched disturbance
    # or accepting a nonzero dynamic residual merely because I-A is singular.
    equality = np.column_stack((np.eye(16) - a, -b[:, 0]))
    _, singular_values, vh = np.linalg.svd(equality, full_matrices=True)
    rank = int(np.sum(singular_values > 1.0e-10))
    null_basis = vh[rank:].T
    if null_basis.shape[1] != 1:
        raise ValueError("frozen equilibrium equality must have one-dimensional nullspace")
    output_map = c_pos @ null_basis[:16, :]
    input_map = null_basis[16, :]
    d_hat = float(d_hat)
    normal = output_map.T @ output_map + regularization * (null_basis.T @ null_basis)
    rhs = -output_map.T * d_hat
    alpha = float(np.asarray(np.linalg.solve(normal, rhs)).reshape(-1)[0])
    if abs(float(input_map[0] * alpha)) > float(u_limit):
        alpha = float(np.clip(alpha, -float(u_limit) / max(abs(float(input_map[0])), 1e-15), float(u_limit) / max(abs(float(input_map[0])), 1e-15)))
    z = null_basis[:, 0] * alpha
    x_s = z[:16]
    u_s = float(z[16])
    residual = float(np.linalg.norm((np.eye(16) - a) @ x_s - b[:, 0] * u_s))
    objective = float((c_pos @ x_s + d_hat) ** 2 + regularization * (x_s @ x_s + u_s * u_s))
    return SteadyStateResult(x_s=x_s, u_s=u_s, residual=residual, objective=objective)


def build_of_tsrmpc_qp(a, b, c_task, gain, p_task, x0, x_s, u_s, d_hat,
                       w_p, w_theta, residual_r, previous_actual_ax,
                       horizon=HORIZON, ax_min=AX_MIN, ax_max=AX_MAX,
                       slew_limit=AX_SLEW) -> QPData:
    """Build the dense OSQP problem in residual inputs ``v``.

    Both amplitude and slew constraints are imposed on the physical command
    ``a_k = u_s - K xi_k + v_k``.  The first slew constraint uses the previous
    actual physical command supplied by the runner.
    """
    a, b, c_task, gain, p_task = _as_model_arrays(a, b, c_task, gain, p_task)
    x0 = np.asarray(x0, dtype=float).reshape(16)
    x_s = np.asarray(x_s, dtype=float).reshape(16)
    if horizon != HORIZON:
        raise ValueError("R2 froze H=20")
    if not np.isfinite(x0).all() or not np.isfinite(x_s).all():
        raise ValueError("states must be finite")
    h = int(horizon)
    xi0 = x0 - x_s
    a_cl = a - b @ gain
    f = np.zeros((h + 1, 16), dtype=float)
    g = np.zeros((h + 1, 16, h), dtype=float)
    f[0] = xi0
    for k in range(h):
        f[k + 1] = a_cl @ f[k]
        g[k + 1] = a_cl @ g[k]
        g[k + 1, :, k] += b[:, 0]

    w = np.diag([float(w_p), 0.25 * float(w_p), float(w_theta), 0.25 * float(w_theta)])
    d_vec = np.asarray([float(d_hat), 0.0, 0.0, 0.0])
    pmat = np.zeros((h, h), dtype=float)
    qvec = np.zeros(h, dtype=float)
    for k in range(1, h):
        affine_y = c_task @ (x_s + f[k]) + d_vec
        map_y = c_task @ g[k]
        pmat += 2.0 * (map_y.T @ w @ map_y)
        qvec += 2.0 * (map_y.T @ w @ affine_y)
    # The terminal artifact is the 16D Task-LQR Riccati matrix, so the
    # terminal residual state is xi_H, not the four-dimensional task output.
    pmat += 2.0 * (g[h].T @ p_task @ g[h])
    qvec += 2.0 * (g[h].T @ p_task @ f[h])

    # Physical accelerations and acceleration-rate terms.
    physical_affine = np.zeros(h, dtype=float)
    physical_map = np.zeros((h, h), dtype=float)
    for k in range(h):
        physical_affine[k] = float(u_s - (gain @ f[k])[0])
        physical_map[k] = (-gain @ g[k]).reshape(h)
        physical_map[k, k] += 1.0
    pmat += 2.0 * float(residual_r) * (np.eye(h) + np.zeros((h, h)))
    qvec += 0.0
    # rate d_k = a_k - a_{k-1}; first previous value is measured, not predicted
    rate_maps = np.zeros((h, h), dtype=float)
    rate_affine = np.zeros(h, dtype=float)
    rate_maps[0] = physical_map[0]
    rate_affine[0] = physical_affine[0] - float(previous_actual_ax)
    for k in range(1, h):
        rate_maps[k] = physical_map[k] - physical_map[k - 1]
        rate_affine[k] = physical_affine[k] - physical_affine[k - 1]
    pmat += 2.0 * float(residual_r) * (rate_maps.T @ rate_maps)
    qvec += 2.0 * float(residual_r) * (rate_maps.T @ rate_affine)
    pmat = (pmat + pmat.T) / 2.0

    rows = []
    lower = []
    upper = []
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
    for beta in (0.05, 0.15, 0.30):
        for w_p in (40.0, 80.0, 160.0):
            for w_theta in (5.0, 20.0):
                for residual_r in (0.5, 2.0):
                    rows.append({"candidate_id": f"of_ts_rmpc_{index:03d}", "beta": beta,
                                 "w_p": w_p, "w_theta": w_theta, "R": residual_r, "H": HORIZON})
                    index += 1
    return rows


class OFTSRMPC:
    """Causal online OF-TSRMPC controller for the x channel."""

    def __init__(self, a, b, c_task, gain, p_task, beta, w_p, w_theta, residual_r,
                 solver: OSQPPreviewSolver | None = None):
        self.a, self.b, self.c_task, self.gain, self.p_task = _as_model_arrays(a, b, c_task, gain, p_task)
        if float(beta) not in (0.05, 0.15, 0.30):
            raise ValueError("beta is outside the frozen grid")
        self.beta, self.w_p, self.w_theta, self.residual_r = map(float, (beta, w_p, w_theta, residual_r))
        self.solver = solver or OSQPPreviewSolver(eps_abs=1e-5, eps_rel=1e-5, max_iter=4000, warm_start=True)
        self.limiter = AccelerationLimiter(AX_MIN, AX_MAX, AX_SLEW)
        self.previous_actual_ax = 0.0
        self.d_hat = 0.0
        self.diagnostics = OFTSRMPCDiagnostics(0.0, 0.0, 0.0, 0.0, 0.0, False, False, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "not_run", 0, 0.0, 0.0, 0.0, 0.0)

    @property
    def c_pos(self):
        return self.c_task[0]

    def reset(self, state=None, reference=None):
        del state, reference
        self.previous_actual_ax = 0.0
        self.d_hat = 0.0
        self.limiter.reset(0.0)

    def command(self, x, measured_task_output, reference_ax=0.0):
        total_started = time.perf_counter_ns()
        x = np.asarray(x, dtype=float).reshape(16)
        measured_task_output = np.asarray(measured_task_output, dtype=float).reshape(4)
        if not np.isfinite(x).all() or not np.isfinite(measured_task_output).all():
            raise ValueError("OF-TSRMPC inputs must be finite")
        d_raw = float(measured_task_output[0] - self.c_pos @ x)
        self.d_hat = (1.0 - self.beta) * self.d_hat + self.beta * d_raw
        steady_started = time.perf_counter_ns()
        steady = solve_steady_state(self.a, self.b, self.c_pos, self.d_hat)
        steady_ms = (time.perf_counter_ns() - steady_started) / 1.0e6
        qp = build_of_tsrmpc_qp(self.a, self.b, self.c_task, self.gain, self.p_task, x,
                                 steady.x_s, steady.u_s, self.d_hat, self.w_p, self.w_theta,
                                 self.residual_r, self.previous_actual_ax)
        residual_started = time.perf_counter_ns()
        v, info = self.solver.solve(qp)
        residual_ms = (time.perf_counter_ns() - residual_started) / 1.0e6
        v0 = float(v[0])
        raw = float(steady.u_s - (self.gain @ (x - steady.x_s))[0] + v0 + float(reference_ax))
        limited = float(self.limiter.limit(raw))
        mismatch = abs(limited - raw)
        self.previous_actual_ax = limited
        d = self.limiter.diagnostics
        self.diagnostics = OFTSRMPCDiagnostics(
            float((-self.gain @ (x - steady.x_s).reshape(-1, 1))[0, 0]),
            float(reference_ax + steady.u_s + (self.gain @ steady.x_s)[0]), raw,
            float(d.amplitude_limited), limited, bool(d.saturated), bool(d.slew_limited), float(np.linalg.norm(x)),
            d_raw, self.d_hat, float(np.linalg.norm(steady.x_s)), steady.u_s, steady.residual, v0,
            str(info.status), int(info.iter), steady_ms, residual_ms, (time.perf_counter_ns() - total_started) / 1.0e6, mismatch,
        )
        return limited


def backbone_command(gain, x):
    """Formal d=0, x_s=0, v=0 parity command before limiting."""
    return float((-np.asarray(gain, dtype=float).reshape(1, 16) @ np.asarray(x, dtype=float).reshape(16, 1))[0, 0])
