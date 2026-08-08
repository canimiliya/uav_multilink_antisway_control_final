"""Task-output-weighted full-state LQR for S6T1."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from uav_sway.linearization.analysis import solve_lqr
from uav_sway.control.acceleration_limiter import AccelerationLimiter


@dataclass(frozen=True)
class TaskLQRDiagnostics:
    lqr_feedback_ax: float
    ax_reference_feedforward: float
    ax_cmd_raw: float
    ax_cmd_amplitude_limited: float
    ax_cmd_limited: float
    ax_saturated: bool
    ax_slew_limited: bool
    lqr_state_norm: float


def build_task_lqr(a: np.ndarray, b: np.ndarray, c_task: np.ndarray,
                   w_p: float, w_theta: float, r_value: float,
                   q_s4: np.ndarray | None = None) -> dict:
    a = np.asarray(a, dtype=float).reshape(16, 16)
    b = np.asarray(b, dtype=float).reshape(16, 1)
    c_task = np.asarray(c_task, dtype=float).reshape(4, 16)
    q_base = np.diag([80, 4, 8, 2, 4, 1, 20, 20, 20, 20, 20, 12, 12, 12, 12, 12]) if q_s4 is None else np.asarray(q_s4, dtype=float)
    w = np.diag([float(w_p), 0.25 * float(w_p), float(w_theta), 0.25 * float(w_theta)])
    q = 0.05 * q_base + c_task.T @ w @ c_task
    r = np.asarray([[float(r_value)]], dtype=float)
    result = solve_lqr(a, b, q, r)
    result.update({"Q": q, "R": r, "W": w, "w_p": float(w_p), "w_theta": float(w_theta), "r_value": float(r_value)})
    if not np.isfinite(result["K"]).all() or result["spectral_radius"] >= 1.0:
        raise ValueError("Task-LQR candidate is not finite and asymptotically stable")
    return result


class TaskLQR:
    """Returns x acceleration while weighting cutter task outputs in Q."""

    def __init__(self, gain: np.ndarray, ax_min: float = -2.0,
                 ax_max: float = 2.0, slew_limit: float = 0.25):
        gain = np.asarray(gain, dtype=float)
        if gain.shape != (1, 16) or not np.isfinite(gain).all():
            raise ValueError("Task-LQR gain must be a finite (1,16) array")
        self.gain = gain.copy()
        self.limiter = AccelerationLimiter(ax_min, ax_max, slew_limit)
        self.diagnostics = TaskLQRDiagnostics(0.0, 0.0, 0.0, 0.0, 0.0, False, False, 0.0)

    def reset(self, state=None, reference=None) -> None:
        del state, reference
        self.limiter.reset(0.0)
        self.diagnostics = TaskLQRDiagnostics(0.0, 0.0, 0.0, 0.0, 0.0, False, False, 0.0)

    def command(self, state: np.ndarray, reference, dt: float = 0.05) -> float:
        del dt
        x = np.asarray(state, dtype=float).reshape(16)
        feedback = float((-self.gain @ x.reshape(-1, 1))[0, 0])
        raw = float(reference.ax_ref + feedback)
        limited = self.limiter.limit(raw)
        diag = self.limiter.diagnostics
        self.diagnostics = TaskLQRDiagnostics(
            feedback, float(reference.ax_ref), raw, float(diag.amplitude_limited), float(limited),
            bool(diag.saturated), bool(diag.slew_limited), float(np.linalg.norm(x)),
        )
        return float(limited)
