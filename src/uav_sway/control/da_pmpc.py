"""Disturbance-aware preview MPC pilot controller."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .acceleration_limiter import AccelerationLimiter
from .base import ReferenceState
from .disturbance_observer import MatchedDisturbanceObserver
from uav_sway.mpc.qp_builder import build_preview_qp
from uav_sway.mpc.qp_builder import build_stabilized_preview_qp
from uav_sway.mpc.preview_model import PreviewModel


@dataclass(frozen=True)
class DAPMPCDiagnostics:
    ax_cmd_raw: float
    ax_cmd_limited: float
    delta_ax: float
    disturbance_hat: float
    solve_time_ms: float
    status: str
    iterations: int
    qp_limiter_mismatch: float = 0.0


class DAPMPC:
    def __init__(self, A, B, Q, P, C_tip, position_scale, tip_weight,
                 solver, observer=None, horizon_steps=20, ax_min=-2.0,
                 ax_max=2.0, slew_limit=0.25, control_weight=1.0):
        self.A=np.asarray(A); self.B=np.asarray(B); self.Q=np.asarray(Q); self.P=np.asarray(P); self.C_tip=np.asarray(C_tip)
        self.position_scale=float(position_scale); self.tip_weight=float(tip_weight)
        self.solver=solver; self.horizon_steps=int(horizon_steps)
        self.ax_min=float(ax_min); self.ax_max=float(ax_max); self.slew_limit=float(slew_limit)
        self.control_weight=float(control_weight)
        self.limiter=AccelerationLimiter(ax_min, ax_max, slew_limit)
        self.observer=observer or MatchedDisturbanceObserver(A,B)
        self._previous_reference = None
        self.diagnostics=DAPMPCDiagnostics(0,0,0,0,0,"reset",0)

    def reset(self, state=None, reference=None):
        del reference
        self.limiter.reset(0.0); self.observer.reset(state, 0.0, 0.0)
        self._previous_reference = None
        self.diagnostics=DAPMPCDiagnostics(0,0,0,0,0,"reset",0)

    def command(self, state, horizon, solve_time_ms=0.0):
        state=np.asarray(state,dtype=float).reshape(16)
        references=horizon.boundary_samples
        current_reference = horizon.action_reference(0)
        previous_action = float(self.limiter.previous)
        if self._previous_reference is None:
            reference_shift = np.zeros(16, dtype=float)
        else:
            reference_shift = PreviewModel.static_reference_shift(self.A, self._previous_reference, current_reference)
        d_hat=self.observer.update(state, previous_action, reference_shift)
        preview = PreviewModel(self.A, self.B, self.Q, self.P, self.C_tip,
                               self.horizon_steps, self.control_weight)
        qp=build_preview_qp(preview, state, references, previous_action,
                            self.position_scale, self.tip_weight, self.ax_min,
                            self.ax_max, self.slew_limit, d_hat)
        solution, info=self.solver.solve(qp)
        delta=float(solution[0]); raw=float(horizon.action_reference(0).ax_ref + delta)
        limited=float(self.limiter.limit(raw))
        d=self.limiter.diagnostics
        self._previous_reference = current_reference
        self.diagnostics=DAPMPCDiagnostics(raw, limited, delta, d_hat, float(solve_time_ms), str(info.status), int(info.iter), abs(limited-raw))
        return limited


@dataclass(frozen=True)
class StabilizedDAPMPCDiagnostics:
    ax_cmd_raw: float
    ax_cmd_limited: float
    lqr_feedback_ax: float
    residual_v: float
    disturbance_hat: float
    solve_time_ms: float
    status: str
    iterations: int
    qp_limiter_mismatch: float


class LQRStabilizedDAPMPC:
    """DA-PMPC residual optimizer built around the frozen S4 LQR."""

    def __init__(self, A, B, Q, P, C_tip, gain, tip_weight, residual_weight,
                 solver, observer=None, horizon_steps=20, ax_min=-2.0,
                 ax_max=2.0, slew_limit=0.25, observer_enabled=True,
                 residual_enabled=True, disturbance_compensation=True):
        self.A = np.asarray(A, dtype=float).reshape(16, 16)
        self.B = np.asarray(B, dtype=float).reshape(16, 1)
        self.Q = np.asarray(Q, dtype=float).reshape(16, 16)
        self.P = np.asarray(P, dtype=float).reshape(16, 16)
        self.C_tip = np.asarray(C_tip, dtype=float).reshape(1, 16)
        self.gain = np.asarray(gain, dtype=float).reshape(1, 16)
        self.tip_weight = float(tip_weight)
        self.residual_weight = float(residual_weight)
        self.solver = solver
        self.observer = observer or MatchedDisturbanceObserver(self.A, self.B[:, 0])
        self.horizon_steps = int(horizon_steps)
        self.limiter = AccelerationLimiter(ax_min, ax_max, slew_limit)
        self.ax_min = float(ax_min)
        self.ax_max = float(ax_max)
        self.slew_limit = float(slew_limit)
        self.observer_enabled = bool(observer_enabled)
        self.residual_enabled = bool(residual_enabled)
        self.disturbance_compensation = bool(disturbance_compensation)
        self._previous_reference = None
        self.diagnostics = StabilizedDAPMPCDiagnostics(0, 0, 0, 0, 0, 0, "reset", 0, 0)

    @property
    def A_cl(self):
        return self.A - self.B @ self.gain

    def reset(self, state=None, reference=None):
        del reference
        self.limiter.reset(0.0)
        self.observer.reset(state, 0.0, 0.0)
        self._previous_reference = None
        self.diagnostics = StabilizedDAPMPCDiagnostics(0, 0, 0, 0, 0, 0, "reset", 0, 0)

    def command(self, state, horizon, solve_time_ms=0.0):
        state = np.asarray(state, dtype=float).reshape(16)
        current_reference = horizon.action_reference(0)
        previous_action = float(self.limiter.previous)
        if self._previous_reference is None:
            shift = np.zeros(16, dtype=float)
        else:
            shift = PreviewModel.static_reference_shift(self.A, self._previous_reference, current_reference)
        d_hat = 0.0
        if self.observer_enabled:
            d_hat = float(self.observer.update(state, previous_action, shift))
        feedback = float((-self.gain @ state.reshape(-1, 1))[0, 0])
        if not self.residual_enabled:
            residual = 0.0
            raw = float(current_reference.ax_ref + feedback - (d_hat if self.disturbance_compensation else 0.0))
            status = "parity"
            iterations = 0
        else:
            preview = PreviewModel(self.A, self.B, self.Q, self.P, self.C_tip,
                                   self.horizon_steps, 1.0)
            qp = build_stabilized_preview_qp(
                preview, state, horizon.boundary_samples, self.gain,
                previous_action, self.tip_weight, self.residual_weight,
                disturbance=(d_hat if self.disturbance_compensation else 0.0),
                ax_min=self.ax_min, ax_max=self.ax_max, slew_limit=self.slew_limit,
            )
            solution, info = self.solver.solve(qp)
            residual = float(solution[0])
            raw = float(current_reference.ax_ref + feedback
                        - (d_hat if self.disturbance_compensation else 0.0)
                        - residual)
            status = str(info.status)
            iterations = int(info.iter)
        limited = float(self.limiter.limit(raw))
        self._previous_reference = current_reference
        self.diagnostics = StabilizedDAPMPCDiagnostics(
            raw, limited, feedback, residual, d_hat, float(solve_time_ms),
            status, iterations, abs(limited - raw),
        )
        return limited
