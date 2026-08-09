"""Five-link adaptation of Jirousek et al. (ICINCO 2025) incremental MPC."""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from uav_sway.mpc.osqp_solver import OSQPPreviewSolver
from uav_sway.mpc.qp_builder import QPData
from uav_sway.v3.controllers import V3ControllerDiagnostics, _V3ControllerBase
from uav_sway.v3.observation import V3Observation, V3Reference


@dataclass(frozen=True)
class IncrementalMPCDiagnostics(V3ControllerDiagnostics):
    delta_command: np.ndarray
    qp_status: str
    qp_iterations: int
    solve_time_ms: float
    limiter_mismatch: float


def build_incremental_qp(a: np.ndarray, b: np.ndarray, c_task: np.ndarray, state: np.ndarray, previous: np.ndarray, parameters: dict) -> QPData:
    """Condense Eq. (24)-(26) of Jirousek et al. with frozen limits."""
    h = int(parameters["horizon_updates"]); nvar = 3 * h
    a_aug = np.block([[a, b], [np.zeros((3, 20)), np.eye(3)]])
    b_aug = np.vstack((b, np.eye(3)))
    c_aug = np.hstack((c_task, np.zeros((12, 3))))
    input_selector = np.hstack((np.zeros((3, 20)), np.eye(3)))
    affine = np.zeros((h + 1, 23)); maps = np.zeros((h + 1, 23, nvar)); affine[0] = np.r_[state, previous]
    for k in range(h):
        affine[k + 1] = a_aug @ affine[k]
        maps[k + 1] = a_aug @ maps[k]
        maps[k + 1, :, 3 * k:3 * k + 3] += b_aug
    weights = np.asarray([
        *([float(parameters["position_weight"])] * 3), *([float(parameters["velocity_weight"])] * 3),
        *([float(parameters["orientation_weight"])] * 3), *([float(parameters["angular_weight"])] * 3),
    ])
    qy = np.diag(weights); pmat = 2.0 * float(parameters["delta_weight"]) * np.eye(nvar); qvec = np.zeros(nvar)
    for k in range(1, h + 1):
        y0 = c_aug @ affine[k]; ym = c_aug @ maps[k]
        u0 = input_selector @ affine[k]; um = input_selector @ maps[k]
        multiplier = 2.0 if k == h else 1.0
        pmat += 2.0 * multiplier * (ym.T @ qy @ ym + float(parameters["input_weight"]) * um.T @ um)
        qvec += 2.0 * multiplier * (ym.T @ qy @ y0 + float(parameters["input_weight"]) * um.T @ u0)
    rows = []; lower = []; upper = []
    for k in range(h):
        for axis in range(3):
            delta_row = np.zeros(nvar); delta_row[3 * k + axis] = 1.0
            rows.append(delta_row); lower.append(-0.25); upper.append(0.25)
            input_map = input_selector[axis] @ maps[k + 1]; input_affine = float(input_selector[axis] @ affine[k + 1])
            rows.append(input_map); lower.append(-2.0 - input_affine); upper.append(2.0 - input_affine)
    pmat = 0.5 * (pmat + pmat.T) + 1.0e-8 * np.eye(nvar)
    return QPData(pmat, qvec, np.asarray(rows), np.asarray(lower), np.asarray(upper), affine, maps)


class Jirousek2025IncrementalMPC(_V3ControllerBase):
    """Incremental physical-command MPC adapted to the frozen 20D five-link model."""

    def __init__(self, a: np.ndarray, b: np.ndarray, c_task: np.ndarray, parameters: dict) -> None:
        super().__init__(); self.a = np.asarray(a).reshape(20, 20); self.b = np.asarray(b).reshape(20, 3); self.c_task = np.asarray(c_task).reshape(12, 20); self.parameters = dict(parameters)
        self.solver = OSQPPreviewSolver(eps_abs=1e-5, eps_rel=1e-5, max_iter=4000, warm_start=True); self.reset()

    def reset(self) -> None:
        super().reset(); z3 = np.zeros(3)
        self.diagnostics = IncrementalMPCDiagnostics(z3, z3, z3, np.zeros(3, dtype=bool), np.zeros(3, dtype=bool), 0.0, z3, z3, "not_run", 0, 0.0, 0.0)

    def command(self, observation: V3Observation, reference: V3Reference, dt: float = 0.05) -> np.ndarray:
        del reference
        if abs(float(dt) - 0.05) > 1e-12: raise ValueError("paper adaptation requires frozen 20 Hz outer rate")
        started = time.perf_counter_ns(); state = np.asarray(observation.full_state_error); previous = self.limiter.previous.copy()
        qp = build_incremental_qp(self.a, self.b, self.c_task, state, previous, self.parameters); solution, info = self.solver.solve(qp)
        delta = solution[:3]; raw = previous + delta; amplitude = np.clip(raw, -2.0, 2.0); final = self.limiter.limit(raw).as_array()
        self.diagnostics = IncrementalMPCDiagnostics(raw.copy(), amplitude, final.copy(), np.abs(raw) > 2.0 + 1e-9, np.abs(final - amplitude) > 1e-9, float(np.linalg.norm(state)), np.zeros(3), delta.copy(), str(info.status), int(info.iter), (time.perf_counter_ns() - started) / 1e6, float(np.max(np.abs(final - raw))))
        return final
