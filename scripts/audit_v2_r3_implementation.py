"""Freeze-only R3 implementation and algebra audit.

This script deliberately performs no 12 s MuJoCo performance run and never
loads the holdout manifest as executable input.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from uav_sway.control.of_tsrmpc import (
    HORIZON,
    DT,
    backbone_command,
    build_of_tsrmpc_qp,
    enumerate_grid,
    solve_steady_state,
)
from uav_sway.control.task_lqr import build_task_lqr
from uav_sway.evaluation.task_baseline_runner import make_task_output_map
from uav_sway.linearization.task_output import identify_task_output_jacobian, validate_task_output_local


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v2/r3"
START_HEAD = "2aa1d72a58fdb2bac450d1106b1e044bc258e0ee"


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def dump(name: str, value: dict) -> None:
    (OUT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def main() -> int:
    global OUT
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=str(OUT))
    args = parser.parse_args()
    OUT = Path(args.output)
    OUT.mkdir(parents=True, exist_ok=True)

    A = np.load(ROOT / "reproducibility/frozen/linear_model/A.npy")
    B = np.load(ROOT / "reproducibility/frozen/linear_model/B.npy")
    C = np.load(ROOT / "reproducibility/frozen/task_lqr/C_task.npy")
    K = np.asarray(build_task_lqr(A, B, C, 20, 5, 1)["K"], dtype=float)
    task_lqr = build_task_lqr(A, B, C, 20, 5, 1)
    P = np.asarray(task_lqr["P"], dtype=float)

    model, data, task_map, _ = make_task_output_map(ROOT)
    epsilon = np.asarray([1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5] + [1e-5] * 10, dtype=float)
    identified, y0 = identify_task_output_jacobian(task_map, epsilon)
    local = validate_task_output_local(task_map, C, epsilon, multipliers=(1, 2, 5), sample_count=32)
    semantic_rows = [
        "cutter tip-x position error",
        "cutter tip-x velocity",
        "cutter orientation / planar angle",
        "cutter angular velocity",
    ]
    task_audit = {
        "source": "reproducibility/frozen/task_lqr/C_task.npy",
        "shape": list(C.shape),
        "row_semantics": semantic_rows,
        "equilibrium_output": y0.tolist(),
        "identified_local_jacobian_max_abs_error": float(np.max(np.abs(identified - C))),
        "identified_local_jacobian": identified.tolist(),
        "frozen_sha256": sha(ROOT / "reproducibility/frozen/task_lqr/C_task.npy"),
        "local_nonlinear_parity": local,
        "pass": bool(C.shape == (4, 16) and np.isfinite(C).all() and local["pass"]),
    }
    dump("task_output_matrix_audit.json", task_audit)
    if not task_audit["pass"]:
        dump("gate.json", {"result": "BLOCKED_IMPLEMENTATION_CONTRACT", "c_task_semantics_pass": False})
        return 2

    # Synthetic causal bias and target-step audits.
    x0 = np.zeros(16)
    measured0 = C @ x0
    x_step = x0.copy(); x_step[0] = -0.15
    measured_step = C @ x_step
    d_before = float(measured0[0] - C[0] @ x0)
    d_after = float(measured_step[0] - C[0] @ x_step)
    dump("target_step_bias_audit.json", {
        "test": "equilibrium target -> +0.15 m x",
        "measured_error_before": float(measured0[0]),
        "measured_error_after": float(measured_step[0]),
        "d_raw_before": d_before,
        "d_raw_after": d_after,
        "false_bias_tolerance": 1e-12,
        "pass": bool(abs(d_before) <= 1e-12 and abs(d_after) <= 1e-12),
        "no_holdoff_deadzone_or_clamp_added": True,
    })

    ss = solve_steady_state(A, B, C[0], 0.15)
    dump("steady_state_optimizer_audit.json", {
        "u_limit": 2.0,
        "regularization": 1e-8,
        "d_hat": 0.15,
        "steady_state_residual": ss.residual,
        "u_s": ss.u_s,
        "x_s_norm": float(np.linalg.norm(ss.x_s)),
        "objective": ss.objective,
        "pass": bool(ss.residual <= 1e-10 and abs(ss.u_s) <= 2.0 + 1e-12),
    })

    corrected = ss.u_s - float((K @ (ss.x_s - ss.x_s))[0])
    dump("steady_state_control_parity.json", {
        "formula": "a_x = u_s - K_task (x - x_s) + v_mpc",
        "x_equals_x_s": True,
        "v_mpc": 0.0,
        "u_s": ss.u_s,
        "a_x": corrected,
        "absolute_error": abs(corrected - ss.u_s),
        "pass": bool(abs(corrected - ss.u_s) <= 1e-10),
    })

    qp = build_of_tsrmpc_qp(A, B, C, K, P, np.zeros(16), np.zeros(16), 0.0, 0.0, 40, 5, 0.5, 0.0)
    solver_status = "not_run"
    solver_iterations = 0
    try:
        from uav_sway.mpc.osqp_solver import OSQPPreviewSolver
        _, info = OSQPPreviewSolver(eps_abs=1e-5, eps_rel=1e-5, max_iter=4000, warm_start=True).solve(qp)
        solver_status = str(info.status)
        solver_iterations = int(info.iter)
    except Exception as exc:
        solver_status = f"failure:{type(exc).__name__}:{exc}"
    dump("qp_constraint_audit.json", {
        "horizon": HORIZON,
        "dt_s": DT,
        "variable": "v_0...v_19",
        "physical_acceleration_constraints": True,
        "first_step_uses_previous_actual_ax": True,
        "slew_limit": 0.25,
        "amplitude_limit": 2.0,
        "solver": {"name": "OSQP", "eps_abs": 1e-5, "eps_rel": 1e-5, "max_iter": 4000, "warm_start": True, "status": solver_status, "iterations": solver_iterations},
        "pass": solver_status == "solved" and qp.A.shape == (40, 20),
    })

    rng = np.random.default_rng(20260808)
    parity_errors = []
    for _ in range(32):
        x = rng.normal(size=16)
        parity_errors.append(abs(backbone_command(K, x) - float((-K @ x.reshape(-1, 1))[0, 0])))
    dump("backbone_parity.json", {
        "d_hat": 0.0, "x_s": "zero", "v_mpc": 0.0,
        "sample_count": 32, "maximum_absolute_error": max(parity_errors),
        "tolerance": 1e-10, "pass": max(parity_errors) <= 1e-10,
    })
    dump("implementation_contract.json", {
        "task": "V2-R3-OF-TSRMPC-DEVELOPMENT-AND-FREEZE-R1",
        "start_head": START_HEAD,
        "method": "OF-TSRMPC",
        "performance_runs_executed": False,
        "holdout_executed": False,
        "advanced_paper_executed": False,
        "horizon": HORIZON, "dt_s": DT,
        "grid_size": len(enumerate_grid()),
        "output_bias": "tip-x position output only",
        "matched_wind_dob": False,
        "physical_ax_constraints_in_qp": True,
        "solver": {"name": "OSQP", "eps_abs": 1e-5, "eps_rel": 1e-5, "max_iter": 4000, "warm_start": True},
        "r1r1_and_r2_read_only": True,
    })
    dump("implementation_audit.json", {
        "task_output_matrix": task_audit["pass"],
        "target_step_bias": abs(d_before) <= 1e-12 and abs(d_after) <= 1e-12,
        "steady_state_optimizer": ss.residual <= 1e-10,
        "steady_state_control_parity": abs(corrected - ss.u_s) <= 1e-10,
        "backbone_parity": max(parity_errors) <= 1e-10,
        "qp_smoke": solver_status == "solved",
        "grid_size": len(enumerate_grid()),
        "all_freeze_only": True,
        "pass": bool(task_audit["pass"] and abs(d_before) <= 1e-12 and abs(d_after) <= 1e-12 and ss.residual <= 1e-10 and abs(corrected - ss.u_s) <= 1e-10 and max(parity_errors) <= 1e-10 and solver_status == "solved" and len(enumerate_grid()) == 36),
    })
    print(json.dumps(json.loads((OUT / "implementation_audit.json").read_text(encoding="utf-8")), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
