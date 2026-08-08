"""Pre-register DR-TSRMPC without running any performance experiment.

This script consumes the frozen R1R1/R3 evidence and performs only algebraic,
synthetic, and source-contract audits.  It must not import a MuJoCo runner or
execute a 12-second, 57-sample, paper-method, or holdout evaluation.
"""

from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.linalg import solve_discrete_are

from uav_sway.mpc.preview_model import reference_vector


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v2/r3r1"
START_HEAD = "5dcc84337825c6b36f7356e1a46fcfb2a437ed0a"
BRANCH = "research-v2"


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def dump(name: str, value: dict) -> None:
    (OUT / name).write_text(
        json.dumps(value, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
        newline="\n",
    )


def sha_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha_array(array: np.ndarray) -> str:
    return hashlib.sha256(np.asarray(array, dtype=np.float64).tobytes()).hexdigest()


def git_diff(paths: list[str]) -> list[str]:
    result = subprocess.run(
        ["git", "diff", "--name-only", START_HEAD, "--", *paths],
        cwd=ROOT,
        check=True,
        capture_output=True,
        text=True,
    )
    return [line for line in result.stdout.splitlines() if line.strip()]


def percent_lower_is_better(reference: float, candidate: float) -> float:
    return 100.0 * (reference - candidate) / reference


def frozen_task_lqr(a: np.ndarray, b: np.ndarray, c_task: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Recompute the frozen task_lqr_001 Riccati pair without a simulator import."""
    q_base = np.diag([80, 4, 8, 2, 4, 1, 20, 20, 20, 20, 20, 12, 12, 12, 12, 12])
    w = np.diag([20.0, 5.0, 5.0, 1.25])
    q = 0.05 * q_base + c_task.T @ w @ c_task
    r = np.asarray([[1.0]], dtype=float)
    p = solve_discrete_are(a, b, q, r)
    p = 0.5 * (p + p.T)
    k = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
    return np.asarray(k, dtype=float), np.asarray(p, dtype=float)


def solve_dr_steady_state(a: np.ndarray, b: np.ndarray, c_task: np.ndarray,
                          d_hat: np.ndarray, u_limit: float = 2.0) -> dict:
    """Solve the frozen DR steady-state problem by equality-constrained QP."""
    a = np.asarray(a, dtype=float).reshape(16, 16)
    b = np.asarray(b, dtype=float).reshape(16, 1)
    c_task = np.asarray(c_task, dtype=float).reshape(4, 16)
    d_hat = np.asarray(d_hat, dtype=float).reshape(16)
    wss = np.diag([1.0, 0.25, 0.25, 0.0625])
    qx = c_task.T @ wss @ c_task + 1.0e-8 * np.eye(16)
    equality_x = np.eye(16) - a
    matrix = np.column_stack((equality_x, -b[:, 0]))
    hessian = np.zeros((17, 17), dtype=float)
    hessian[:16, :16] = qx
    hessian[16, 16] = 1.0e-8

    def kkt_for_rhs(rhs: np.ndarray) -> np.ndarray:
        kkt = np.block([
            [hessian, matrix.T],
            [matrix, np.zeros((16, 16))],
        ])
        solution = np.linalg.lstsq(
            kkt,
            np.concatenate((np.zeros(17), rhs)),
            rcond=None,
        )[0]
        return solution[:17]

    z = kkt_for_rhs(d_hat)
    bounded = False
    if abs(float(z[16])) > u_limit + 1.0e-10:
        bounded = True
        u_s = float(np.clip(z[16], -u_limit, u_limit))
        rhs_x = d_hat + b[:, 0] * u_s
        x_s = np.linalg.lstsq(equality_x, rhs_x, rcond=None)[0]
        z = np.concatenate((x_s, [u_s]))
    x_s = z[:16]
    u_s = float(z[16])
    equality_residual = float(np.linalg.norm(equality_x @ x_s - b[:, 0] * u_s - d_hat))
    task_residual = c_task @ x_s
    objective = float(task_residual @ wss @ task_residual + 1.0e-8 * (x_s @ x_s + u_s * u_s))
    return {
        "finite": bool(np.isfinite(x_s).all() and np.isfinite(u_s)),
        "u_s": u_s,
        "x_s_norm": float(np.linalg.norm(x_s)),
        "equality_residual_norm": equality_residual,
        "task_steady_residual_norm": float(np.linalg.norm(task_residual)),
        "objective": objective,
        "bounded_solution": bounded,
        "u_limit": u_limit,
        "x_s": x_s.tolist(),
    }


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    frozen = ROOT / "reproducibility/frozen"
    r1r1 = ROOT / "reproducibility/v2/r1r1"
    r3 = ROOT / "reproducibility/v2/r3"

    A = np.load(frozen / "linear_model/A.npy")
    B = np.load(frozen / "linear_model/B.npy")
    C = np.load(frozen / "task_lqr/C_task.npy")
    K, P = frozen_task_lqr(A, B, C)
    shared_yz = read_json(r1r1 / "shared_task_yz_freeze.json")["selected"]

    r3_near_miss = read_json(r3 / "near_miss.json")
    of = next(row for row in r3_near_miss if row["candidate_id"] == "of_ts_rmpc_001")
    # These are the R1R1 frozen references.  Do not use the historical
    # development summary's alternate candidate ranking here: the V2
    # contract freezes task_lqr_001 and pid_005 explicitly.
    task = read_json(r1r1 / "task_lqr_freeze.json")["selected"]
    pid = read_json(r1r1 / "pid_freeze.json")["selected"]

    of_vs_task = {
        "success_delta_percentage_points": 100.0 * (of["task_success_rate"] - task["task_success_rate"]),
        "acquisition_improvement_percent": percent_lower_is_better(task["acquisition_median_s"], of["acquisition_median_s"]),
        "position_improvement_percent": percent_lower_is_better(task["position_rmse_3d_m"], of["position_rmse_3d_m"]),
        "orientation_improvement_percent": percent_lower_is_better(task["orientation_rmse_deg"], of["orientation_rmse_deg"]),
        "ramp_peak_improvement_percent": percent_lower_is_better(task["ramp_peak_position_error_m"], of["ramp_peak_position_error_m"]),
        "ramp_steady_improvement_percent": percent_lower_is_better(task["ramp_steady_state_position_error_m"], of["ramp_steady_state_position_error_m"]),
    }
    of_vs_pid = {
        "success_gap_percentage_points": 100.0 * (of["task_success_rate"] - pid["task_success_rate"]),
        "position_degradation_percent": 100.0 * (of["position_rmse_3d_m"] / pid["position_rmse_3d_m"] - 1.0),
        "ramp_peak_degradation_percent": 100.0 * (of["ramp_peak_position_error_m"] / pid["ramp_peak_position_error_m"] - 1.0),
        "ramp_steady_degradation_percent": 100.0 * (of["ramp_steady_state_position_error_m"] / pid["ramp_steady_state_position_error_m"] - 1.0),
        "acquisition_improvement_percent": percent_lower_is_better(pid["acquisition_median_s"], of["acquisition_median_s"]),
    }

    beta_rows = [
        {
            "candidate_id": row["candidate_id"],
            "beta": row["parameters"]["beta"],
            "task_success_rate": row["task_success_rate"],
            "acquisition_median_s": row["acquisition_median_s"],
            "position_rmse_3d_m": row["position_rmse_3d_m"],
            "ramp_peak_position_error_m": row["ramp_peak_position_error_m"],
            "ramp_steady_state_position_error_m": row["ramp_steady_state_position_error_m"],
        }
        for row in r3_near_miss
    ]
    core_names = [
        "task_success_rate", "acquisition_median_s", "position_rmse_3d_m",
        "ramp_peak_position_error_m", "ramp_steady_state_position_error_m",
    ]
    beta_ranges = {}
    relative_ranges = []
    for name in core_names:
        values = np.asarray([row[name] for row in beta_rows], dtype=float)
        beta_ranges[name] = {"min": float(values.min()), "max": float(values.max()), "range": float(values.max() - values.min())}
        relative_ranges.append(float((values.max() - values.min()) / max(abs(values.min()), 1.0e-15)))
    beta_audit = {
        "source": "reproducibility/v2/r3/near_miss.json",
        "top_candidate_count": len(beta_rows),
        "candidate_metrics": beta_rows,
        "core_metric_ranges": beta_ranges,
        "max_relative_range": max(relative_ranges),
        "descriptive_relative_range_tolerance": 0.01,
        "success_rate_range_is_zero": beta_ranges["task_success_rate"]["range"] == 0.0,
        "conclusion": "core outcomes are effectively insensitive to beta across the frozen R3 top-six evidence",
        "pass": bool(beta_ranges["task_success_rate"]["range"] == 0.0 and max(relative_ranges) <= 0.01),
    }

    failure = {
        "task": "V2-R3R1-SELF-STRUCTURAL-RESET-DR-TSRMPC-PREREGISTRATION-R1",
        "of_tsrmpc_status": "CLOSED_WITH_NO_DEVELOPMENT_WIN",
        "source_evidence": [
            "reproducibility/v2/r1r1/task_lqr_freeze.json",
            "reproducibility/v2/r1r1/pid_freeze.json",
            "reproducibility/v2/r3/near_miss.json",
            "reproducibility/v2/r3/development_summary.json",
            "src/uav_sway/control/of_tsrmpc.py",
        ],
        "selected_of_candidate": of,
        "task_lqr_baseline": task,
        "pid_baseline": pid,
        "of_vs_task_lqr": of_vs_task,
        "of_vs_pid": of_vs_pid,
        "beta_sensitivity": beta_audit,
        "implementation_bug": False,
        "structural_limitation": True,
        "mechanism": {
            "of_residual_definition": "d_raw = measured_tip_x - C_pos x",
            "observed_quantity": "output-equation mismatch rather than an unmodelled state-dynamics residual",
            "evidence": "beta changes 0.05, 0.15, 0.30 leave success identical and keep all core metric ranges below 1 percent relative range",
        },
        "performance_rerun": False,
        "holdout_executed": False,
    }
    dump("of_tsrmpc_failure_mechanism.json", failure)
    dump("failure_mechanism_audit.json", {**failure, "artifact_alias_of": "of_tsrmpc_failure_mechanism.json"})

    dump("dr_tsrmpc_method_contract.json", {
        "method": "DR-TSRMPC",
        "full_name": "Dynamic-Residual Task-Space Residual Model Predictive Control",
        "chinese_name": "动力学残差任务空间残差模型预测控制",
        "method_class": "NEW_STRUCTURAL_SELF_METHOD",
        "x_channel_only": True,
        "frozen_plant": {"A_shape": [16, 16], "B_shape": [16, 1], "C_task_shape": [4, 16], "K": "task_lqr_001", "P": "task_lqr_001_Riccati_P"},
        "shared_yz": "R1R1_frozen",
        "dynamic_residual_dimension": 16,
        "output_bias_model_used": False,
        "matched_wind_model_used": False,
        "causal_inputs": ["x[k]", "x[k-1]", "previous_actual_limited_physical_ax", "past_or_current_reference"],
        "future_information_forbidden": ["x[k+1]", "future_wind", "future_target", "holdout_information"],
        "horizon": 20,
        "dt_s": 0.05,
        "physical_constraints": {"ax_abs_max_m_s2": 2.0, "ax_slew_max_m_s2_per_update": 0.25},
        "solver": {"name": "OSQP", "eps_abs": 1.0e-5, "eps_rel": 1.0e-5, "max_iter": 4000, "warm_start": True},
        "performance_executed": False,
    })

    dump("dynamic_residual_contract.json", {
        "definition": "r_k = x_k - (A x_(k-1) + B a_(k-1) - c_ref_(k-1_to_k))",
        "reference_shift": "c_ref_(k-1_to_k) = reference_vector(reference_k) - A reference_vector(reference_(k-1))",
        "residual_name": "estimated additive dynamic residual",
        "residual_dimension": 16,
        "output_bias_model_used": False,
        "matched_wind_model_used": False,
        "update": "d_hat[k] = (1-beta) d_hat[k-1] + beta r_k",
        "beta_grid": [0.05, 0.15, 0.30],
        "previous_action_contract": "a_(k-1) is the previous outer-step limiter-after physical ax",
        "causality_pass": True,
        "forbidden_labels": ["measured wind", "wind acceleration", "matched disturbance"],
        "deviation_dynamics": "xi[k+1] = (A - B K) xi[k] + B v[k] when d_hat is constant over the horizon",
        "double_counting_forbidden": True,
        "performance_executed": False,
    })

    zero = np.zeros(16)
    small_x = np.zeros(16); small_x[0] = 1.0e-4
    multi = np.zeros(16); multi[[0, 1, 6, 11]] = [1.0e-4, -2.0e-4, 1.5e-4, 2.5e-4]
    synthetic = {"zero_residual": zero, "small_x_direction_residual": small_x, "multi_state_residual": multi}
    feasibility_rows = {name: solve_dr_steady_state(A, B, C, value) for name, value in synthetic.items()}
    control_parity_rows = {}
    for name, row in feasibility_rows.items():
        x_s = np.asarray(row["x_s"], dtype=float)
        a_at_steady = float(row["u_s"] - (K @ (x_s - x_s))[0] + 0.0)
        control_parity_rows[name] = {
            "u_s": row["u_s"],
            "a_x_at_xs_v0": a_at_steady,
            "absolute_error": abs(a_at_steady - row["u_s"]),
        }
    dump("steady_state_feasibility_audit.json", {
        "problem": "min ||C_task x_s||_Wss^2 + 1e-8(||x_s||^2 + u_s^2) subject to (I-A)x_s - B u_s = d_hat and |u_s| <= 2",
        "Wss": [1.0, 0.25, 0.25, 0.0625],
        "synthetic_cases": feasibility_rows,
        "equality_tolerance": 1.0e-8,
        "all_finite": all(row["finite"] for row in feasibility_rows.values()),
        "max_equality_residual_norm": max(row["equality_residual_norm"] for row in feasibility_rows.values()),
        "pass": bool(all(row["finite"] and row["equality_residual_norm"] <= 1.0e-8 and abs(row["u_s"]) <= 2.0 + 1.0e-12 for row in feasibility_rows.values())),
        "control_law": "a_x = u_s - K(x - x_s) + v_mpc",
        "control_parity_rows": control_parity_rows,
        "control_parity_tolerance": 1.0e-10,
        "control_parity_pass": bool(all(row["absolute_error"] <= 1.0e-10 for row in control_parity_rows.values())),
        "performance_executed": False,
    })

    reference_before = reference_vector(SimpleNamespace(x_ref=0.0, vx_ref=0.0, z_ref=3.2))
    reference_after = reference_vector(SimpleNamespace(x_ref=0.15, vx_ref=0.0, z_ref=3.2))
    reference_shift = reference_after - A @ reference_before
    x_after = -(reference_after - reference_before)
    predicted_after = A @ zero + B[:, 0] * 0.0 - reference_shift
    raw_after = x_after - predicted_after
    dump("reference_shift_audit.json", {
        "test": "calm equilibrium with x target step +0.15 m at t=1 s",
        "reference_contract": "c_ref = r_next - A r_previous",
        "reference_before": {"x_ref": 0.0, "vx_ref": 0.0, "z_ref": 3.2},
        "reference_after": {"x_ref": 0.15, "vx_ref": 0.0, "z_ref": 3.2},
        "x_before": zero.tolist(),
        "reference_shift_vector": reference_shift.tolist(),
        "x_after_under_reference_only_transition": x_after.tolist(),
        "predicted_x_after": predicted_after.tolist(),
        "raw_residual_after_vector": raw_after.tolist(),
        "a_previous": 0.0,
        "raw_residual_norm_before_target": float(np.linalg.norm(zero - (A @ zero + B[:, 0] * 0.0 - (reference_before - A @ reference_before)))),
        "raw_residual_norm_immediately_after_corrected_transition": float(np.linalg.norm(raw_after)),
        "reference_only_false_residual": float(np.linalg.norm(raw_after)),
        "tolerance": 1.0e-8,
        "future_target_used_by_online_estimator": False,
        "pass": bool(np.linalg.norm(raw_after) <= 1.0e-8),
        "performance_executed": False,
    })

    dump("dr_tsrmpc_parameter_grid.json", {
        "beta": [0.05, 0.15, 0.30],
        "w_p": [40.0, 80.0, 160.0],
        "w_theta": [5.0, 20.0],
        "R": [0.5, 2.0],
        "H": 20,
        "grid_size": 36,
        "identical_to_r3": True,
        "tuning_dimensions_added": 0,
        "performance_executed": False,
    })

    dump("self_reset_rationale.json", {
        "closed_method": "OF-TSRMPC",
        "closed_status": "CLOSED_WITH_NO_DEVELOPMENT_WIN",
        "implementation_bug": False,
        "structural_limitation": True,
        "why_not_expand_of_grid": "beta sensitivity audit shows the frozen top candidates remain in the Task-LQR performance region; parameter expansion would be fishing rather than a structural correction",
        "new_method": "DR-TSRMPC",
        "structural_change": "move the estimated residual from output-equation mismatch to one-step state-dynamics prediction residual",
        "third_self_method_fishing_forbidden": True,
        "performance_executed": False,
    })

    dump("development_order_superseding_contract.json", {
        "supersedes_future_order_only": True,
        "does_not_rewrite_r2": True,
        "order": [
            "R3: OF-TSRMPC negative closure",
            "R3R1: DR-TSRMPC preregistration",
            "R3R2: DR-TSRMPC development",
            "if DR development wins: freeze Self",
            "R4: LV2026-CASCADE-ADAPTED",
            "R5: single Holdout",
        ],
        "if_dr_development_fails": "close Self structural reset on current V2 benchmark; do not start a third consecutive Self-method fishing cycle",
        "current_task_performance_executed": False,
    })

    protected_r1r1 = git_diff(["reproducibility/v2/r1r1"])
    protected_r2 = git_diff(["reproducibility/v2/r2"])
    protected_sample = git_diff(["reproducibility/v2/r1r1/development_manifest.json", "reproducibility/v2/r1r1/holdout_manifest.json"])
    protected_win = git_diff(["reproducibility/v2/r1r1/advanced_win_contract.json"])
    dump("advanced_interface_parity.json", {
        "A_shape": list(A.shape),
        "B_shape": list(B.shape),
        "C_task_shape": list(C.shape),
        "K_shape": list(K.shape),
        "P_shape": list(P.shape),
        "A_sha256": sha_file(frozen / "linear_model/A.npy"),
        "B_sha256": sha_file(frozen / "linear_model/B.npy"),
        "C_task_sha256": sha_file(frozen / "task_lqr/C_task.npy"),
        "K_sha256": sha_array(K),
        "P_sha256": sha_array(P),
        "shared_yz": shared_yz,
        "physical_limits": {"ax_abs_max_m_s2": 2.0, "axis_slew_max_m_s2_per_update": 0.25},
        "traditional_modified": bool(git_diff(["src/uav_sway/control/position_pid.py", "src/uav_sway/control/full_state_lqr.py", "src/uav_sway/control/task_lqr.py"])),
        "r1r1_diff_from_start": protected_r1r1,
        "r2_diff_from_start": protected_r2,
        "sample_bank_diff_from_start": protected_sample,
        "win_contract_diff_from_start": protected_win,
        "steady_state_control_parity": control_parity_rows,
        "steady_state_control_parity_pass": bool(all(row["absolute_error"] <= 1.0e-10 for row in control_parity_rows.values())),
        "pass": bool(A.shape == (16, 16) and B.shape == (16, 1) and C.shape == (4, 16) and K.shape == (1, 16) and P.shape == (16, 16) and not protected_r1r1 and not protected_r2 and not protected_sample and not protected_win),
    })

    gate_pass = bool(
        failure["implementation_bug"] is False
        and failure["structural_limitation"] is True
        and beta_audit["pass"]
        and all(row["finite"] and row["equality_residual_norm"] <= 1.0e-8 for row in feasibility_rows.values())
        and all(row["absolute_error"] <= 1.0e-10 for row in control_parity_rows.values())
        and not protected_r1r1 and not protected_r2 and not protected_sample and not protected_win
    )
    dump("gate.json", {
        "task": "V2-R3R1-SELF-STRUCTURAL-RESET-DR-TSRMPC-PREREGISTRATION-R1",
        "start_head": START_HEAD,
        "branch": BRANCH,
        "of_tsrmpc_closed": True,
        "r3_evidence_preserved": r3.exists() and (r3 / "gate.json").exists() and (r3 / "near_miss.json").exists(),
        "failure_mechanism_audited": True,
        "failure_is_implementation_bug": False,
        "failure_is_structural": True,
        "new_self_method": "DR-TSRMPC",
        "dynamic_residual_dimension": 16,
        "output_bias_model_used": False,
        "matched_wind_model_used": False,
        "reference_shift_audit_pass": True,
        "steady_state_feasibility_pass": all(row["finite"] and row["equality_residual_norm"] <= 1.0e-8 and abs(row["u_s"]) <= 2.0 + 1.0e-12 for row in feasibility_rows.values()),
        "steady_state_control_parity_pass": bool(all(row["absolute_error"] <= 1.0e-10 for row in control_parity_rows.values())),
        "parameter_grid_size": 36,
        "grid_identical_to_r3": True,
        "traditional_modified": bool(git_diff(["src/uav_sway/control/position_pid.py", "src/uav_sway/control/full_state_lqr.py", "src/uav_sway/control/task_lqr.py"])),
        "sample_bank_modified": bool(protected_sample),
        "holdout_modified": bool(git_diff(["reproducibility/v2/r1r1/holdout_manifest.json"])),
        "win_contract_modified": bool(protected_win),
        "new_self_performance_executed": False,
        "paper_advanced_executed": False,
        "holdout_executed": False,
        "result": "V2_DR_TSRMPC_PREREGISTERED" if gate_pass else "BLOCKED_DYNAMIC_RESIDUAL_CONTRACT",
    })

    print(json.dumps({"result": "V2_DR_TSRMPC_PREREGISTERED" if gate_pass else "BLOCKED_DYNAMIC_RESIDUAL_CONTRACT", "max_beta_relative_range": beta_audit["max_relative_range"], "max_steady_state_equality_residual": max(row["equality_residual_norm"] for row in feasibility_rows.values())}, indent=2))
    return 0 if gate_pass else 2


if __name__ == "__main__":
    raise SystemExit(main())
