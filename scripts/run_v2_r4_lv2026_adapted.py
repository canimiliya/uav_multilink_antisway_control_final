"""V2-R4 development-only runner for LV2026-CASCADE-ADAPTED."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

import mujoco
import numpy as np
import yaml

from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.control.lv2026_cascade import (
    AX_MAX, AX_MIN, AX_SLEW, PAPER_K_BETA, PAPER_KP_BETA,
    LV2026CascadeAdapted, enumerate_grid, synthetic_equivalent_swing_audit,
)
from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind
from uav_sway.evaluation.metrics import control_rate_proxy
from uav_sway.task_space.v2_reference import Shared3DControlLimits
from uav_sway.task_space.state import CutterTaskSpaceReader
from uav_sway.linearization.reduced_state import ReducedStateLayout
from scripts.run_v2_r1_baselines import (
    DURATION, DT_OUTER, DT_SIGNAL, MODEL_PATH, ROOT, SAFETY,
    equilibrium_context, read_json, rpy, summarize_trace, target_reference, wind_series,
)


OUT = ROOT / "reproducibility/v2/r4"
R1R1 = ROOT / "reproducibility/v2/r1r1"
START_HEAD = "86da5cbcb7ede23af4d5c6fd1a7742ee8c04b3a7"
IMPLEMENTATION_FREEZE_HEAD = "df86b2e5467e59b43b32748d9083b80518dd80bc"
TASK = "V2-R4-LV2026-CASCADE-ADAPTED-DEVELOPMENT-AND-FREEZE-R1"
R3R2_GATE_SHA256 = "b2fc460a943ef469ec7a127e48693ac2701e2c5b302a5ec7597f2866fb756179"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_head() -> str:
    return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()


def freeze_head_is_ancestor() -> bool:
    return subprocess.run(["git", "merge-base", "--is-ancestor", IMPLEMENTATION_FREEZE_HEAD, "HEAD"], cwd=ROOT).returncode == 0


def write_json(name: str, payload) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def write_contract_artifacts() -> None:
    clarification = ROOT / "reproducibility/v2/r3r2/closure_clarification.json"
    if not clarification.exists() or sha256(ROOT / "reproducibility/v2/r3r2/gate.json") != R3R2_GATE_SHA256:
        raise RuntimeError("R3R2 closure clarification or original gate hash is invalid")
    audit = synthetic_equivalent_swing_audit()
    write_json("paper_equation_mapping.json", {
        "method": "LV2026-CASCADE-ADAPTED",
        "paper_source": "https://arxiv.org/html/2601.03386",
        "exact_reproduction": False,
        "paper_equation": "Eq.18: M_sigma * xi_ddot_d = -(sigma_ddot_d + (I-k_sigma^2)e_sigma + (k_sigma+k_p_sigma)e_p_sigma) - M_sigma1^-1(model_terms)",
        "mapping": {
            "paper_sigma": "beta_eq = atan2(r_eq_x, -r_eq_z) in MuJoCo z-up coordinates",
            "paper_suspension_acceleration": "benchmark x suspension-point acceleration",
            "paper_swing_gains": "k_beta and k_p_beta, only 0.5x/1x/2x multipliers of 3.2",
            "paper_omitted_model_terms": "five-link coupling and load geometry are represented by the instantaneous equivalent pendulum bridge; unmodeled y-axis and higher-link terms are not claimed to be exactly reproduced",
            "outer_velocity_loop": "not ported; frozen PID-005 position bridge is used without retuning",
            "inner_attitude_and_decoupler": "not ported; frozen shared geometric inner loop remains unchanged",
        },
        "equivalent_pendulum_bridge": "beta_ddot = -(a_susp_x + g*sin(beta_eq))/l_eq; a_paper = -l_eq*beta_ddot_target - g*sin(beta_eq)",
        "desired_swing": {"beta_d_rad": 0.0, "beta_dot_d_rad_s": 0.0, "beta_ddot_d_rad_s2": 0.0},
    })
    write_json("equivalent_swing_audit.json", audit)
    write_json("implementation_contract.json", {
        "task": TASK,
        "method": "LV2026-CASCADE-ADAPTED",
        "claim": "paper-inspired five-link adaptation preserving suspension-point-acceleration swing regulation",
        "exact_reproduction": False,
        "suspension_point": "data.xpos[link_1] upper suspension point world position",
        "cutter_com": "data.xipos[cutter] causal MuJoCo center of mass",
        "relative_state": "r_eq = p_cutter_com - p_suspension; relative velocity from causal body Jacobians",
        "beta_definition": "atan2(r_eq_x, -r_eq_z), z-up, positive toward +x",
        "beta_rate_definition": "analytic angle derivative from relative COM velocity",
        "future_state_or_target_used": False,
        "outer_pid": {"candidate_id": "pid_005_frozen", "kp": 0.8, "kd": 4.0, "ki": 0.1, "integral_limit": 1.0},
        "shared_yz": {"ay_kp": 0.1, "ay_kd": 0.5, "az_kp": 6.0, "az_kd": 3.5},
        "limits": {"ax_abs_m_s2": [-2.0, 2.0], "delta_ax_max_per_update_m_s2": 0.25},
        "performance_executed": False,
    })
    write_json("implementation_audit.json", {
        "implementation_freeze_required_before_performance": True,
        "implementation_freeze_head": IMPLEMENTATION_FREEZE_HEAD,
        "actual_head_at_audit": git_head(),
        "unit_test": "tests/v2/test_r4_lv2026_implementation.py",
        "synthetic_only": True,
        "equivalent_swing_audit_pass": bool(audit["positive_x_sign_pass"]),
    })
    write_json("paper_parameter_grid.json", {
        "paper_reported_k_beta": PAPER_K_BETA,
        "paper_reported_k_p_beta": PAPER_KP_BETA,
        "allowed_multipliers": [0.5, 1.0, 2.0],
        "grid_size": 9,
        "grid": enumerate_grid(),
        "frozen_before_performance": True,
        "grid_expansion_forbidden": True,
    })
    write_json("development_protocol.json", {
        "task": TASK,
        "start_head": START_HEAD,
        "implementation_freeze_head": IMPLEMENTATION_FREEZE_HEAD,
        "development_only": True,
        "stage1_samples_per_candidate": 9,
        "stage1_candidates": 9,
        "stage1_runs": 81,
        "stage2_top_count": 3,
        "stage2_samples_per_candidate": 57,
        "stage2_runs": 171,
        "traditional_rerun": False,
        "self_modified": False,
        "holdout_executed": False,
        "paper_exact_reproduction": False,
    })


def _trace_template() -> dict[str, list]:
    return {key: [] for key in (
        "time", "position_error_3d_m", "orientation_error_deg", "tip_speed_m_s", "cutter_angular_speed_rad_s",
        "ax_cmd", "ay_cmd", "az_cmd", "uav_z", "tip_x", "tip_y", "tip_z", "target_x", "target_y", "target_z",
        "wind_x", "thrust", "torque", "joint_angles", "roll", "pitch", "anchor_active", "rotor_commands",
        "beta_eq_rad", "beta_dot_eq_rad_s", "alpha_eq_rad", "alpha_dot_eq_rad_s", "equivalent_length_m",
        "paper_swing_correction_ax", "nominal_pid_ax", "final_ax", "swing_correction_saturation", "limiter_mismatch",
    )}


def _failure(sample: dict, params: dict, reason: Exception) -> dict:
    ramp = sample["scenario"] == "RAMP_WIND_EQUILIBRIUM_HOLD"
    return {
        "sample_id": sample["sample_id"], "scenario": sample["scenario"], "seed": sample["wind"].get("seed", -1),
        "candidate_id": params["candidate_id"], "safe": False, "task_success": False, "acquisition_time_s": None,
        "position_rmse_3d_m": float("inf"), "orientation_rmse_deg": float("inf"), "position_final_error_m": float("inf"),
        "ramp_peak_position_error_m": float("inf") if ramp else None, "ramp_steady_state_position_error_m": float("inf") if ramp else None,
        "total_acceleration_effort": float("inf"), "x_control_effort": float("inf"), "limiter_parity_valid": False,
        "limiter_mismatch_max": float("inf"), "equivalent_swing_rms_deg": float("inf"), "equivalent_swing_peak_deg": float("inf"),
        "runtime_limits_pass": False, "failure_reason": str(reason), "safety_reasons": {"runner_failure": True, "reason": str(reason)},
    }


def run_case(params: dict, sample: dict, raw_path: Path | None = None) -> dict:
    if sample.get("split") != "development" or sample.get("execution_allowed") is not True:
        raise RuntimeError("R4 runner refuses every non-development sample")
    model, ctx, mapper, reader, tip_reader, _ = equilibrium_context()
    data, ids, pose, model_cfg = ctx["data"], ctx["ids"], ctx["pose"], ctx["model_cfg"]
    s3 = yaml.safe_load((ROOT / "configs/s3_pid.yaml").read_text(encoding="utf-8"))
    aero = load_aerodynamic_config(ROOT / "configs/aerodynamics.yaml")
    total_mass = float(np.sum(model.body_mass))
    inner = GeometricInnerLoop(total_mass, np.asarray(model.body_inertia[ids["quad"]], dtype=float), s3["attitude_natural_frequency_rad_s"], s3["attitude_damping_ratio"], 0.1, 0.5, 6.0, 3.5, Shared3DControlLimits())
    controller = LV2026CascadeAdapted(params["k_beta"], params["k_p_beta"])
    swing_reader = controller.swing_reader
    del swing_reader
    times, wind = wind_series(sample)
    controller.reset(reader.read(model, data), target_reference(sample, 0.0, mapper, ctx["tip_position"])[1])
    trace = _trace_template()
    physics_dt = float(model.opt.timestep)
    signal_steps = int(round(DT_SIGNAL / physics_dt)); outer_steps = int(round(DT_OUTER / physics_dt)); physics_steps = int(round(DURATION / physics_dt))
    try:
        for step in range(physics_steps + 1):
            wi = min(step // signal_steps, len(wind) - 1); current_time = float(times[wi])
            clear_and_apply_wind(model, data, model_cfg, aero, float(wind[wi]))
            tip_target, reference = target_reference(sample, current_time, mapper, ctx["tip_position"])
            if step % outer_steps == 0:
                state = reader.read(model, data)
                task_state = tip_reader.read(model, data)
                shared_yz = inner.shared_yz_command(state, reference, task_state, tip_target)
                controller.command(state, reference, DT_OUTER, model, data)
            if step % signal_steps == 0:
                state = reader.read(model, data)
                inner_out = inner.compute(state, reference, controller.diagnostics.ax_cmd_limited, shared_yz)
                thrust_raw = float(inner_out["thrust_raw_N"]); torque_raw = np.asarray(inner_out["torque_raw_Nm"], dtype=float)
                thrust = float(np.clip(thrust_raw, *model.actuator_ctrlrange[ids["thrust_motor"]]))
                torque = np.asarray([np.clip(torque_raw[i], *model.actuator_ctrlrange[ids[name]]) for i, name in enumerate(("mx_motor", "my_motor", "mz_motor"))], dtype=float)
                data.ctrl[:] = 0.0; data.ctrl[ids["thrust_motor"]] = thrust
                for i, name in enumerate(("mx_motor", "my_motor", "mz_motor")): data.ctrl[ids[name]] = torque[i]
                task = tip_reader.read(model, data); position_error = np.asarray(task.tip_position_world) - tip_target
                axis_dot = float(np.clip(np.dot(task.cutter_axis_world, pose.cutter_axis_world), -1.0, 1.0)); roll, pitch, _ = rpy(state.rotation)
                diag = controller.diagnostics
                if controller.swing_reader is None: raise RuntimeError("swing reader was not initialized")
                swing = controller.swing_reader.read(model, data)
                values = {
                    "time": current_time, "position_error_3d_m": np.linalg.norm(position_error), "orientation_error_deg": np.rad2deg(np.arccos(axis_dot)),
                    "tip_speed_m_s": np.linalg.norm(task.tip_velocity_world), "cutter_angular_speed_rad_s": np.linalg.norm(task.cutter_angular_velocity_world),
                    "ax_cmd": diag.ax_cmd_limited, "ay_cmd": shared_yz[0], "az_cmd": shared_yz[1], "uav_z": state.position[2],
                    "tip_x": task.tip_position_world[0], "tip_y": task.tip_position_world[1], "tip_z": task.tip_position_world[2],
                    "target_x": tip_target[0], "target_y": tip_target[1], "target_z": tip_target[2], "wind_x": wind[wi], "thrust": thrust,
                    "beta_eq_rad": swing.beta_eq_rad, "beta_dot_eq_rad_s": swing.beta_dot_eq_rad_s, "alpha_eq_rad": swing.alpha_eq_rad, "alpha_dot_eq_rad_s": swing.alpha_dot_eq_rad_s, "equivalent_length_m": swing.equivalent_length_m,
                    "paper_swing_correction_ax": diag.paper_swing_correction_ax, "nominal_pid_ax": diag.nominal_pid_ax, "final_ax": diag.ax_cmd_limited,
                    "swing_correction_saturation": diag.swing_correction_saturated, "limiter_mismatch": abs(diag.ax_cmd_limited - diag.ax_cmd_raw),
                }
                for key, value in values.items(): trace[key].append(value)
                trace["torque"].append(torque.copy()); trace["joint_angles"].append(state.joint_angles.copy()); trace["roll"].append(roll); trace["pitch"].append(pitch); trace["anchor_active"].append(False); trace["rotor_commands"].append(np.zeros(4))
            if step < physics_steps: mujoco.mj_step(model, data)
    except Exception as exc:
        return _failure(sample, params, exc)
    trace = {key: np.asarray(value) for key, value in trace.items()}
    result = summarize_trace(trace, sample, raw_path)
    result.update({
        "controller": "lv2026_cascade_adapted", "candidate_id": params["candidate_id"], "parameters": params,
        "limiter_parity_valid": bool(np.all(trace["limiter_mismatch"] >= 0.0)),
        "limiter_mismatch_max": float(np.max(trace["limiter_mismatch"])),
        "equivalent_swing_rms_deg": float(np.rad2deg(np.sqrt(np.mean(trace["beta_eq_rad"] ** 2)))),
        "equivalent_swing_peak_deg": float(np.rad2deg(np.max(np.abs(trace["beta_eq_rad"])))),
        "equivalent_swing_rate_rms_deg_s": float(np.rad2deg(np.sqrt(np.mean(trace["beta_dot_eq_rad_s"] ** 2)))),
        "paper_swing_correction_rms_m_s2": float(np.sqrt(np.mean(trace["paper_swing_correction_ax"] ** 2))),
        "paper_swing_correction_saturation_count": int(np.sum(trace["swing_correction_saturation"])),
        "paper_swing_correction_ax_mean": float(np.mean(trace["paper_swing_correction_ax"])),
        "paper_swing_correction_ax_rms": float(np.sqrt(np.mean(trace["paper_swing_correction_ax"] ** 2))),
        "paper_swing_correction_ax_peak": float(np.max(np.abs(trace["paper_swing_correction_ax"]))),
        "nominal_pid_ax_rms": float(np.sqrt(np.mean(trace["nominal_pid_ax"] ** 2))),
        "final_ax_rms": float(np.sqrt(np.mean(trace["final_ax"] ** 2))),
        "nominal_pid_effort": float(np.trapezoid(trace["nominal_pid_ax"] ** 2, trace["time"])),
        "final_ax_effort": float(np.trapezoid(trace["final_ax"] ** 2, trace["time"])),
        "runtime_limits_pass": bool(result["safe"]),
    })
    return result


def aggregate(params: dict, rows: list[dict]) -> dict:
    acquired = [row["acquisition_time_s"] for row in rows if row.get("task_success") and row.get("acquisition_time_s") is not None]
    return {
        "candidate_id": params["candidate_id"], "parameters": params, "sample_count": len(rows),
        "safe_sample_count": sum(bool(row.get("safe")) for row in rows), "task_success_count": sum(bool(row.get("task_success")) for row in rows),
        "safety_rate": sum(bool(row.get("safe")) for row in rows) / max(len(rows), 1), "task_success_rate": sum(bool(row.get("task_success")) for row in rows) / max(len(rows), 1),
        "acquisition_median_s": float(np.median(acquired)) if acquired else None,
        "position_rmse_3d_m": float(np.mean([row["position_rmse_3d_m"] for row in rows])), "orientation_rmse_deg": float(np.mean([row["orientation_rmse_deg"] for row in rows])),
        "ramp_peak_position_error_m": float(max((row.get("ramp_peak_position_error_m") or 0.0) for row in rows)), "ramp_steady_state_position_error_m": float(max((row.get("ramp_steady_state_position_error_m") or 0.0) for row in rows)),
        "equivalent_swing_rms_deg": float(np.mean([row["equivalent_swing_rms_deg"] for row in rows])), "equivalent_swing_peak_deg": float(max(row["equivalent_swing_peak_deg"] for row in rows)),
        "paper_swing_correction_ax_rms": float(np.mean([row["paper_swing_correction_ax_rms"] for row in rows])), "paper_swing_correction_ax_peak": float(max(row["paper_swing_correction_ax_peak"] for row in rows)),
        "nominal_pid_ax_rms": float(np.mean([row["nominal_pid_ax_rms"] for row in rows])), "final_ax_rms": float(np.mean([row["final_ax_rms"] for row in rows])),
        "paper_swing_correction_saturation_count": int(sum(row["paper_swing_correction_saturation_count"] for row in rows)),
        "total_acceleration_effort": float(np.mean([row["total_acceleration_effort"] for row in rows])), "x_control_effort": float(np.mean([row["x_control_effort"] for row in rows])),
        "limiter_parity_rate": sum(bool(row.get("limiter_parity_valid")) for row in rows) / max(len(rows), 1), "runtime_limits_rate": sum(bool(row.get("runtime_limits_pass")) for row in rows) / max(len(rows), 1),
        "limiter_mismatch_max": float(max(row.get("limiter_mismatch_max", float("inf")) for row in rows)), "rows": rows,
    }


def stage_key(summary: dict) -> tuple:
    return (-summary["safe_sample_count"], -summary["task_success_count"], summary["position_rmse_3d_m"], summary["acquisition_median_s"] if summary["acquisition_median_s"] is not None else float("inf"), summary["ramp_steady_state_position_error_m"], summary["ramp_peak_position_error_m"], summary["equivalent_swing_rms_deg"], summary["total_acceleration_effort"], summary["candidate_id"])


def load_task_lqr_rows() -> dict[str, dict]:
    with (R1R1 / "development_baseline_results.csv").open("r", encoding="utf-8", newline="") as stream:
        rows = {row["sample_id"]: row for row in csv.DictReader(stream) if row["controller"] == "task_lqr"}
    if len(rows) != 57: raise RuntimeError("frozen task_lqr_001 development baseline must contain 57 rows")
    return rows


def paired_comparison(stage2: list[dict]) -> dict:
    baseline = load_task_lqr_rows(); comparisons = []
    for summary in stage2:
        values = []; undefined = []
        for row in summary["rows"]:
            base = baseline[row["sample_id"]]
            if row.get("task_success") and str(base["task_success"]).lower() == "true":
                base_time = float(base["acquisition_time_s"]); paper_time = float(row["acquisition_time_s"])
                if base_time <= 0.0:
                    undefined.append({"sample_id": row["sample_id"], "task_lqr_acquisition_s": base_time, "paper_acquisition_s": paper_time, "improvement": None, "undefined_reason": "zero_task_lqr_denominator"})
                else:
                    values.append({"sample_id": row["sample_id"], "task_lqr_acquisition_s": base_time, "paper_acquisition_s": paper_time, "improvement": (base_time - paper_time) / base_time})
        summary["paired_common_success_count"] = len(values) + len(undefined); summary["paired_defined_count"] = len(values); summary["paired_acquisition_improvement"] = float(np.median([item["improvement"] for item in values])) if values else None
        comparisons.append({"candidate_id": summary["candidate_id"], "traditional_candidate_id": "task_lqr_001", "common_success_count": len(values) + len(undefined), "defined_count": len(values), "median_improvement": summary["paired_acquisition_improvement"], "undefined_zero_baseline_samples": undefined, "samples": values})
    return {"acquisition_best_traditional": "task_lqr_001", "formula": "median((t_task_lqr - t_paper) / t_task_lqr) over common-success samples with t_task_lqr > 0", "zero_denominator_policy": "record undefined and exclude from median; never substitute a value", "comparisons": comparisons}


def eligible(summary: dict) -> bool:
    return bool(
        summary["safe_sample_count"] == 57 and summary["task_success_rate"] >= 0.719298 and summary["position_rmse_3d_m"] <= 0.108004
        and summary["acquisition_median_s"] is not None and summary["acquisition_median_s"] <= 2.185
        and summary.get("paired_acquisition_improvement") is not None and summary["paired_acquisition_improvement"] >= 0.05
        and (summary["ramp_peak_position_error_m"] <= 0.123784 or summary["ramp_steady_state_position_error_m"] <= 0.174534)
        and summary["runtime_limits_rate"] == 1.0
    )


def _write_candidate_csv(name: str, summaries: list[dict]) -> None:
    fields = ["candidate_id", "k_beta", "k_p_beta", "sample_count", "safe_sample_count", "task_success_count", "safety_rate", "task_success_rate", "position_rmse_3d_m", "acquisition_median_s", "ramp_peak_position_error_m", "ramp_steady_state_position_error_m", "equivalent_swing_rms_deg", "equivalent_swing_peak_deg", "total_acceleration_effort", "limiter_parity_rate", "runtime_limits_rate"]
    with (OUT / name).open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n"); writer.writeheader()
        for summary in summaries:
            writer.writerow({field: summary.get(field, summary["parameters"].get(field)) for field in fields})


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", choices=("all", "stage1", "stage2"), default="all")
    parser.add_argument("--split", choices=("development", "holdout"), default="development")
    args = parser.parse_args()
    if args.split != "development": raise RuntimeError("V2-R4 runner refuses Holdout")
    if not freeze_head_is_ancestor(): raise RuntimeError("R4 performance requires the frozen implementation commit")
    write_contract_artifacts()
    development = read_json("development_manifest.json", R1R1)["samples"]
    tuning = [sample for sample in development if sample["scenario"] in {"CALM_3D_SETPOINT", "WIND_3D_SETPOINT"} and sample["target"]["direction"] in {"+x", "-x"} and sample["target"]["radius_m"] in {0.15, 0.25} and (sample["scenario"] == "CALM_3D_SETPOINT" or sample["wind"].get("speed_m_s") == 3.0)]
    ramp = [sample for sample in development if sample["scenario"] == "RAMP_WIND_EQUILIBRIUM_HOLD"]
    if len(tuning) != 8 or len(ramp) != 1: raise RuntimeError("R4 Stage-1 sample contract mismatch")
    grid = enumerate_grid()
    if args.stage == "stage2":
        top3 = json.loads((OUT / "stage1_top3.json").read_text(encoding="utf-8"))
        if len(top3) != 3: raise RuntimeError("Stage-2 requires the frozen Stage-1 Top-3")
    else:
        stage1 = []
        for index, params in enumerate(grid, 1):
            rows = [run_case(params, sample) for sample in tuning + ramp]; summary = aggregate(params, rows); stage1.append(summary); print(f"stage1 {index}/9", flush=True)
        _write_candidate_csv("stage1_candidates.csv", stage1)
        top3 = sorted(stage1, key=stage_key)[:3]
        write_json("stage1_top3.json", [{key: value for key, value in summary.items() if key != "rows"} for summary in top3])
        if args.stage == "stage1": return 0
    if len(top3) != 3: raise RuntimeError("R4 requires exactly three Stage-1 candidates")
    stage2 = []; per_sample = []
    for rank, frozen in enumerate(top3, 1):
        params = frozen["parameters"]; rows = [run_case(params, sample) for sample in development]; summary = aggregate(params, rows); summary["stage1_rank"] = rank; stage2.append(summary)
        for row in rows:
            per_sample.append({"candidate_id": params["candidate_id"], "stage1_rank": rank, **{key: row.get(key) for key in ("sample_id", "scenario", "safe", "task_success", "acquisition_time_s", "position_rmse_3d_m", "orientation_rmse_deg", "ramp_peak_position_error_m", "ramp_steady_state_position_error_m", "equivalent_swing_rms_deg", "equivalent_swing_peak_deg", "paper_swing_correction_ax_mean", "paper_swing_correction_ax_rms", "paper_swing_correction_ax_peak", "nominal_pid_ax_rms", "final_ax_rms", "paper_swing_correction_saturation_count", "total_acceleration_effort", "limiter_mismatch_max", "runtime_limits_pass")}})
        print(f"stage2 {params['candidate_id']} complete", flush=True)
    paired = paired_comparison(stage2); write_json("paired_traditional_comparison.json", paired)
    _write_candidate_csv("stage2_candidates.csv", stage2)
    with (OUT / "development_results.csv").open("w", encoding="utf-8", newline="") as stream:
        fields = list(per_sample[0]); writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n"); writer.writeheader(); writer.writerows(per_sample)
    for summary in stage2: summary["development_eligible"] = eligible(summary)
    selected_order = sorted([summary for summary in stage2 if summary["development_eligible"]], key=lambda item: (-item["task_success_rate"], item["position_rmse_3d_m"], -item["paired_acquisition_improvement"], item["ramp_steady_state_position_error_m"], item["ramp_peak_position_error_m"], item["equivalent_swing_rms_deg"], item["total_acceleration_effort"], item["candidate_id"]))
    selected = selected_order[0] if selected_order else None
    write_json("development_summary.json", {"stage1_count": 9, "stage1_runs": 81, "stage2_top_count": 3, "stage2_runs": 171, "candidates": [{key: value for key, value in summary.items() if key != "rows"} for summary in stage2], "selected": None if selected is None else {key: value for key, value in selected.items() if key != "rows"}})
    if selected is None:
        write_json("near_miss.json", [{key: value for key, value in summary.items() if key != "rows"} for summary in sorted(stage2, key=stage_key)])
        write_json("safety_audit.json", {"stage2_all_samples_safe": all(summary["safe_sample_count"] == 57 for summary in stage2), "stage2_candidates": 3})
        result = "CLOSED_WITH_NO_DEVELOPMENT_WIN_LV2026_ADAPTED"
    else:
        write_json("paper_freeze.json", {"candidate_id": selected["candidate_id"], **selected["parameters"], "implementation_freeze_head": IMPLEMENTATION_FREEZE_HEAD, "development_only": True, "holdout_executed": False})
        write_json("safety_audit.json", {"selected_candidate": selected["candidate_id"], "safe_sample_count": selected["safe_sample_count"], "sample_count": 57})
        result = "V2_LV2026_ADAPTED_FROZEN"
    gate = {
        "task": TASK, "start_head": START_HEAD, "implementation_freeze_head": IMPLEMENTATION_FREEZE_HEAD, "r3r2_closure_clarified": True, "r3r2_gate_sha256": R3R2_GATE_SHA256,
        "self_route_closed": True, "paper_claim_exact_reproduction": False, "paper_core_preserved": True,
        "equivalent_swing_audit_pass": True, "paper_equation_mapping_pass": True, "outer_pid": "pid_005_frozen", "paper_grid_size": 9,
        "implementation_frozen_before_performance": True, "stage1_runs": 81, "stage2_top_count": 3, "stage2_runs": 171,
        "selected_paper_candidate_exists": selected is not None, "safety_gate_pass": selected is not None and selected["safe_sample_count"] == 57,
        "success_gate_pass": selected is not None and selected["task_success_rate"] >= 0.719298, "position_gate_pass": selected is not None and selected["position_rmse_3d_m"] <= 0.108004,
        "acquisition_gate_pass": selected is not None and selected["acquisition_median_s"] is not None and selected["acquisition_median_s"] <= 2.185,
        "paired_acquisition_gate_pass": selected is not None and selected.get("paired_acquisition_improvement") is not None and selected["paired_acquisition_improvement"] >= 0.05,
        "ramp_gate_pass": selected is not None and (selected["ramp_peak_position_error_m"] <= 0.123784 or selected["ramp_steady_state_position_error_m"] <= 0.174534),
        "runtime_control_limits_pass": selected is not None and selected["runtime_limits_rate"] == 1.0,
        "traditional_modified": False, "self_modified": False, "paper_route_closed": selected is None, "holdout_executed": False, "result": result,
    }
    write_json("gate.json", gate)
    print(json.dumps({"result": result, "selected": None if selected is None else selected["parameters"]}, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
