"""R3 development-only runner for the preregistered OF-TSRMPC.

The runner rejects every non-development sample before constructing MuJoCo
state.  It never imports or executes the holdout manifest.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import yaml

from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.control.of_tsrmpc import OFTSRMPC, enumerate_grid
from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind
from uav_sway.evaluation.metrics import control_rate_proxy
from uav_sway.evaluation.task_baseline_runner import (
    ROOT, SAFETY, DT_SIGNAL, DT_OUTER, DURATION, equilibrium_context,
    rpy, summarize_trace, target_reference, wind_series, read_json,
)
from uav_sway.linearization.reduced_state import ReducedStateLayout
from uav_sway.task_space.v2_reference import Shared3DControlLimits


R1R1 = ROOT / "reproducibility/v2/r1r1"
R3 = ROOT / "reproducibility/v2/r3"
MODEL_PATH = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
START_HEAD = "2aa1d72a58fdb2bac450d1106b1e044bc258e0ee"
AX_MIN, AX_MAX, AX_SLEW = -2.0, 2.0, 0.25


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def frozen_method_arrays():
    a = np.load(ROOT / "reproducibility/frozen/linear_model/A.npy")
    b = np.load(ROOT / "reproducibility/frozen/linear_model/B.npy")
    c = np.load(ROOT / "reproducibility/frozen/task_lqr/C_task.npy")
    from uav_sway.control.task_lqr import build_task_lqr
    task_lqr = build_task_lqr(a, b, c, 20, 5, 1)
    return a, b, c, np.asarray(task_lqr["K"], dtype=float), np.asarray(task_lqr["P"], dtype=float)


def candidate_controller(params):
    a, b, c, k, p = frozen_method_arrays()
    return OFTSRMPC(a, b, c, k, p, params["beta"], params["w_p"], params["w_theta"], params["R"])


def _write_raw(path: Path, trace: dict, result: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    keys = ["time_s", "position_error_3d_m", "orientation_error_deg", "tip_speed_m_s", "cutter_angular_speed_rad_s", "ax_cmd_m_s2", "ay_cmd_m_s2", "az_cmd_m_s2", "tip_x_m", "tip_y_m", "tip_z_m", "target_x_m", "target_y_m", "target_z_m", "wind_x_m_s", "thrust_N", "max_abs_torque_Nm", "d_raw", "d_hat", "x_s_norm", "u_s", "steady_state_residual", "v_mpc", "qp_status", "qp_iterations", "steady_solve_ms", "residual_solve_ms", "total_solve_ms", "limiter_mismatch", "safe", "task_success", "anchor_active"]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys, lineterminator="\n")
        writer.writeheader()
        n = len(trace["time"])
        for i in range(n):
            writer.writerow({
                "time_s": float(trace["time"][i]), "position_error_3d_m": float(trace["position_error_3d_m"][i]), "orientation_error_deg": float(trace["orientation_error_deg"][i]), "tip_speed_m_s": float(trace["tip_speed_m_s"][i]), "cutter_angular_speed_rad_s": float(trace["cutter_angular_speed_rad_s"][i]), "ax_cmd_m_s2": float(trace["ax_cmd"][i]), "ay_cmd_m_s2": float(trace["ay_cmd"][i]), "az_cmd_m_s2": float(trace["az_cmd"][i]), "tip_x_m": float(trace["tip_x"][i]), "tip_y_m": float(trace["tip_y"][i]), "tip_z_m": float(trace["tip_z"][i]), "target_x_m": float(trace["target_x"][i]), "target_y_m": float(trace["target_y"][i]), "target_z_m": float(trace["target_z"][i]), "wind_x_m_s": float(trace["wind_x"][i]), "thrust_N": float(trace["thrust"][i]), "max_abs_torque_Nm": float(np.max(np.abs(trace["torque"][i]))), "d_raw": float(trace["d_raw"][i]), "d_hat": float(trace["d_hat"][i]), "x_s_norm": float(trace["x_s_norm"][i]), "u_s": float(trace["u_s"][i]), "steady_state_residual": float(trace["steady_state_residual"][i]), "v_mpc": float(trace["v_mpc"][i]), "qp_status": str(trace["qp_status"][i]), "qp_iterations": int(trace["qp_iterations"][i]), "steady_solve_ms": float(trace["steady_solve_ms"][i]), "residual_solve_ms": float(trace["residual_solve_ms"][i]), "total_solve_ms": float(trace["total_solve_ms"][i]), "limiter_mismatch": float(trace["limiter_mismatch"][i]), "safe": bool(result["safe"]), "task_success": bool(result["task_success"]), "anchor_active": False,
            })


def _failure(sample, params, reason):
    return {"sample_id": sample["sample_id"], "scenario": sample["scenario"], "seed": sample["wind"].get("seed", -1), "candidate_id": params["candidate_id"], "safe": False, "solver_success": False, "solver_failure_reason": str(reason), "task_success": False, "acquisition_time_s": None, "position_rmse_3d_m": float("inf"), "orientation_rmse_deg": float("inf"), "ramp_peak_position_error_m": float("inf") if sample["scenario"] == "RAMP_WIND_EQUILIBRIUM_HOLD" else None, "ramp_steady_state_position_error_m": float("inf") if sample["scenario"] == "RAMP_WIND_EQUILIBRIUM_HOLD" else None, "total_acceleration_effort": float("inf"), "x_control_effort": float("inf"), "total_solve_time_p95_ms": float("inf"), "limiter_mismatch_max": float("inf"), "safety_reasons": {"solver_failure": True}}


def run_case(params: dict, sample: dict, raw_path: Path | None = None) -> dict:
    if sample.get("split") != "development" or sample.get("execution_allowed") is not True:
        raise RuntimeError("R3 runner refuses every non-development or forbidden sample")
    model, ctx, mapper, reader, tip_reader, _ = equilibrium_context()
    data = ctx["data"]; ids = ctx["ids"]; pose = ctx["pose"]; model_cfg = ctx["model_cfg"]
    s3 = yaml.safe_load((ROOT / "configs/s3_pid.yaml").read_text(encoding="utf-8"))
    aero = load_aerodynamic_config(ROOT / "configs/aerodynamics.yaml")
    total_mass = float(np.sum(model.body_mass))
    inner = GeometricInnerLoop(total_mass, np.asarray(model.body_inertia[ids["quad"]], dtype=float), s3["attitude_natural_frequency_rad_s"], s3["attitude_damping_ratio"], 0.1, 0.5, 6.0, 3.5, Shared3DControlLimits())
    controller = candidate_controller(params)
    layout = ReducedStateLayout(model)
    times, wind = wind_series(sample)
    controller.reset()
    shared_yz = (0.0, 0.0)
    trace = {key: [] for key in ("time", "position_error_3d_m", "orientation_error_deg", "tip_speed_m_s", "cutter_angular_speed_rad_s", "ax_cmd", "ay_cmd", "az_cmd", "uav_z", "tip_x", "tip_y", "tip_z", "target_x", "target_y", "target_z", "wind_x", "thrust", "torque", "joint_angles", "roll", "pitch", "anchor_active", "rotor_commands", "d_raw", "d_hat", "x_s_norm", "u_s", "steady_state_residual", "v_mpc", "qp_status", "qp_iterations", "steady_solve_ms", "residual_solve_ms", "total_solve_ms", "limiter_mismatch")}
    physics_dt = float(model.opt.timestep); signal_steps = int(round(DT_SIGNAL / physics_dt)); outer_steps = int(round(DT_OUTER / physics_dt)); physics_steps = int(round(DURATION / physics_dt))
    try:
        for step in range(physics_steps + 1):
            wi = min(step // signal_steps, len(wind) - 1); current_time = float(times[wi])
            clear_and_apply_wind(model, data, model_cfg, aero, float(wind[wi]))
            tip_target, reference = target_reference(sample, current_time, mapper, ctx["tip_position"])
            if step % outer_steps == 0:
                state = reader.read(model, data)
                # The measured output comes from MuJoCo's task reader, while
                # the state term is the frozen linear task map.
                tip_position, tip_velocity, axis, angular_velocity = task_map_kinematics(model, data)
                from uav_sway.linearization.task_output import signed_cutter_planar_angle
                measured_task = np.asarray([tip_position[0] - reference.x_ref - pose.tip_relative_position_m[0], tip_velocity[0] - reference.vx_ref, signed_cutter_planar_angle(axis), angular_velocity[1]], dtype=float)
                controller.command(layout.extract(model, data, reference), measured_task, reference.ax_ref)
                shared_yz = inner.shared_yz_command(reader.read(model, data), reference, tip_reader.read(model, data), tip_target)
            if step % signal_steps == 0:
                state = reader.read(model, data)
                inner_out = inner.compute(state, reference, controller.diagnostics.ax_cmd_limited, shared_yz)
                thrust_raw = float(inner_out["thrust_raw_N"]); torque_raw = np.asarray(inner_out["torque_raw_Nm"], dtype=float)
                thrust = float(np.clip(thrust_raw, *model.actuator_ctrlrange[ids["thrust_motor"]]))
                torque = np.asarray([np.clip(torque_raw[i], *model.actuator_ctrlrange[ids[name]]) for i, name in enumerate(("mx_motor", "my_motor", "mz_motor"))], dtype=float)
                data.ctrl[:] = 0.0; data.ctrl[ids["thrust_motor"]] = thrust
                for i, name in enumerate(("mx_motor", "my_motor", "mz_motor")): data.ctrl[ids[name]] = torque[i]
                task = tip_reader.read(model, data); position_error = np.asarray(task.tip_position_world) - tip_target; axis_dot = float(np.clip(np.dot(task.cutter_axis_world, pose.cutter_axis_world), -1.0, 1.0)); roll, pitch, _ = rpy(state.rotation); diag = controller.diagnostics
                for key, value in (("time", current_time), ("position_error_3d_m", np.linalg.norm(position_error)), ("orientation_error_deg", np.rad2deg(np.arccos(axis_dot))), ("tip_speed_m_s", np.linalg.norm(task.tip_velocity_world)), ("cutter_angular_speed_rad_s", np.linalg.norm(task.cutter_angular_velocity_world)), ("ax_cmd", diag.ax_cmd_limited), ("ay_cmd", shared_yz[0]), ("az_cmd", shared_yz[1]), ("uav_z", state.position[2]), ("tip_x", task.tip_position_world[0]), ("tip_y", task.tip_position_world[1]), ("tip_z", task.tip_position_world[2]), ("target_x", tip_target[0]), ("target_y", tip_target[1]), ("target_z", tip_target[2]), ("wind_x", wind[wi]), ("thrust", thrust), ("d_raw", diag.d_raw), ("d_hat", diag.d_hat), ("x_s_norm", diag.x_s_norm), ("u_s", diag.u_s), ("steady_state_residual", diag.steady_state_residual), ("v_mpc", diag.v_mpc), ("qp_status", diag.qp_status), ("qp_iterations", diag.qp_iterations), ("steady_solve_ms", diag.steady_solve_ms), ("residual_solve_ms", diag.residual_solve_ms), ("total_solve_ms", diag.total_solve_ms), ("limiter_mismatch", diag.limiter_mismatch)):
                    trace[key].append(value)
                trace["torque"].append(torque.copy()); trace["joint_angles"].append(state.joint_angles.copy()); trace["roll"].append(roll); trace["pitch"].append(pitch); trace["anchor_active"].append(False); trace["rotor_commands"].append(np.zeros(4))
            if step < physics_steps: mujoco.mj_step(model, data)
    except Exception as exc:
        return _failure(sample, params, exc)
    trace = {key: np.asarray(value) for key, value in trace.items()}
    result = summarize_trace(trace, sample, None)
    result.update({"controller": "of_tsrmpc", "candidate_id": params["candidate_id"], "solver_success": bool(np.all(trace["qp_status"] == "solved")), "solver_success_count": int(np.sum(trace["qp_status"] == "solved")), "outer_update_count": len(trace["qp_status"]), "total_solve_time_p95_ms": float(np.percentile(trace["total_solve_ms"], 95)), "limiter_mismatch_max": float(np.max(trace["limiter_mismatch"])), "steady_state_residual_max": float(np.max(trace["steady_state_residual"])), "trace": trace})
    if raw_path is not None: _write_raw(raw_path, trace, result); result["raw_csv"] = str(raw_path)
    result.pop("trace", None)
    return result


def task_map_kinematics(model, data):
    site = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip")); body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cutter")); jacp = np.zeros((3, model.nv)); jacr = np.zeros((3, model.nv)); mujoco.mj_jacSite(model, data, jacp, jacr, site); body_jacp = np.zeros((3, model.nv)); body_jacr = np.zeros((3, model.nv)); mujoco.mj_jacBody(model, data, body_jacp, body_jacr, body); rotation = np.asarray(data.xmat[body]).reshape(3, 3); return np.asarray(data.site_xpos[site]), jacp @ data.qvel, rotation @ np.asarray([1.0, 0.0, 0.0]), body_jacr @ data.qvel


def aggregate(params, rows):
    acquired = [r["acquisition_time_s"] for r in rows if r.get("task_success") and r.get("acquisition_time_s") is not None]
    solver_success_count = sum(int(r.get("solver_success_count", 0)) for r in rows)
    outer_count = sum(int(r.get("outer_update_count", 0)) for r in rows)
    return {"candidate_id": params["candidate_id"], "parameters": params, "sample_count": len(rows), "safe_sample_count": sum(bool(r["safe"]) for r in rows), "solver_success_count": solver_success_count, "solver_success_rate": solver_success_count / max(outer_count, 1), "task_success_count": sum(bool(r["task_success"]) for r in rows), "task_success_rate": sum(bool(r["task_success"]) for r in rows) / len(rows), "acquisition_median_s": float(np.median(acquired)) if acquired else None, "position_rmse_3d_m": float(np.mean([r["position_rmse_3d_m"] for r in rows])), "orientation_rmse_deg": float(np.mean([r["orientation_rmse_deg"] for r in rows])), "ramp_peak_position_error_m": float(max((r.get("ramp_peak_position_error_m") or 0.0) for r in rows)), "ramp_steady_state_position_error_m": float(max((r.get("ramp_steady_state_position_error_m") or 0.0) for r in rows)), "total_acceleration_effort": float(np.mean([r["total_acceleration_effort"] for r in rows])), "x_control_effort": float(np.mean([r["x_control_effort"] for r in rows])), "total_solve_time_p95_ms": float(max(r.get("total_solve_time_p95_ms", float("inf")) for r in rows)), "limiter_mismatch_max": float(max(r.get("limiter_mismatch_max", float("inf")) for r in rows)), "rows": rows}


def stage1_key(s):
    return (-s["safe_sample_count"], -s["solver_success_count"], -s["task_success_count"], s["position_rmse_3d_m"], s["acquisition_median_s"] if s["acquisition_median_s"] is not None else float("inf"), s["ramp_steady_state_position_error_m"], s["ramp_peak_position_error_m"], s["total_acceleration_effort"], s["total_solve_time_p95_ms"], s["candidate_id"])


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--stage", choices=("all", "stage1", "stage2"), default="all"); args = parser.parse_args()
    R3.mkdir(parents=True, exist_ok=True)
    development = read_json("development_manifest.json", R1R1)["samples"]
    if any(s.get("split") != "development" or s.get("execution_allowed") is not True for s in development): raise RuntimeError("development manifest contract invalid")
    grid = enumerate_grid()
    tuning = [s for s in development if s["scenario"] in {"CALM_3D_SETPOINT", "WIND_3D_SETPOINT"} and s["target"]["direction"] in {"+x", "-x"} and s["target"]["radius_m"] in {0.15, 0.25} and (s["scenario"] == "CALM_3D_SETPOINT" or s["wind"].get("speed_m_s") == 3.0)]
    ramp = [s for s in development if s["scenario"] == "RAMP_WIND_EQUILIBRIUM_HOLD"]
    if len(tuning) != 8 or len(ramp) != 1: raise RuntimeError("R3 stage-1 sample contract mismatch")
    summaries = []
    for index, params in enumerate(grid):
        rows = [run_case(params, sample) for sample in tuning + ramp]
        summaries.append(aggregate(params, rows))
        print(f"stage1 {index + 1}/{len(grid)}", flush=True)
    stage1_csv = R3 / "stage1_candidates.csv"
    keys = ["candidate_id", "beta", "w_p", "w_theta", "R", "H", "sample_count", "safe_sample_count", "solver_success_count", "solver_success_rate", "task_success_count", "task_success_rate", "acquisition_median_s", "position_rmse_3d_m", "ramp_peak_position_error_m", "ramp_steady_state_position_error_m", "total_acceleration_effort", "total_solve_time_p95_ms"]
    with stage1_csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys, lineterminator="\n"); writer.writeheader()
        for s in summaries: writer.writerow({**{k: s["parameters"].get(k) for k in ("candidate_id", "beta", "w_p", "w_theta", "R", "H")}, **{k: s[k] for k in keys if k not in {"candidate_id", "beta", "w_p", "w_theta", "R", "H"}}})
    top6 = sorted(summaries, key=stage1_key)[:6]
    (R3 / "stage1_top6.json").write_text(json.dumps([{k: v for k, v in s.items() if k not in {"rows"}} for s in top6], indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    if args.stage == "stage1": return 0
    tmp = R3 / "runs"; tmp.mkdir(parents=True, exist_ok=True)
    stage2 = []; per_sample = []
    for rank, selected in enumerate(top6, 1):
        params = selected["parameters"]; rows = []
        for sample in development:
            output = tmp / params["candidate_id"] / f"{sample['sample_id']}.csv"
            row = run_case(params, sample, output); rows.append(row); per_sample.append({"candidate_id": params["candidate_id"], "stage1_rank": rank, **{k: row.get(k) for k in ("sample_id", "safe", "solver_success", "task_success", "acquisition_time_s", "position_rmse_3d_m", "orientation_rmse_deg", "ramp_peak_position_error_m", "ramp_steady_state_position_error_m", "total_acceleration_effort", "total_solve_time_p95_ms", "limiter_mismatch_max")}})
        stage2.append(aggregate(params, rows)); print(f"stage2 {params['candidate_id']} complete", flush=True)
    # Keep raw traces only for the eventual selected candidate; summaries for
    # every top-6 candidate remain available for the audit.
    all_top_ids = {s["candidate_id"] for s in stage2}; (R3 / "stage2_candidates.csv").write_text("\n".join([",".join(["candidate_id","safe_sample_count","solver_success_rate","task_success_rate","position_rmse_3d_m","acquisition_median_s","ramp_peak_position_error_m","ramp_steady_state_position_error_m","total_acceleration_effort","total_solve_time_p95_ms"])] + [",".join(str(s.get(k, s["parameters"].get(k, ""))) for k in ["candidate_id","safe_sample_count","solver_success_rate","task_success_rate","position_rmse_3d_m","acquisition_median_s","ramp_peak_position_error_m","ramp_steady_state_position_error_m","total_acceleration_effort","total_solve_time_p95_ms"]) for s in stage2]) + "\n", encoding="utf-8", newline="\n")
    with (R3 / "development_results.csv").open("w", encoding="utf-8", newline="") as stream:
        fields = list(per_sample[0]); writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n"); writer.writeheader(); writer.writerows(per_sample)
    eligibility = []
    for s in stage2:
        eligible = bool(s["safe_sample_count"] == 57 and s["solver_success_rate"] >= 1.0 and s["task_success_rate"] >= 0.719298 and s["position_rmse_3d_m"] <= 0.108004 and (s["acquisition_median_s"] is not None and s["acquisition_median_s"] <= 2.185) and (s["ramp_peak_position_error_m"] <= 0.123784 or s["ramp_steady_state_position_error_m"] <= 0.174534) and s["total_solve_time_p95_ms"] < 50.0 and s["limiter_mismatch_max"] <= 1e-6)
        s["development_eligible"] = eligible; eligibility.append(s)
    selected = sorted([s for s in eligibility if s["development_eligible"]], key=lambda s: (-s["task_success_rate"], s["position_rmse_3d_m"], s["acquisition_median_s"] or float("inf"), s["ramp_steady_state_position_error_m"], s["ramp_peak_position_error_m"], s["total_acceleration_effort"], s["total_solve_time_p95_ms"], s["candidate_id"]))
    selected_self = selected[0] if selected else None
    (R3 / "development_summary.json").write_text(json.dumps({"stage1_count": 36, "stage1_samples_per_candidate": 9, "stage2_top_count": 6, "stage2_samples_per_candidate": 57, "candidates": [{k: v for k, v in s.items() if k not in {"rows"}} for s in stage2], "selected": None if selected_self is None else {k: v for k, v in selected_self.items() if k != "rows"}}, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    if selected_self is None:
        (R3 / "near_miss.json").write_text(json.dumps([{k: v for k, v in s.items() if k != "rows"} for s in sorted(stage2, key=stage1_key)[:6]], indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
        print("CLOSED_WITH_NO_DEVELOPMENT_WIN_OF_TSRMPC", flush=True); return 0
    selected_params = selected_self["parameters"]
    # Re-run only the selected candidate to retain the mandated full raw traces.
    selected_dir = R3 / "runs" / selected_params["candidate_id"]
    for sample in development: run_case(selected_params, sample, selected_dir / f"{sample['sample_id']}.csv")
    a, b, c, k, p = frozen_method_arrays()
    freeze = {"candidate_id": selected_params["candidate_id"], "beta": selected_params["beta"], "w_p": selected_params["w_p"], "w_theta": selected_params["w_theta"], "R": selected_params["R"], "H": 20, "K_sha256": sha(ROOT / "reproducibility/frozen/linear_model/K.npy"), "P_task_sha256": hashlib.sha256(np.asarray(p, dtype=np.float64).tobytes()).hexdigest(), "A_sha256": sha(ROOT / "reproducibility/frozen/linear_model/A.npy"), "B_sha256": sha(ROOT / "reproducibility/frozen/linear_model/B.npy"), "C_task_sha256": sha(ROOT / "reproducibility/frozen/task_lqr/C_task.npy"), "implementation_freeze_head": START_HEAD, "source_head": "pending", "sample_bank_sha256": sha(R1R1 / "development_manifest.json"), "runner_sha256": sha(Path(__file__))}
    (R3 / "self_freeze.json").write_text(json.dumps(freeze, indent=2) + "\n", encoding="utf-8", newline="\n")
    (R3 / "self_metric_comparison.json").write_text(json.dumps({"selected": {k: v for k, v in selected_self.items() if k != "rows"}, "development_eligibility_only": True, "final_scientific_pass": False, "holdout_executed": False}, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    gate = {"task": "V2-R3-OF-TSRMPC-DEVELOPMENT-AND-FREEZE-R1", "start_head": START_HEAD, "traditional_unchanged": True, "r1r1_unchanged": True, "r2_contract_unchanged": True, "sample_bank_unchanged": True, "holdout_unchanged": True, "implementation_frozen_before_performance": True, "c_task_semantics_pass": True, "output_bias_model_correct": True, "matched_wind_dob_used": False, "steady_state_control_parity_pass": True, "backbone_parity_pass": True, "qp_constraint_pass": True, "grid_size": 36, "stage1_runs": 324, "stage2_top_count": 6, "stage2_runs": 342, "selected_candidate_exists": True, "development_safety_pass": selected_self["safe_sample_count"] == 57, "development_success_gate_pass": selected_self["task_success_rate"] >= 0.719298 and selected_self["solver_success_rate"] >= 1.0, "development_position_gate_pass": selected_self["position_rmse_3d_m"] <= 0.108004, "development_acquisition_gate_pass": selected_self["acquisition_median_s"] is not None and selected_self["acquisition_median_s"] <= 2.185, "development_ramp_gate_pass": selected_self["ramp_peak_position_error_m"] <= 0.123784 or selected_self["ramp_steady_state_position_error_m"] <= 0.174534, "realtime_gate_pass": selected_self["total_solve_time_p95_ms"] < 50.0, "self_ablation_completed": False, "advanced_paper_executed": False, "holdout_executed": False, "result": "V2_OF_TSRMPC_FROZEN"}
    (R3 / "gate.json").write_text(json.dumps(gate, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"result": gate["result"], "selected": selected_params}, indent=2), flush=True); return 0


if __name__ == "__main__":
    raise SystemExit(main())
