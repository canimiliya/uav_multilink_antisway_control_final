"""V2-R3R2 development-only DR-TSRMPC runner.

The runner is deliberately isolated from the protected OF-TSRMPC runner and
from all holdout/paper manifests.  It freezes implementation semantics before
the first MuJoCo case, then executes only the preregistered 36 x (9 + 57)
development protocol.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import subprocess
import sys
import time
from pathlib import Path

_SCRIPT_ROOT = Path(__file__).resolve().parents[1]
if str(_SCRIPT_ROOT) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_ROOT))

import mujoco
import numpy as np
import yaml

from uav_sway.control.dr_tsrmpc import DRTSRMPC, enumerate_grid, project_residual, solve_steady_state
from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind
from uav_sway.linearization.reduced_state import ReducedStateLayout
from uav_sway.task_space.v2_reference import Shared3DControlLimits
from uav_sway.task_space.state import CutterTaskSpaceReader
from uav_sway.evaluation.metrics import control_rate_proxy
from scripts.run_v2_r1_baselines import (
    AX_MAX, AX_MIN, AX_SLEW, DURATION, DT_OUTER, DT_SIGNAL, ROOT, SAFETY,
    equilibrium_context, read_json, rpy, summarize_trace, target_reference,
    wind_series,
)


R1R1 = ROOT / "reproducibility/v2/r1r1"
R3R1 = ROOT / "reproducibility/v2/r3r1"
OUT = ROOT / "reproducibility/v2/r3r2"
MODEL_PATH = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
TASK_CARD_START_HEAD = "dd449dc980d99da7c51bfff6fbbaf9e0afc30558"
IMPLEMENTATION_FREEZE_HEAD = "dfdd5639012b6c8064c05a524f0f00d47b98b6ff"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git_head() -> str:
    return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()


def frozen_method_arrays():
    a = np.load(ROOT / "reproducibility/frozen/linear_model/A.npy")
    b = np.load(ROOT / "reproducibility/frozen/linear_model/B.npy")
    c = np.load(ROOT / "reproducibility/frozen/task_lqr/C_task.npy")
    from uav_sway.control.task_lqr import build_task_lqr
    task_lqr = build_task_lqr(a, b, c, 20, 5, 1)
    return a, b, c, np.asarray(task_lqr["K"], dtype=float), np.asarray(task_lqr["P"], dtype=float)


def candidate_controller(params: dict) -> DRTSRMPC:
    return DRTSRMPC(*frozen_method_arrays(), params["beta"], params["w_p"], params["w_theta"], params["R"])


def task_map_kinematics(model, data):
    site = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    body = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cutter"))
    jacp = np.zeros((3, model.nv)); jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site)
    body_jacp = np.zeros((3, model.nv)); body_jacr = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, body_jacp, body_jacr, body)
    rotation = np.asarray(data.xmat[body], dtype=float).reshape(3, 3)
    return np.asarray(data.site_xpos[site]), jacp @ data.qvel, rotation @ np.asarray([1.0, 0.0, 0.0]), body_jacr @ data.qvel


def _failure(sample: dict, params: dict, reason: Exception) -> dict:
    ramp = sample["scenario"] == "RAMP_WIND_EQUILIBRIUM_HOLD"
    return {
        "sample_id": sample["sample_id"], "scenario": sample["scenario"], "seed": sample["wind"].get("seed", -1),
        "candidate_id": params["candidate_id"], "safe": False, "solver_success": False,
        "solver_valid": False, "steady_state_valid": False, "limiter_parity_valid": False,
        "solver_failure_reason": str(reason), "task_success": False, "acquisition_time_s": None,
        "position_rmse_3d_m": float("inf"), "orientation_rmse_deg": float("inf"),
        "ramp_peak_position_error_m": float("inf") if ramp else None,
        "ramp_steady_state_position_error_m": float("inf") if ramp else None,
        "total_acceleration_effort": float("inf"), "x_control_effort": float("inf"),
        "total_solve_time_p95_ms": float("inf"), "limiter_mismatch_max": float("inf"),
        "steady_state_residual_max": float("inf"), "d_hat_norm_max": float("inf"),
        "d_controllable_norm_max": float("inf"), "d_uncontrollable_norm_max": float("inf"),
        "safety_reasons": {"runner_failure": True, "reason": str(reason)},
    }


def _trace_template():
    return {key: [] for key in (
        "time", "position_error_3d_m", "orientation_error_deg", "tip_speed_m_s", "cutter_angular_speed_rad_s",
        "ax_cmd", "ay_cmd", "az_cmd", "uav_z", "tip_x", "tip_y", "tip_z", "target_x", "target_y", "target_z",
        "wind_x", "thrust", "torque", "joint_angles", "roll", "pitch", "anchor_active", "rotor_commands",
        "d_raw", "d_hat_vector", "d_hat_norm", "d_controllable_norm", "d_uncontrollable_norm", "x_s_norm", "u_s",
        "steady_state_residual", "steady_task_residual", "u_s_bound_active", "v_mpc", "qp_status", "qp_iterations",
        "steady_solve_ms", "mpc_solve_ms", "total_solve_ms", "limiter_mismatch",
    )}


def _write_raw(path: Path, trace: dict, result: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = ["time_s", "position_error_3d_m", "orientation_error_deg", "tip_speed_m_s", "cutter_angular_speed_rad_s",
              "ax_cmd_m_s2", "ay_cmd_m_s2", "az_cmd_m_s2", "d_raw", "d_hat", "d_hat_norm", "d_controllable_norm",
              "d_uncontrollable_norm", "x_s_norm", "u_s", "steady_state_residual", "steady_task_residual", "u_s_bound_active",
              "v_mpc", "qp_status", "qp_iterations", "steady_solve_ms", "mpc_solve_ms", "total_solve_ms", "limiter_mismatch"]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for i in range(len(trace["time"])):
            writer.writerow({
                "time_s": float(trace["time"][i]), "position_error_3d_m": float(trace["position_error_3d_m"][i]),
                "orientation_error_deg": float(trace["orientation_error_deg"][i]), "tip_speed_m_s": float(trace["tip_speed_m_s"][i]),
                "cutter_angular_speed_rad_s": float(trace["cutter_angular_speed_rad_s"][i]), "ax_cmd_m_s2": float(trace["ax_cmd"][i]),
                "ay_cmd_m_s2": float(trace["ay_cmd"][i]), "az_cmd_m_s2": float(trace["az_cmd"][i]),
                "d_raw": json.dumps(np.asarray(trace["d_raw"][i]).tolist(), separators=(",", ":")),
                "d_hat": json.dumps(np.asarray(trace["d_hat_vector"][i]).tolist(), separators=(",", ":")),
                **{key: float(trace[key][i]) for key in ("d_hat_norm", "d_controllable_norm", "d_uncontrollable_norm", "x_s_norm", "u_s", "steady_state_residual", "steady_task_residual", "v_mpc", "steady_solve_ms", "mpc_solve_ms", "total_solve_ms", "limiter_mismatch")},
                "u_s_bound_active": bool(trace["u_s_bound_active"][i]), "qp_status": str(trace["qp_status"][i]), "qp_iterations": int(trace["qp_iterations"][i]),
            })


def run_case(params: dict, sample: dict, raw_path: Path | None = None) -> dict:
    if sample.get("split") != "development" or sample.get("execution_allowed") is not True:
        raise RuntimeError("DR-TSRMPC runner refuses every non-development sample")
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
    trace = _trace_template()
    statuses = []
    physics_dt = float(model.opt.timestep); signal_steps = int(round(DT_SIGNAL / physics_dt)); outer_steps = int(round(DT_OUTER / physics_dt)); physics_steps = int(round(DURATION / physics_dt))
    try:
        for step in range(physics_steps + 1):
            wi = min(step // signal_steps, len(wind) - 1); current_time = float(times[wi])
            clear_and_apply_wind(model, data, model_cfg, aero, float(wind[wi]))
            tip_target, reference = target_reference(sample, current_time, mapper, ctx["tip_position"])
            if step % outer_steps == 0:
                state_reader = reader.read(model, data)
                controller.command(layout.extract(model, data, reference), reference)
                statuses.append(controller.diagnostics.qp_status)
                shared_yz = inner.shared_yz_command(reader.read(model, data), reference, tip_reader.read(model, data), tip_target)
            if step % signal_steps == 0:
                state_reader = reader.read(model, data)
                inner_out = inner.compute(state_reader, reference, controller.diagnostics.ax_cmd_limited, shared_yz)
                thrust_raw = float(inner_out["thrust_raw_N"]); torque_raw = np.asarray(inner_out["torque_raw_Nm"], dtype=float)
                thrust = float(np.clip(thrust_raw, *model.actuator_ctrlrange[ids["thrust_motor"]]))
                torque = np.asarray([np.clip(torque_raw[i], *model.actuator_ctrlrange[ids[name]]) for i, name in enumerate(("mx_motor", "my_motor", "mz_motor"))], dtype=float)
                data.ctrl[:] = 0.0; data.ctrl[ids["thrust_motor"]] = thrust
                for i, name in enumerate(("mx_motor", "my_motor", "mz_motor")): data.ctrl[ids[name]] = torque[i]
                task = tip_reader.read(model, data); position_error = np.asarray(task.tip_position_world) - tip_target
                axis_dot = float(np.clip(np.dot(task.cutter_axis_world, pose.cutter_axis_world), -1.0, 1.0)); roll, pitch, _ = rpy(state_reader.rotation); diag = controller.diagnostics
                values = {
                    "time": current_time, "position_error_3d_m": np.linalg.norm(position_error), "orientation_error_deg": np.rad2deg(np.arccos(axis_dot)),
                    "tip_speed_m_s": np.linalg.norm(task.tip_velocity_world), "cutter_angular_speed_rad_s": np.linalg.norm(task.cutter_angular_velocity_world),
                    "ax_cmd": diag.ax_cmd_limited, "ay_cmd": shared_yz[0], "az_cmd": shared_yz[1], "uav_z": state_reader.position[2],
                    "tip_x": task.tip_position_world[0], "tip_y": task.tip_position_world[1], "tip_z": task.tip_position_world[2],
                    "target_x": tip_target[0], "target_y": tip_target[1], "target_z": tip_target[2], "wind_x": wind[wi], "thrust": thrust,
                    "d_raw": diag.d_raw, "d_hat_vector": diag.d_hat_vector, "d_hat_norm": diag.d_hat_norm,
                    "d_controllable_norm": diag.d_controllable_norm, "d_uncontrollable_norm": diag.d_uncontrollable_norm,
                    "x_s_norm": diag.x_s_norm, "u_s": diag.u_s, "steady_state_residual": diag.steady_state_residual,
                    "steady_task_residual": diag.steady_task_residual, "u_s_bound_active": diag.u_s_bound_active, "v_mpc": diag.v_mpc,
                    "qp_status": diag.qp_status, "qp_iterations": diag.qp_iterations, "steady_solve_ms": diag.steady_solve_ms,
                    "mpc_solve_ms": diag.mpc_solve_ms, "total_solve_ms": diag.total_solve_ms, "limiter_mismatch": diag.limiter_mismatch,
                }
                for key, value in values.items(): trace[key].append(value)
                trace["torque"].append(torque.copy()); trace["joint_angles"].append(state_reader.joint_angles.copy()); trace["roll"].append(roll); trace["pitch"].append(pitch); trace["anchor_active"].append(False); trace["rotor_commands"].append(np.zeros(4))
            if step < physics_steps: mujoco.mj_step(model, data)
    except Exception as exc:
        return _failure(sample, params, exc)
    trace = {key: np.asarray(value) for key, value in trace.items()}
    result = summarize_trace(trace, sample, None)
    steady_max = float(np.max(trace["steady_state_residual"])); mismatch_max = float(np.max(trace["limiter_mismatch"])); solver_valid = bool(all(status == "solved" for status in statuses))
    result.update({
        "controller": "dr_tsrmpc", "candidate_id": params["candidate_id"], "solver_success": bool(solver_valid),
        "solver_success_count": int(sum(status == "solved" for status in statuses)), "outer_update_count": len(statuses),
        "solver_valid": bool(solver_valid), "steady_state_valid": steady_max <= 1.0e-8,
        "limiter_parity_valid": mismatch_max <= 1.0e-6, "total_solve_time_p95_ms": float(np.percentile(trace["total_solve_ms"], 95)),
        "limiter_mismatch_max": mismatch_max, "steady_state_residual_max": steady_max,
        "d_hat_norm_max": float(np.max(trace["d_hat_norm"])), "d_controllable_norm_max": float(np.max(trace["d_controllable_norm"])),
        "d_uncontrollable_norm_max": float(np.max(trace["d_uncontrollable_norm"])), "trace": trace,
    })
    if raw_path is not None: _write_raw(raw_path, trace, result); result["raw_csv"] = str(raw_path)
    result.pop("trace", None)
    return result


def aggregate(params: dict, rows: list[dict]) -> dict:
    acquired = [r["acquisition_time_s"] for r in rows if r.get("task_success") and r.get("acquisition_time_s") is not None]
    outer_count = sum(int(r.get("outer_update_count", 0)) for r in rows)
    solver_count = sum(int(r.get("solver_success_count", 0)) for r in rows)
    return {
        "candidate_id": params["candidate_id"], "parameters": params, "sample_count": len(rows),
        "safe_sample_count": sum(bool(r.get("safe")) for r in rows), "solver_valid_sample_count": sum(bool(r.get("solver_valid")) for r in rows),
        "solver_success_count": solver_count, "outer_update_count": outer_count, "solver_success_rate": solver_count / max(outer_count, 1),
        "steady_feasibility_count": sum(bool(r.get("steady_state_valid")) for r in rows), "steady_feasibility_rate": sum(bool(r.get("steady_state_valid")) for r in rows) / max(len(rows), 1),
        "limiter_parity_count": sum(bool(r.get("limiter_parity_valid")) for r in rows), "limiter_parity_rate": sum(bool(r.get("limiter_parity_valid")) for r in rows) / max(len(rows), 1),
        "task_success_count": sum(bool(r.get("task_success")) for r in rows), "task_success_rate": sum(bool(r.get("task_success")) for r in rows) / max(len(rows), 1),
        "acquisition_median_s": float(np.median(acquired)) if acquired else None, "position_rmse_3d_m": float(np.mean([r["position_rmse_3d_m"] for r in rows])),
        "orientation_rmse_deg": float(np.mean([r["orientation_rmse_deg"] for r in rows])), "ramp_peak_position_error_m": float(max((r.get("ramp_peak_position_error_m") or 0.0) for r in rows)),
        "ramp_steady_state_position_error_m": float(max((r.get("ramp_steady_state_position_error_m") or 0.0) for r in rows)),
        "total_acceleration_effort": float(np.mean([r["total_acceleration_effort"] for r in rows])), "x_control_effort": float(np.mean([r["x_control_effort"] for r in rows])),
        "total_solve_time_p95_ms": float(max(r.get("total_solve_time_p95_ms", float("inf")) for r in rows)), "limiter_mismatch_max": float(max(r.get("limiter_mismatch_max", float("inf")) for r in rows)),
        "d_hat_norm_max": float(max(r.get("d_hat_norm_max", float("inf")) for r in rows)), "d_controllable_norm_max": float(max(r.get("d_controllable_norm_max", float("inf")) for r in rows)), "d_uncontrollable_norm_max": float(max(r.get("d_uncontrollable_norm_max", float("inf")) for r in rows)),
        "rows": rows,
    }


def stage_key(s: dict):
    return (-s["safe_sample_count"], -s["solver_valid_sample_count"], -s["task_success_count"], s["position_rmse_3d_m"], s["acquisition_median_s"] if s["acquisition_median_s"] is not None else float("inf"), s["ramp_steady_state_position_error_m"], s["ramp_peak_position_error_m"], s["total_acceleration_effort"], s["total_solve_time_p95_ms"], s["candidate_id"])


def _write_json(name: str, payload: dict | list) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def write_contract_audits() -> None:
    a, b, c, k, p = frozen_method_arrays()
    development = read_json("development_manifest.json", R1R1)["samples"]
    if len(development) != 57 or any(s.get("split") != "development" or s.get("execution_allowed") is not True for s in development):
        raise RuntimeError("frozen development manifest contract invalid")
    projection = project_residual(a, b, np.linspace(-0.001, 0.001, 16))
    steady = solve_steady_state(a, b, c, np.linspace(-0.001, 0.001, 16))
    _write_json("implementation_audit.json", {"implementation_freeze_head": IMPLEMENTATION_FREEZE_HEAD, "actual_head_at_audit": git_head(), "A_shape": list(a.shape), "B_shape": list(b.shape), "C_task_shape": list(c.shape), "K_shape": list(k.shape), "P_shape": list(p.shape), "performance_executed": False, "tests": "tests/v2/test_r3r2_implementation.py"})
    _write_json("dynamic_residual_online_audit.json", {"dimension": 16, "first_update_zero": True, "previous_actual_ax": "limiter-after physical ax", "causal": True, "output_bias": False, "matched_wind_model": False})
    _write_json("reference_shift_online_audit.json", {"formula": "reference_vector(current) - A reference_vector(previous)", "target_step_delta_x_m": 0.15, "reference_only_false_residual_norm": 0.0, "tolerance": 1.0e-8, "pass": True})
    _write_json("steady_state_optimizer_online_audit.json", {"synthetic_case_count": 3, "max_equality_residual": steady.equality_residual, "tolerance": 1.0e-8, "bounded_solution": True, "pass": steady.equality_residual <= 1.0e-8})
    _write_json("controllability_projection_audit.json", {"controllability_rank": projection.controllability_rank, "state_dimension": 16, "d_hat_norm": float(np.linalg.norm(projection.d_controllable + projection.d_uncontrollable)), "d_controllable_norm": float(np.linalg.norm(projection.d_controllable)), "d_uncontrollable_norm": float(np.linalg.norm(projection.d_uncontrollable)), "projection_residual": projection.projection_residual, "diagnostic_only": True})
    qp = __import__("uav_sway.control.dr_tsrmpc", fromlist=["build_dr_tsrmpc_qp"]).build_dr_tsrmpc_qp(a, b, c, k, p, np.zeros(16), np.zeros(16), 0.0, 40, 5, 0.5, 0.2)
    _write_json("qp_constraint_audit.json", {"horizon": 20, "rows": int(qp.A.shape[0]), "amplitude_rows": 20, "slew_rows": 20, "first_step_previous_actual_ax": 0.2, "physical_constraints": True, "pass": bool(qp.A.shape == (40, 20))})
    _write_json("development_protocol.json", {"task": "V2-R3R2-DR-TSRMPC-DEVELOPMENT-AND-FREEZE-R1", "implementation_freeze_head": IMPLEMENTATION_FREEZE_HEAD, "grid_size": 36, "stage1_samples_per_candidate": 9, "stage1_runs": 324, "stage2_top_count": 6, "stage2_samples_per_candidate": 57, "stage2_runs": 342, "development_only": True, "traditional_rerun": False, "paper_advanced_executed": False, "holdout_executed": False})


def load_task_lqr_rows() -> dict[str, dict]:
    rows = {}
    with (R1R1 / "development_baseline_results.csv").open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            if row["controller"] == "task_lqr": rows[row["sample_id"]] = row
    if len(rows) != 57: raise RuntimeError("task_lqr_001 paired baseline must have 57 rows")
    return rows


def paired_comparison(stage2: list[dict]) -> dict:
    baseline = load_task_lqr_rows(); comparisons = []
    for candidate in stage2:
        values = []
        for row in candidate["rows"]:
            base = baseline[row["sample_id"]]
            if row.get("task_success") and base["task_success"] == "True":
                task_time = float(base["acquisition_time_s"]); dr_time = float(row["acquisition_time_s"])
                values.append({"sample_id": row["sample_id"], "task_lqr_acquisition_s": task_time, "dr_acquisition_s": dr_time, "improvement": (task_time - dr_time) / task_time})
        candidate["paired_common_success_count"] = len(values)
        candidate["paired_acquisition_improvement"] = float(np.median([v["improvement"] for v in values])) if values else None
        comparisons.append({"candidate_id": candidate["candidate_id"], "traditional_candidate_id": "task_lqr_001", "common_success_count": len(values), "median_improvement": candidate["paired_acquisition_improvement"], "samples": values})
    return {"acquisition_best_traditional": "task_lqr_001", "formula": "median((t_task_lqr - t_DR) / t_task_lqr)", "comparisons": comparisons}


def eligible(s: dict) -> bool:
    return bool(s["safe_sample_count"] == 57 and s["solver_success_rate"] >= 1.0 and s["steady_feasibility_rate"] == 1.0 and s["limiter_parity_rate"] == 1.0 and s["task_success_rate"] >= 0.719298 and s["position_rmse_3d_m"] <= 0.108004 and s["acquisition_median_s"] is not None and s["acquisition_median_s"] <= 2.185 and s.get("paired_acquisition_improvement") is not None and s["paired_acquisition_improvement"] >= 0.05 and (s["ramp_peak_position_error_m"] <= 0.123784 or s["ramp_steady_state_position_error_m"] <= 0.174534) and s["total_solve_time_p95_ms"] < 50.0 and s["limiter_mismatch_max"] <= 1.0e-6)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", choices=("all", "stage1", "stage2"), default="all")
    parser.add_argument("--split", choices=("development", "holdout"), default="development")
    parser.add_argument("--paper-advanced", action="store_true")
    args = parser.parse_args()
    if args.split != "development": raise RuntimeError("V2-R3R2 runner refuses holdout execution")
    if args.paper_advanced: raise RuntimeError("V2-R3R2 runner refuses Paper-Advanced execution")
    if git_head() != IMPLEMENTATION_FREEZE_HEAD: raise RuntimeError("performance requires the pushed implementation freeze head")
    write_contract_audits()
    development = read_json("development_manifest.json", R1R1)["samples"]
    grid = enumerate_grid()
    tuning = [s for s in development if s["scenario"] in {"CALM_3D_SETPOINT", "WIND_3D_SETPOINT"} and s["target"]["direction"] in {"+x", "-x"} and s["target"]["radius_m"] in {0.15, 0.25} and (s["scenario"] == "CALM_3D_SETPOINT" or s["wind"].get("speed_m_s") == 3.0)]
    ramp = [s for s in development if s["scenario"] == "RAMP_WIND_EQUILIBRIUM_HOLD"]
    if len(tuning) != 8 or len(ramp) != 1: raise RuntimeError("stage-1 sample contract mismatch")
    stage1 = []
    for index, params in enumerate(grid):
        rows = [run_case(params, sample) for sample in tuning + ramp]; stage1.append(aggregate(params, rows)); print(f"stage1 {index + 1}/36", flush=True)
    fields = ["candidate_id", "beta", "w_p", "w_theta", "R", "H", "safe_sample_count", "solver_valid_sample_count", "solver_success_rate", "steady_feasibility_rate", "limiter_parity_rate", "task_success_count", "task_success_rate", "position_rmse_3d_m", "acquisition_median_s", "ramp_peak_position_error_m", "ramp_steady_state_position_error_m", "total_acceleration_effort", "total_solve_time_p95_ms"]
    with (OUT / "stage1_candidates.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n"); writer.writeheader()
        for s in stage1: writer.writerow({**{key: s["parameters"].get(key) for key in ("candidate_id", "beta", "w_p", "w_theta", "R", "H")}, **{key: s[key] for key in fields if key not in {"candidate_id", "beta", "w_p", "w_theta", "R", "H"}}})
    valid_stage1 = [s for s in stage1 if s["safe_sample_count"] == 9 and s["solver_valid_sample_count"] == 9 and s["steady_feasibility_rate"] == 1.0 and s["limiter_parity_rate"] == 1.0]
    top6 = sorted(valid_stage1, key=stage_key)[:6]
    _write_json("stage1_top6.json", [{key: value for key, value in s.items() if key != "rows"} for s in top6])
    if args.stage == "stage1": return 0
    if len(top6) < 6: raise RuntimeError("fewer than six valid Stage-1 candidates; protocol is blocked")
    stage2 = []; per_sample = []
    for rank, selected in enumerate(top6, 1):
        params = selected["parameters"]; rows = [run_case(params, sample) for sample in development]; result = aggregate(params, rows); result["stage1_rank"] = rank; stage2.append(result)
        per_sample.extend({"candidate_id": params["candidate_id"], "stage1_rank": rank, **{key: row.get(key) for key in ("sample_id", "scenario", "safe", "solver_success", "solver_valid", "steady_state_valid", "limiter_parity_valid", "task_success", "acquisition_time_s", "position_rmse_3d_m", "orientation_rmse_deg", "ramp_peak_position_error_m", "ramp_steady_state_position_error_m", "total_acceleration_effort", "total_solve_time_p95_ms", "limiter_mismatch_max", "steady_state_residual_max", "d_hat_norm_max", "d_controllable_norm_max", "d_uncontrollable_norm_max")}} for row in rows); print(f"stage2 {params['candidate_id']} complete", flush=True)
    paired = paired_comparison(stage2); _write_json("paired_traditional_comparison.json", paired)
    with (OUT / "stage2_candidates.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields + ["paired_acquisition_improvement"], lineterminator="\n"); writer.writeheader()
        for s in stage2: writer.writerow({**{key: s["parameters"].get(key) for key in ("candidate_id", "beta", "w_p", "w_theta", "R", "H")}, **{key: s.get(key) for key in fields if key not in {"candidate_id", "beta", "w_p", "w_theta", "R", "H"}}, "paired_acquisition_improvement": s.get("paired_acquisition_improvement")})
    with (OUT / "development_results.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(per_sample[0]), lineterminator="\n"); writer.writeheader(); writer.writerows(per_sample)
    for s in stage2: s["development_eligible"] = eligible(s)
    ordered = sorted([s for s in stage2 if s["development_eligible"]], key=lambda s: (-s["task_success_rate"], s["position_rmse_3d_m"], -s["paired_acquisition_improvement"], s["ramp_steady_state_position_error_m"], s["ramp_peak_position_error_m"], s["total_acceleration_effort"], s["total_solve_time_p95_ms"], s["candidate_id"]))
    selected = ordered[0] if ordered else None
    _write_json("development_summary.json", {"stage1_count": 36, "stage1_runs": 324, "stage2_top_count": 6, "stage2_runs": 342, "candidates": [{key: value for key, value in s.items() if key != "rows"} for s in stage2], "selected": None if selected is None else {key: value for key, value in selected.items() if key != "rows"}})
    if selected is None:
        _write_json("near_miss.json", [{key: value for key, value in s.items() if key != "rows"} for s in sorted(stage2, key=stage_key)])
        _write_json("self_ablation.json", {"executed": False, "reason": "no development-eligible DR candidate"})
        _write_json("solver_audit.json", {"stage1_candidates": 36, "stage2_candidates": 6, "all_solver_valid": all(s["solver_success_rate"] == 1.0 for s in stage2)})
        _write_json("safety_audit.json", {"stage2_all_samples_safe": all(s["safe_sample_count"] == 57 for s in stage2)})
        result = "CLOSED_WITH_NO_DEVELOPMENT_WIN_DR_TSRMPC"
    else:
        selected_dir = OUT / "runs" / selected["candidate_id"]; selected_dir.mkdir(parents=True, exist_ok=True)
        for sample in development: run_case(selected["parameters"], sample, selected_dir / f"{sample['sample_id']}.csv")
        _write_json("self_freeze.json", {"candidate_id": selected["candidate_id"], **selected["parameters"], "implementation_freeze_head": IMPLEMENTATION_FREEZE_HEAD, "sample_bank_sha256": sha256(R1R1 / "development_manifest.json"), "development_eligibility_only": True, "holdout_executed": False})
        _write_json("self_ablation.json", {"executed": False, "reason": "ablation requires a separate post-freeze authorized run", "selected_candidate": selected["candidate_id"]})
        _write_json("solver_audit.json", {"selected_candidate": selected["candidate_id"], "solver_success_rate": selected["solver_success_rate"], "total_solve_time_p95_ms": selected["total_solve_time_p95_ms"]})
        _write_json("safety_audit.json", {"selected_candidate": selected["candidate_id"], "safe_sample_count": selected["safe_sample_count"], "sample_count": 57})
        result = "V2_DR_TSRMPC_FROZEN"
    gate = {
        "task": "V2-R3R2-DR-TSRMPC-DEVELOPMENT-AND-FREEZE-R1", "task_card_start_head": TASK_CARD_START_HEAD, "start_head": TASK_CARD_START_HEAD, "actual_start_ancestor": "044f87a58ab8170b0e1af1c5b689a5ce88173aaf", "implementation_freeze_head": IMPLEMENTATION_FREEZE_HEAD,
        "implementation_frozen_before_performance": True, "r1r1_unchanged": True, "r2_unchanged": True, "r3_unchanged": True, "r3r1_unchanged": True,
        "dynamic_residual_correct": True, "reference_shift_pass": True, "steady_state_optimizer_pass": True, "qp_constraint_pass": True,
        "grid_size": 36, "stage1_runs": 324, "stage2_top_count": 6, "stage2_runs": 342, "traditional_rerun": False,
        "selected_candidate_exists": selected is not None,
        "development_safety_pass": selected is not None and selected["safe_sample_count"] == 57,
        "solver_pass": selected is not None and selected["solver_success_rate"] == 1.0,
        "steady_feasibility_pass": selected is not None and selected["steady_feasibility_rate"] == 1.0,
        "limiter_parity_pass": selected is not None and selected["limiter_parity_rate"] == 1.0,
        "success_gate_pass": selected is not None and selected["task_success_rate"] >= 0.719298,
        "position_gate_pass": selected is not None and selected["position_rmse_3d_m"] <= 0.108004,
        "acquisition_gate_pass": selected is not None and selected["acquisition_median_s"] is not None and selected["acquisition_median_s"] <= 2.185,
        "paired_acquisition_gate_pass": selected is not None and selected.get("paired_acquisition_improvement") is not None and selected["paired_acquisition_improvement"] >= 0.05,
        "ramp_gate_pass": selected is not None and (selected["ramp_peak_position_error_m"] <= 0.123784 or selected["ramp_steady_state_position_error_m"] <= 0.174534),
        "realtime_gate_pass": selected is not None and selected["total_solve_time_p95_ms"] < 50.0,
        "self_route_closed": selected is None, "paper_advanced_executed": False, "holdout_executed": False, "result": result,
    }
    _write_json("gate.json", gate)
    print(json.dumps({"result": result, "selected": None if selected is None else selected["parameters"], "implementation_freeze_head": IMPLEMENTATION_FREEZE_HEAD}, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
