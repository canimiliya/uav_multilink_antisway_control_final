"""Run the frozen endpoint-equilibrium reachability certificate on Development."""

from __future__ import annotations

import csv
import json
import os
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import mujoco
import numpy as np
from scipy.optimize import least_squares

from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind_world
from uav_sway.models.model_config import load_model_config
from uav_sway.native_stack.case_semantics.resolver import NativeCaseResolver


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/native_stack/governance"
MODEL_PATH = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
MODEL_CONFIG_PATH = ROOT / "configs/model_5link.yaml"
AERO_PATH = ROOT / "configs/aerodynamics.yaml"
NOMINAL = {"calm", "moderate", "stochastic"}
CHALLENGE = {"strong_sustained", "strong_transient", "ramp"}


def quaternion_roll_pitch(roll: float, pitch: float) -> np.ndarray:
    cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
    cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
    return np.array([cr * cp, sr * cp, cr * sp, -sr * sp], dtype=float)


def certify(identity: dict[str, Any]) -> dict[str, Any]:
    resolver = NativeCaseResolver()
    case = resolver.resolve(identity)
    if case.split != "development" or not case.execution["execution_allowed"]:
        raise PermissionError("equilibrium certificate is Development-only")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    model_config = load_model_config(MODEL_CONFIG_PATH)
    aerodynamic = load_aerodynamic_config(AERO_PATH)
    signals = resolver.canonical_signals(case)
    final_wind = np.asarray(signals["wind_world"][-1], dtype=float)
    target = np.asarray(case.target["final_cutter_target_world_m"], dtype=float)
    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    quad_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    actuator_ids = [
        int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name))
        for name in ("thrust_motor", "mx_motor", "my_motor", "mz_motor")
    ]
    joint_ids = [
        int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{index}"))
        for index in range(1, 6)
    ]
    joint_qpos = [int(model.jnt_qposadr[index]) for index in joint_ids]

    last_tip = np.zeros(3)
    last_external = np.zeros(3)
    last_base = np.zeros(3)

    def evaluate(values: np.ndarray) -> np.ndarray:
        nonlocal last_tip, last_external, last_base
        roll, pitch = values[:2]
        joints = values[2:7]
        controls = values[7:11]
        data.qpos[:] = 0.0
        data.qpos[:3] = [0.0, 0.0, 3.2]
        data.qpos[3:7] = quaternion_roll_pitch(float(roll), float(pitch))
        data.qpos[joint_qpos] = joints
        data.qvel[:] = 0.0
        data.qacc[:] = 0.0
        data.ctrl[:] = 0.0
        data.xfrc_applied[:] = 0.0
        data.eq_active[:] = 0
        mujoco.mj_forward(model, data)
        data.qpos[:3] += target - np.asarray(data.site_xpos[tip_id])
        mujoco.mj_forward(model, data)
        wind = clear_and_apply_wind_world(model, data, model_config, aerodynamic, final_wind)
        data.ctrl[actuator_ids] = controls
        mujoco.mj_forward(model, data)
        last_tip = np.asarray(data.site_xpos[tip_id]).copy()
        last_external = np.asarray(wind["total_world"], dtype=float).copy()
        last_base = np.asarray(data.xpos[quad_id]).copy()
        return np.asarray(data.qacc, dtype=float).copy()

    joint_bound = np.deg2rad(99.0)
    attitude_bound = np.deg2rad(24.0)
    lower = np.r_[[-attitude_bound, -attitude_bound], np.full(5, -joint_bound), [0.0, -25.0, -25.0, -12.0]]
    upper = np.r_[[attitude_bound, attitude_bound], np.full(5, joint_bound), [285.74568, 25.0, 25.0, 12.0]]
    initial = np.r_[np.zeros(7), 13.24 * 9.81, np.zeros(3)]
    result = least_squares(
        evaluate, initial, bounds=(lower, upper), method="trf", x_scale="jac",
        ftol=1e-11, xtol=1e-11, gtol=1e-11, max_nfev=600,
    )
    residual = evaluate(result.x)
    endpoint_error = float(np.linalg.norm(last_tip - target))
    max_residual = float(np.max(np.abs(residual)))
    roll, pitch = result.x[:2]
    controls = result.x[7:11]
    initial_tip = np.asarray(case.target["initial_cutter_target_world_m"], dtype=float)
    target_distance = float(np.linalg.norm(target - initial_tip))
    conservative_acceleration = 1.0
    rest_to_rest_s = 2.0 * np.sqrt(target_distance / conservative_acceleration)
    longest_mode_period_s = 2.0 * np.pi * np.sqrt(2.5 / 9.81)
    certified_transition_s = rest_to_rest_s + 2.0 * longest_mode_period_s
    available_s = case.duration_s - case.issue_offset_s
    endpoint_safe = bool(
        target[2] > 0.05
        and last_base[2] > 0.05
        and max(abs(float(roll)), abs(float(pitch))) < np.deg2rad(25.0)
        and np.max(np.abs(result.x[2:7])) < np.deg2rad(100.0)
        and np.linalg.norm(last_base[:2]) < 8.0
        and np.linalg.norm(target[:2] - initial_tip[:2]) < 10.0
    )
    equilibrium_pass = bool(result.success and max_residual <= 1e-3 and endpoint_error <= 1e-6)
    time_pass = bool(certified_transition_s <= available_s)
    success = equilibrium_pass and endpoint_safe and time_pass
    if not equilibrium_pass:
        mechanism = "NO_CERTIFIED_STATIC_EQUILIBRIUM"
    elif not endpoint_safe:
        mechanism = "ENDPOINT_SAFETY_GEOMETRY"
    elif not time_pass:
        mechanism = "CONSERVATIVE_TRANSITION_TIME"
    else:
        mechanism = "CERTIFIED_REACHABLE"
    return {
        "sample_id": case.sample_id,
        "task_family": case.identity["task_family"],
        "wind_kind": case.identity["wind_kind"],
        "wind_direction": case.identity["wind_direction"],
        "safe": endpoint_safe,
        "endpoint_success": success,
        "equilibrium_pass": equilibrium_pass,
        "time_pass": time_pass,
        "failure_mechanism": mechanism,
        "solver_success": bool(result.success),
        "solver_nfev": int(result.nfev),
        "max_generalized_acceleration_residual": max_residual,
        "endpoint_position_error_m": endpoint_error,
        "available_time_s": available_s,
        "certified_transition_time_s": certified_transition_s,
        "rest_to_rest_translation_time_s": rest_to_rest_s,
        "two_longest_mode_periods_s": 2.0 * longest_mode_period_s,
        "required_thrust_N": float(controls[0]),
        "required_torque_Nm": controls[1:].tolist(),
        "roll_pitch_deg": np.rad2deg([roll, pitch]).tolist(),
        "joint_angles_deg": np.rad2deg(result.x[2:7]).tolist(),
        "uav_position_world_m": last_base.tolist(),
        "target_world_m": target.tolist(),
        "final_wind_world_m_s": final_wind.tolist(),
        "final_external_force_world_N": last_external.tolist(),
        "semantic_fingerprint": case.case_semantic_fingerprint,
        "execution_authority": "NONCAUSAL_ENDPOINT_EQUILIBRIUM_CERTIFICATE_NO_SELECTION_AUTHORITY",
    }


def aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    def block(subset):
        return {
            "cases": len(subset),
            "safety_rate": float(np.mean([row["safe"] for row in subset])),
            "endpoint_success_rate": float(np.mean([row["endpoint_success"] for row in subset])),
            "equilibrium_pass_rate": float(np.mean([row["equilibrium_pass"] for row in subset])),
            "time_pass_rate": float(np.mean([row["time_pass"] for row in subset])),
        }
    nominal = [row for row in rows if row["wind_kind"] in NOMINAL]
    challenge = [row for row in rows if row["wind_kind"] in CHALLENGE]
    by_wind = {wind: block([row for row in rows if row["wind_kind"] == wind]) for wind in sorted(NOMINAL | CHALLENGE)}
    by_task = {task: block([row for row in rows if row["task_family"] == task]) for task in ("setpoint", "smooth_trajectory")}
    endpoint_rate = float(np.mean([row["endpoint_success"] for row in rows]))
    return {
        "version": "p2-r0g-equilibrium-reachability-certificate-v1",
        "executed": True,
        "development_cases": len(rows),
        "controller_baseline": False,
        "selection_authority": "NONE",
        "all": block(rows),
        "nominal": block(nominal),
        "challenge": block(challenge),
        "by_wind": by_wind,
        "by_task_family": by_task,
        "required_wrench": {
            "max_thrust_N": max(row["required_thrust_N"] for row in rows),
            "max_abs_torque_Nm": np.max(np.abs(np.asarray([row["required_torque_Nm"] for row in rows])), axis=0).tolist(),
            "limits": {"thrust_N": 285.74568, "torque_Nm": [25.0, 25.0, 12.0]},
        },
        "max_generalized_acceleration_residual": max(row["max_generalized_acceleration_residual"] for row in rows),
        "max_certified_transition_time_s": max(row["certified_transition_time_s"] for row in rows),
        "min_available_time_s": min(row["available_time_s"] for row in rows),
        "failure_mechanisms": dict(sorted({key: sum(row["failure_mechanism"] == key for row in rows) for key in {row["failure_mechanism"] for row in rows}}.items())),
        "mission_feasible": endpoint_rate >= 0.95,
        "MISSION_ENVELOPE_STRUCTURAL_PROBLEM": endpoint_rate < 0.80,
        "interpretation": "This certifies endpoint equilibrium and conservative transfer-time authority, not trajectory tracking quality or controller competence.",
        "holdout_executed": False,
    }


def main() -> None:
    protocol = json.loads((OUT / "oracle_equilibrium_protocol.json").read_text(encoding="utf-8"))
    if not protocol["frozen_before_execution"] or protocol["parameter_search"] or protocol["holdout_allowed"]:
        raise AssertionError("equilibrium protocol is not frozen")
    manifest = json.loads((ROOT / "reproducibility/native_stack/r0/native_development_manifest.json").read_text(encoding="utf-8"))
    identities = manifest["cases"]
    if len(identities) != 200 or any(case["split"] != "development" for case in identities):
        raise AssertionError("unexpected Development identities")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows = list(pool.map(certify, identities, chunksize=1))
    rows.sort(key=lambda row: row["sample_id"])
    with (OUT / "oracle_equilibrium_results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        for row in rows:
            serialized = dict(row)
            for key in ("required_torque_Nm", "roll_pitch_deg", "joint_angles_deg", "uav_position_world_m", "target_world_m", "final_wind_world_m_s", "final_external_force_world_N"):
                serialized[key] = json.dumps(serialized[key], separators=(",", ":"))
            writer.writerow(serialized)
    (OUT / "oracle_diagnostic.json").write_text(json.dumps(aggregate(rows), indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
