"""Run the selected S3 PID on all three frozen scenarios and emit raw gates."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import mujoco
import yaml

from uav_sway.control.runtime_model import create_runtime_model
from uav_sway.disturbances.wind_io import read_wind_csv
from uav_sway.evaluation.controlled_metrics import compute_controlled_metrics, load_controlled_csv
from uav_sway.evaluation.controlled_runner import ensure_calm_wind, run_controlled_scenario
from uav_sway.evaluation.controlled_schema import controlled_schema_description
from uav_sway.models.model_config import load_model_config

ROOT = Path(__file__).resolve().parents[1]


def _raw_gate(run_paths: dict[str, Path]) -> dict:
    thresholds = {
        "approach_stop": (0.25, 0.45, 0.15, 3.0),
        "crosswind_hover": (0.30, 0.35, 0.15, 0.0),
        "gust_micro_adjust": (0.20, 0.30, 0.15, 0.30),
    }
    results = {}
    residual = False
    all_pass = True
    for scenario, path in run_paths.items():
        columns, v = load_controlled_csv(path)
        max_joint = max(float(np.max(np.abs(v[c]))) for c in columns if c.startswith("joint_") and c.endswith("_angle"))
        residual = residual or float(np.max(np.abs(v["tip_displacement"]))) > 0.02 or max_joint > 0.005
        final_target, max_rmse, max_z_rmse, _ = thresholds[scenario]
        final_error = abs(float(v["uav_x"][-1] - v["x_ref"][-1]))
        checks = {
            "finite": bool(all(np.isfinite(a).all() for a in v.values() if a.dtype != object and a.dtype != bool)),
            "anchor_inactive": not bool(np.any(v["anchor_active"])), "minimum_uav_height": float(np.min(v["uav_z"])) > 0.05, "minimum_tip_height": float(np.min(v["tip_z"])) > 0.05,
            "joint_range": max_joint < np.deg2rad(100.0), "roll_limit": float(np.max(np.abs(v["roll_rad"]))) < np.deg2rad(25.0), "pitch_limit": float(np.max(np.abs(v["pitch_rad"]))) < np.deg2rad(25.0),
            "ax_limit": float(np.max(np.abs(v["ax_cmd_limited"]))) <= 2.0 + 1e-12, "ax_slew_limit": float(np.max(np.abs(np.diff(v["ax_cmd_limited"])))) <= 0.25 + 1e-12,
            "thrust_limit": bool(np.all(v["thrust_cmd_limited_N"] >= -1e-12) and np.all(v["thrust_cmd_limited_N"] <= 285.74568 + 1e-12)),
            "torque_limit": bool(np.all(np.abs(v["mx_cmd_limited_Nm"]) <= 25.0 + 1e-12) and np.all(np.abs(v["my_cmd_limited_Nm"]) <= 25.0 + 1e-12) and np.all(np.abs(v["mz_cmd_limited_Nm"]) <= 12.0 + 1e-12)),
            "final_x_error": final_error <= final_target, "x_rmse": float(np.sqrt(np.trapezoid((v["uav_x"] - v["x_ref"])**2, v["time"]) / (v["time"][-1] - v["time"][0]))) <= max_rmse,
            "z_rmse": float(np.sqrt(np.trapezoid((v["uav_z"] - v["z_ref"])**2, v["time"]) / (v["time"][-1] - v["time"][0]))) <= max_z_rmse,
        }
        checks = {key: bool(value) for key, value in checks.items()}
        results[scenario] = {"checks": checks, "pass": bool(all(checks.values()))}
        all_pass = all_pass and results[scenario]["pass"]
    return {"pass": bool(all_pass and residual), "residual_sway_confirmed": bool(residual), "scenarios": results, "source": "independent_raw_csv_recomputation"}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-config", required=True)
    parser.add_argument("--controller-config", required=True)
    parser.add_argument("--scenarios", nargs="+", required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    if not args.headless:
        raise SystemExit("S3 runner requires --headless")
    output = Path(args.output_dir); output.mkdir(parents=True, exist_ok=True)
    frozen_runtime = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
    calm = output / "inputs/calm.csv"
    ensure_calm_wind(calm, read_wind_csv(ROOT / "reproducibility/frozen/ls_pmpc/constant_crosswind.csv")["time"])
    controller = yaml.safe_load(Path(args.controller_config).read_text(encoding="utf-8"))
    model_config = load_model_config(args.model_config)
    run_paths = {}
    wind_by_scenario = {"approach_stop": calm, "crosswind_hover": ROOT / "reproducibility/frozen/ls_pmpc/constant_crosswind.csv", "gust_micro_adjust": ROOT / "reproducibility/frozen/ls_pmpc/one_cosine_gust.csv"}
    for scenario in args.scenarios:
        path = output / "runs" / scenario / "run.csv"
        run_paths[scenario] = path
        metrics = run_controlled_scenario(args.model_config, controller, scenario, wind_by_scenario[scenario], ROOT / "reproducibility/frozen/ls_pmpc" / f"{scenario}.csv", path, ROOT, True)
        (path.parent / "metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8", newline="\n")
    gate = _raw_gate(run_paths)
    (output / "raw_gate.json").write_text(json.dumps(gate, indent=2) + "\n", encoding="utf-8", newline="\n")
    summary = []
    for scenario, path in run_paths.items():
        metrics = compute_controlled_metrics(path, {"approach_stop": 6.0, "crosswind_hover": 4.0, "gust_micro_adjust": 5.0}[scenario])
        summary.append({"scenario": scenario, **{k: metrics[k] for k in ("x_position_rmse_m", "z_position_rmse_m", "final_x_error_m", "tip_max_abs_m", "tip_rms_m", "minimum_tip_height_m", "maximum_abs_roll_rad", "maximum_abs_pitch_rad", "control_rate_proxy", "saturation_rate")}})
    with (output / "pid_summary.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summary[0]), lineterminator="\n"); writer.writeheader(); writer.writerows(summary)
    (output / "pid_summary.json").write_text(json.dumps({"controller": "pid", "scenarios": summary, "raw_gate": gate}, indent=2) + "\n", encoding="utf-8", newline="\n")
    runtime_model = mujoco.MjModel.from_xml_path(str(output / "runtime/model_5link_controlled.xml"))
    quad_id = int(mujoco.mj_name2id(runtime_model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    inertia = np.asarray(runtime_model.body_inertia[quad_id], dtype=float)
    k_r = (inertia * 4.0**2).tolist()
    k_omega = (2.0 * 0.9 * inertia * 4.0).tolist()
    (output / "controller_contract.json").write_text(json.dumps({"controller": "pid", "protocol_mode": "free_flight_controlled", "anchor_active": False, "rotor_motors_zero": True, "pid_reads": ["uav_x", "uav_vx", "x_ref", "vx_ref", "ax_ref"], "pid_does_not_read": ["joint_angles", "joint_velocities", "tip_displacement"]}, indent=2) + "\n", encoding="utf-8", newline="\n")
    (output / "inner_loop_parameters.json").write_text(json.dumps({"total_system_mass_kg": float(np.sum(runtime_model.body_mass)), "attitude_natural_frequency_rad_s": 4.0, "attitude_damping_ratio": 0.9, "K_R": k_r, "K_Omega": k_omega, "position_y": {"kp": 1.5, "kd": 2.0}, "position_z": {"kp": 4.0, "kd": 3.5}, "torque_limits_Nm": [-25.0, 25.0, -25.0, 25.0, -12.0, 12.0], "source": "shared_inner_loop_simulation_assumption"}, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps(gate, indent=2))
    return 0 if gate["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
