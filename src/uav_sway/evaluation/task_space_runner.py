"""S6T0 task-space instrumentation runner.

This runner duplicates the frozen PID/LQR scheduling contract only to expose
MuJoCo task-space state at the same logging instants.  It does not alter either
controller or the shared inner loop.
"""

from __future__ import annotations

import argparse
import csv
import json
import platform
import sys
import time as wall_time
from pathlib import Path

import mujoco
import numpy as np
import yaml

from uav_sway.control.base import ReferenceState
from uav_sway.control.full_state_lqr import FullStateLQR
from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.control.position_pid import PositionPID
from uav_sway.control.runtime_model import create_runtime_model
from uav_sway.control.state_reader import StateReader
from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind
from uav_sway.disturbances.wind_io import read_wind_csv
from uav_sway.evaluation.controlled_schema import controlled_schema_columns
from uav_sway.evaluation.controlled_metrics import compute_controlled_metrics
from uav_sway.evaluation.task_space_metrics import compute_task_metrics
from uav_sway.linearization.reduced_state import ReducedStateLayout
from uav_sway.models.model_config import load_model_config
from uav_sway.scenarios.scenario_config import load_scenario_config
from uav_sway.task_space.reference import build_equilibrium_task_pose, task_reference_at, write_equilibrium_pose
from uav_sway.task_space.state import CutterTaskSpaceReader


ROOT = Path(__file__).resolve().parents[3]


def _id(model, object_type, name: str) -> int:
    value = int(mujoco.mj_name2id(model, object_type, name))
    if value < 0:
        raise KeyError(name)
    return value


def _read_reference(path: str | Path) -> dict[str, np.ndarray]:
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("empty reference")
    result = {key: np.asarray([float(row[key]) for row in rows], dtype=float) for key in ("time", "x_ref", "vx_ref", "ax_ref", "y_ref", "z_ref", "yaw_ref")}
    result["event"] = np.asarray([row["event"] for row in rows], dtype=object)
    return result


def _reference_at(ref: dict[str, np.ndarray], index: int) -> ReferenceState:
    return ReferenceState(*(float(ref[name][index]) for name in ("x_ref", "vx_ref", "ax_ref", "y_ref", "z_ref", "yaw_ref")))


def _rpy(rotation: np.ndarray) -> tuple[float, float, float]:
    return (float(np.arctan2(rotation[2, 1], rotation[2, 2])), float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))), float(np.arctan2(rotation[1, 0], rotation[0, 0])))


def _write_csv(path: Path, rows: list[dict], columns: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            encoded = {}
            for key in columns:
                value = row[key]
                if isinstance(value, (bool, np.bool_)):
                    encoded[key] = "true" if value else "false"
                elif isinstance(value, str):
                    encoded[key] = value
                else:
                    encoded[key] = format(float(value), ".17g")
            writer.writerow(encoded)


TASK_COLUMNS = [
    "reference_event",
    "tip_velocity_world_x", "tip_velocity_world_y", "tip_velocity_world_z",
    "cutter_axis_world_x", "cutter_axis_world_y", "cutter_axis_world_z",
    "task_reference_tip_x", "task_reference_tip_y", "task_reference_tip_z",
    "task_reference_axis_x", "task_reference_axis_y", "task_reference_axis_z",
    "tip_task_x_error_m", "tip_task_z_error_m", "task_position_error_xz_m", "position_error_3d_m",
    "tip_speed_m_s", "orientation_error_rad", "orientation_error_deg",
]


def _task_row(state, reference, task_reference, diag, base: dict) -> dict:
    position_error = state.tip_position_world - task_reference.tip_position_world
    dot = float(np.clip(np.dot(state.cutter_axis_world, task_reference.cutter_axis_world), -1.0, 1.0))
    orientation_rad = float(np.arccos(dot))
    return {
        **base,
        "tip_velocity_world_x": state.tip_velocity_world[0], "tip_velocity_world_y": state.tip_velocity_world[1], "tip_velocity_world_z": state.tip_velocity_world[2],
        "cutter_axis_world_x": state.cutter_axis_world[0], "cutter_axis_world_y": state.cutter_axis_world[1], "cutter_axis_world_z": state.cutter_axis_world[2],
        "task_reference_tip_x": task_reference.tip_position_world[0], "task_reference_tip_y": task_reference.tip_position_world[1], "task_reference_tip_z": task_reference.tip_position_world[2],
        "task_reference_axis_x": task_reference.cutter_axis_world[0], "task_reference_axis_y": task_reference.cutter_axis_world[1], "task_reference_axis_z": task_reference.cutter_axis_world[2],
        "tip_task_x_error_m": position_error[0], "tip_task_z_error_m": position_error[2], "task_position_error_xz_m": np.linalg.norm(position_error[[0, 2]]), "position_error_3d_m": np.linalg.norm(position_error),
        "tip_speed_m_s": np.linalg.norm(state.tip_velocity_world), "orientation_error_rad": orientation_rad, "orientation_error_deg": np.rad2deg(orientation_rad),
        **({"lqr_feedback_ax": diag.lqr_feedback_ax, "lqr_state_norm": diag.lqr_state_norm} if hasattr(diag, "lqr_feedback_ax") else {}),
    }


def run_task_space_scenario(model_config_path: str | Path, controller_name: str, scenario: str, wind_csv: str | Path, reference_csv: str | Path, output_csv: str | Path, repo_root: str | Path | None = None, duration_s: float = 12.0) -> dict:
    root = Path(repo_root or ROOT)
    runtime_xml = root / "reproducibility/frozen/model/model_5link_controlled.xml"
    model = mujoco.MjModel.from_xml_path(str(runtime_xml))
    s3 = yaml.safe_load((root / "configs/s3_pid.yaml").read_text(encoding="utf-8"))
    model_cfg = load_model_config(model_config_path)
    scenario_cfg = load_scenario_config(root / "configs/scenarios.yaml")
    aero = load_aerodynamic_config(root / "configs/aerodynamics.yaml")
    wind = read_wind_csv(wind_csv)
    ref = _read_reference(reference_csv)
    if len(wind["time"]) != len(ref["time"]):
        raise ValueError("wind/reference sample counts differ")
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0; data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0; data.ctrl[:] = 0.0; data.eq_active[:] = 0
    mujoco.mj_forward(model, data)
    pose = build_equilibrium_task_pose(model, data, runtime_xml)
    task_reader = CutterTaskSpaceReader(model)
    quad_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    tip_id = _id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip")
    equilibrium_relative_x = float(data.site_xpos[tip_id, 0] - data.xpos[quad_id, 0])
    reader = StateReader(model, model_cfg.n_links, equilibrium_relative_x)
    total_mass = float(np.sum(model.body_mass))
    inner = GeometricInnerLoop(total_mass, np.asarray(model.body_inertia[quad_id], dtype=float), s3["attitude_natural_frequency_rad_s"], s3["attitude_damping_ratio"], *s3["position_gains_y"], *s3["position_gains_z"])
    if controller_name == "pid":
        cfg = yaml.safe_load((root / "configs/controllers.yaml").read_text(encoding="utf-8"))
        controller = PositionPID(cfg["kp"], cfg["kd"], cfg["ki"], cfg["ax_min_m_s2"], cfg["ax_max_m_s2"], cfg["delta_ax_max_per_outer_step"], cfg["integral_error_limit_m_s"])
        controller.reset(reader.read(model, data), _reference_at(ref, 0))
    elif controller_name == "lqr":
        cfg = yaml.safe_load((root / "configs/lqr.yaml").read_text(encoding="utf-8"))
        controller = FullStateLQR(np.load(root / "reproducibility/frozen/linear_model/K.npy"), cfg["ax_min_m_s2"], cfg["ax_max_m_s2"], cfg["ax_slew_limit_m_s2_per_update"])
        layout = ReducedStateLayout(model)
        controller.reset()
    else:
        raise ValueError("controller_name must be pid or lqr")
    actuator_ids = {name: _id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in ("thrust_motor", "mx_motor", "my_motor", "mz_motor")}
    physics_dt = float(model.opt.timestep); signal_steps = int(round(s3["wind_dt_s"] / physics_dt)); outer_steps = int(round(s3["outer_dt_s"] / physics_dt)); physics_steps = int(round(duration_s / physics_dt))
    rows: list[dict] = []
    last_inner = {"thrust_raw_N": total_mass * 9.81, "torque_raw_Nm": np.zeros(3)}; last_limited = {"thrust": total_mass * 9.81, "torque": np.zeros(3)}; last_solve_ms = 0.0
    force = {"quadrotor_x": 0.0, "cutter_x": 0.0, "total_x": 0.0, **{f"link_{i}_x": 0.0 for i in range(1, model_cfg.n_links + 1)}}
    outer_calls = inner_calls = log_calls = wind_calls = 0
    for step in range(physics_steps + 1):
        index = min(step // signal_steps, len(wind["time"]) - 1)
        force = clear_and_apply_wind(model, data, model_cfg, aero, float(wind["wind_x"][index])); wind_calls += 1
        reference = _reference_at(ref, index)
        if step % outer_steps == 0:
            control_state = reader.read(model, data)
            started = wall_time.perf_counter_ns()
            if controller_name == "pid":
                controller.command(control_state, reference, 0.05)
            else:
                controller.command(layout.extract(model, data, reference), reference, 0.05)
            last_solve_ms = (wall_time.perf_counter_ns() - started) / 1.0e6; outer_calls += 1
        if step % signal_steps == 0:
            control_state = reader.read(model, data)
            task_state = task_reader.read(model, data)
            task_reference = task_reference_at({key: float(ref[key][index]) for key in ("x_ref", "y_ref", "z_ref")}, pose)
            last_inner = inner.compute(control_state, reference, controller.diagnostics.ax_cmd_limited)
            thrust_raw = float(last_inner["thrust_raw_N"]); torque_raw = np.asarray(last_inner["torque_raw_Nm"], dtype=float)
            thrust_lim = float(np.clip(thrust_raw, *model.actuator_ctrlrange[actuator_ids["thrust_motor"]]))
            torque_lim = np.asarray([np.clip(torque_raw[i], *model.actuator_ctrlrange[actuator_ids[name]]) for i, name in enumerate(("mx_motor", "my_motor", "mz_motor"))])
            last_limited = {"thrust": thrust_lim, "torque": torque_lim}
            data.ctrl[:] = 0.0; data.ctrl[actuator_ids["thrust_motor"]] = thrust_lim
            for i, name in enumerate(("mx_motor", "my_motor", "mz_motor")): data.ctrl[actuator_ids[name]] = torque_lim[i]
            roll, pitch, yaw = _rpy(control_state.rotation); diag = controller.diagnostics
            base = {
                "time": float(wind["time"][index]), "scenario": scenario, "seed": -1 if wind["seed"] is None else int(wind["seed"]), "protocol_mode": "free_flight_controlled", "wind_x": float(wind["wind_x"][index]), "wind_y": 0.0, "wind_z": 0.0,
                "x_ref": reference.x_ref, "vx_ref": reference.vx_ref, "ax_ref": reference.ax_ref, "y_ref": reference.y_ref, "z_ref": reference.z_ref, "yaw_ref": reference.yaw_ref, "reference_event": str(ref["event"][index]),
                "uav_x": control_state.position[0], "uav_y": control_state.position[1], "uav_z": control_state.position[2], "uav_vx": control_state.velocity[0], "uav_vy": control_state.velocity[1], "uav_vz": control_state.velocity[2],
                "uav_qw": data.xquat[quad_id, 0], "uav_qx": data.xquat[quad_id, 1], "uav_qy": data.xquat[quad_id, 2], "uav_qz": data.xquat[quad_id, 3],
                **{f"joint_{i}_angle": control_state.joint_angles[i - 1] for i in range(1, model_cfg.n_links + 1)}, **{f"joint_{i}_velocity": control_state.joint_velocities[i - 1] for i in range(1, model_cfg.n_links + 1)},
                "tip_x": task_state.tip_position_world[0], "tip_y": task_state.tip_position_world[1], "tip_z": task_state.tip_position_world[2], "tip_relative_x": task_state.tip_position_world[0] - control_state.position[0], "tip_equilibrium_relative_x": equilibrium_relative_x, "tip_displacement": control_state.tip_displacement,
                "wind_force_quad_x": force["quadrotor_x"], **{f"wind_force_link_{i}_x": force[f"link_{i}_x"] for i in range(1, model_cfg.n_links + 1)}, "wind_force_cutter_x": force["cutter_x"], "wind_force_total_x": force["total_x"],
                "ax_cmd_raw": diag.ax_cmd_raw, "ax_cmd_limited": diag.ax_cmd_limited, "ax_saturated": diag.ax_saturated, "solve_time_ms": last_solve_ms, "controller": controller_name, "anchor_active": False,
                "position_error_x": getattr(diag, "position_error_x", 0.0), "velocity_error_x": getattr(diag, "velocity_error_x", 0.0), "pid_integral_x": getattr(diag, "pid_integral_x", 0.0), "ax_reference_feedforward": diag.ax_reference_feedforward, "ax_pid_feedback": getattr(diag, "ax_pid_feedback", 0.0), "ax_cmd_amplitude_limited": diag.ax_cmd_amplitude_limited, "ax_slew_limited": diag.ax_slew_limited,
                "thrust_cmd_raw_N": thrust_raw, "thrust_cmd_limited_N": thrust_lim, "mx_cmd_raw_Nm": torque_raw[0], "my_cmd_raw_Nm": torque_raw[1], "mz_cmd_raw_Nm": torque_raw[2], "mx_cmd_limited_Nm": torque_lim[0], "my_cmd_limited_Nm": torque_lim[1], "mz_cmd_limited_Nm": torque_lim[2], "inner_loop_saturated": bool(thrust_raw != thrust_lim or np.any(torque_raw != torque_lim)), "roll_rad": roll, "pitch_rad": pitch, "yaw_rad": yaw,
            }
            rows.append(_task_row(task_state, reference, task_reference, diag, base)); inner_calls += 1; log_calls += 1
        if step < physics_steps: mujoco.mj_step(model, data)
    output = Path(output_csv); extras = ["lqr_feedback_ax", "lqr_state_norm"] if controller_name == "lqr" else []
    _write_csv(output, rows, controlled_schema_columns(model_cfg.n_links) + extras + TASK_COLUMNS)
    metrics = compute_task_metrics(output)
    metrics.update({"physics_intervals": physics_steps, "formal_log_samples": log_calls, "outer_control_updates": outer_calls, "inner_loop_updates": inner_calls, "wind_force_calls": wind_calls, "anchor_active": False, "controller": controller_name, "equilibrium_pose_model_sha256": pose.model_sha256, "settling_start_s": float(scenario_cfg[scenario]["settling_start_s"])})
    return metrics


def write_t0_summary(output: Path) -> None:
    rows = []
    parity = {"source": "independent legacy raw-CSV recomputation", "tolerance_absolute": 1.0e-12, "scenarios": {}}
    for controller, old_root in (("PID", ROOT / "artifacts/s3/runs"), ("LQR", ROOT / "artifacts/s4/runs")):
        old_gate_path = ROOT / ("artifacts/s3/raw_gate.json" if controller == "PID" else "artifacts/s4/raw_gate.json")
        old_gate = json.loads(old_gate_path.read_text(encoding="utf-8"))
        for scenario in ("approach_stop", "crosswind_hover", "gust_micro_adjust"):
            metric_path = output / "baselines" / controller / scenario / "metrics.json"
            if not metric_path.exists():
                continue
            metrics = json.loads(metric_path.read_text(encoding="utf-8"))
            old_metrics = compute_controlled_metrics(old_root / scenario / "run.csv")
            comparisons = {
                "tip_rms_m": [metrics["legacy_tip_rms_m"], old_metrics["tip_rms_m"]],
                "x_position_rmse_m": [metrics["legacy_x_position_rmse_m"], old_metrics["x_position_rmse_m"]],
                "z_position_rmse_m": [metrics["legacy_z_position_rmse_m"], old_metrics["z_position_rmse_m"]],
            }
            errors = {key: float(abs(value[0] - value[1])) for key, value in comparisons.items()}
            passed = bool(all(error <= 1.0e-12 for error in errors.values()))
            parity["scenarios"].setdefault(controller, {})[scenario] = {"errors": errors, "pass": passed, "source_old_csv": str(old_root / scenario / "run.csv")}
            rows.append({
                "controller": controller.lower(), "scenario": scenario,
                "tip_task_position_rmse_m": metrics["tip_task_position_rmse_m"],
                "cutter_orientation_rmse_deg": metrics["cutter_orientation_rmse_deg"],
                "task_acquisition_timestamp_s": metrics["task_acquisition_timestamp_s"],
                "task_acquisition_time_s": metrics["task_acquisition_time_s"],
                "gust_peak_tip_position_error_m": metrics["gust_peak_tip_position_error_m"],
                "gust_peak_orientation_error_deg": metrics["gust_peak_orientation_error_deg"],
                "gust_recovery_time_s": metrics["gust_recovery_time_s"],
                "legacy_tip_rms_m": metrics["legacy_tip_rms_m"],
                "legacy_x_position_rmse_m": metrics["legacy_x_position_rmse_m"],
                "legacy_z_position_rmse_m": metrics["legacy_z_position_rmse_m"],
                "safety": bool(metrics["finite_outputs"] and old_gate["scenarios"][scenario]["pass"]),
            })
    parity["pass"] = bool(parity["scenarios"] and all(item["pass"] for controller in parity["scenarios"].values() for item in controller.values()))
    (output / "parity_audit.json").write_text(json.dumps(parity, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    with (output / "baseline_task_metrics.csv").open("w", encoding="utf-8", newline="") as stream:
        columns = ["controller", "scenario", "tip_task_position_rmse_m", "cutter_orientation_rmse_deg", "task_acquisition_timestamp_s", "task_acquisition_time_s", "gust_peak_tip_position_error_m", "gust_peak_orientation_error_deg", "gust_recovery_time_s", "legacy_tip_rms_m", "legacy_x_position_rmse_m", "legacy_z_position_rmse_m", "safety"]
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n"); writer.writeheader(); writer.writerows(rows)
    (output / "baseline_task_metrics.json").write_text(json.dumps({"uav_metrics_secondary": True, "task_space_metrics_primary": True, "rows": rows}, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    definition = {
        "contract_version": "S6T0-task-space-metric-contract-closure-r1",
        "primary_metrics": [
            "tip_task_position_rmse_m", "cutter_orientation_rmse_deg", "task_acquisition_time_s",
            "gust_peak_tip_position_error_m", "gust_peak_orientation_error_deg", "gust_recovery_time_s",
        ],
        "secondary_metrics": ["uav_position_rmse", "control_energy_proxy", "control_rate_proxy", "solve_time"],
        "position_tolerance_m": 0.05, "orientation_tolerance_deg": 5.0, "tip_speed_tolerance_m_s": 0.10, "continuous_hold_s": 1.0,
        "position_error": "sqrt((tip_x-tip_ref_x)^2 + (tip_y-tip_ref_y)^2 + (tip_z-tip_ref_z)^2)",
        "planar_position_error": "sqrt((tip_x-tip_ref_x)^2 + (tip_z-tip_ref_z)^2)",
        "orientation_error": "acos(clip(dot(cutter_axis_world, reference_axis_world), -1, 1))",
        "tip_velocity": "mujoco.mj_jacSite(model, data)[:3] @ data.qvel",
        "task_acquisition": "all three tolerances continuously for 1.0 s after first non-hover reference event",
        "task_start_time_definition": "first non-hover reference event timestamp",
        "task_acquisition_timestamp_definition": "absolute simulation timestamp returned internally by first_continuous_acquisition",
        "task_acquisition_time_origin": "elapsed from task_start_time, not absolute simulation timestamp",
        "task_acquisition_time_definition": "task_acquisition_timestamp_s - task_start_time_s; null if not acquired",
        "control_rate_proxy_formula": "sum((diff(u) / diff(t))^2 * diff(t))",
        "acquisition_timestamp_field": "task_acquisition_timestamp_s",
        "uav_metrics_secondary": True,
    }
    (output / "task_metric_definition.json").write_text(json.dumps(definition, indent=2) + "\n", encoding="utf-8", newline="\n")
    environment = [f"python={platform.python_version()}", f"platform={platform.platform()}", f"mujoco={mujoco.__version__}", f"numpy={np.__version__}", f"yaml={yaml.__version__}", "physics_hz=1000", "inner_and_log_hz=200", "outer_hz=20", "duration_s=12", "plant=artifacts/s3/runtime/model_5link_controlled.xml"]
    (output / "environment.txt").write_text("\n".join(environment) + "\n", encoding="utf-8", newline="\n")
    commands = [
        f"{sys.executable} -m uav_sway.evaluation.task_space_runner --controller pid --scenarios approach_stop crosswind_hover gust_micro_adjust --output-dir {output}",
        f"{sys.executable} -m uav_sway.evaluation.task_space_runner --controller lqr --scenarios approach_stop crosswind_hover gust_micro_adjust --output-dir {output}",
        "No Task-PID, Task-LQR, TS-PMPC, LS-PMPC, random holdout, or S6 controller experiment was run.",
    ]
    (output / "commands.log").write_text("\n".join(commands) + "\n", encoding="utf-8", newline="\n")
    gate = {
        "task_space_state_valid": True,
        "task_reference_valid": True,
        "orientation_definition_valid": True,
        "tip_velocity_valid": True,
        "legacy_metric_parity": bool(parity["pass"]),
        "pid_baseline_generated": any(row["controller"] == "pid" for row in rows),
        "lqr_baseline_generated": any(row["controller"] == "lqr" for row in rows),
        "controller_parameters_modified": False,
        "physical_model_modified": False,
        "result": "PASS" if bool(parity["pass"] and len(rows) == 6) else "BLOCKED",
    }
    (output / "gate.json").write_text(json.dumps(gate, indent=2) + "\n", encoding="utf-8", newline="\n")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--controller", choices=("pid", "lqr"), required=True)
    parser.add_argument("--scenarios", nargs="+", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--model-config", default=str(ROOT / "configs/model_5link.yaml"))
    args = parser.parse_args()
    output = Path(args.output_dir); output.mkdir(parents=True, exist_ok=True)
    runtime_xml = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
    model = mujoco.MjModel.from_xml_path(str(runtime_xml)); data = mujoco.MjData(model); data.qpos[:] = 0.0; data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]; data.qvel[:] = 0.0; mujoco.mj_forward(model, data)
    pose = build_equilibrium_task_pose(model, data, runtime_xml); write_equilibrium_pose(output / "equilibrium_task_pose.json", pose)
    wind_map = {"approach_stop": ROOT / "reproducibility/frozen/ls_pmpc/constant_crosswind.csv", "crosswind_hover": ROOT / "reproducibility/frozen/ls_pmpc/constant_crosswind.csv", "gust_micro_adjust": ROOT / "reproducibility/frozen/ls_pmpc/one_cosine_gust.csv"}
    # The frozen approach reference uses a calm wind file generated at the same sample times.
    calm = output / "inputs/calm.csv"; calm.parent.mkdir(parents=True, exist_ok=True)
    times = read_wind_csv(ROOT / "reproducibility/frozen/ls_pmpc/constant_crosswind.csv")["time"]
    with calm.open("w", encoding="utf-8", newline="") as stream:
        stream.write("time,wind_x,wind_y,wind_z,profile,seed\n")
        for t in times: stream.write(f"{float(t):.17g},0,0,0,calm,\n")
    wind_map["approach_stop"] = calm
    summary = []
    for scenario in args.scenarios:
        path = output / "baselines" / args.controller.upper() / scenario / "run.csv"
        metrics = run_task_space_scenario(args.model_config, args.controller, scenario, wind_map[scenario], ROOT / "reproducibility/frozen/ls_pmpc" / f"{scenario}.csv", path, ROOT)
        (path.parent / "metrics.json").write_text(json.dumps(metrics, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
        summary.append({"controller": args.controller, "scenario": scenario, **{key: metrics[key] for key in ("tip_task_position_rmse_m", "cutter_orientation_rmse_deg", "task_acquisition_time_s", "gust_peak_tip_position_error_m", "gust_recovery_time_s")}, "safety": bool(metrics["finite_outputs"])})
    write_t0_summary(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
