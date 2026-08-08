"""Headless anchored-wind S2 smoke runner; it contains no controller."""

from __future__ import annotations

import csv
from pathlib import Path

import mujoco
import numpy as np

from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind
from uav_sway.disturbances.wind_io import read_wind_csv
from uav_sway.evaluation.metrics import compute_metrics
from uav_sway.evaluation.schema import schema_columns
from uav_sway.models.model_config import load_model_config
from uav_sway.scenarios.scenario_config import load_scenario_config


def _id(model, object_type, name: str) -> int:
    result = int(mujoco.mj_name2id(model, object_type, name))
    if result < 0:
        raise KeyError(name)
    return result


def run_wind_validation(
    model_config_path: str | Path,
    scenario: str,
    wind_csv: str | Path,
    reference_csv: str | Path,
    output_csv: str | Path,
    aerodynamic_config_path: str | Path,
    scenarios_config_path: str | Path,
) -> dict:
    model_config = load_model_config(model_config_path)
    wind = read_wind_csv(wind_csv)
    scenario_config = load_scenario_config(scenarios_config_path)
    reference_rows = _read_reference(reference_csv)
    output_csv = Path(output_csv)
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    repo_root = Path(model_config_path).resolve().parents[1]
    xml_path = repo_root / "artifacts" / "s1" / "generated" / f"model_{model_config.n_links}link.xml"
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    if not np.isclose(model.opt.timestep, float(scenario_config["physics_dt"])):
        raise ValueError("S2 physics dt does not match generated model timestep")
    data = mujoco.MjData(model)
    data.qpos[:7] = np.array([0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0])
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    anchor_id = _id(model, mujoco.mjtObj.mjOBJ_EQUALITY, "passive_anchor")
    data.eq_active[:] = 0
    data.eq_active[anchor_id] = 1
    mujoco.mj_forward(model, data)
    quad_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
    tip_id = _id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip")
    joint_ids = [_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}") for i in range(1, model_config.n_links + 1)]
    joint_qpos = [model.jnt_qposadr[joint_id] for joint_id in joint_ids]
    joint_qvel = [model.jnt_dofadr[joint_id] for joint_id in joint_ids]
    equilibrium_relative_x = float(data.site_xpos[tip_id, 0] - data.xpos[quad_id, 0])
    aero = load_aerodynamic_config(aerodynamic_config_path)
    dt_signal = float(scenario_config["wind_and_log_dt"])
    physics_steps = int(round(float(scenario_config["duration_s"]) / model.opt.timestep))
    sample_every = int(round(dt_signal / model.opt.timestep))
    columns = schema_columns(model_config.n_links)
    reference_by_time = {float(row["time"]): row for row in reference_rows}
    writer_rows: list[dict] = []
    for step in range(physics_steps + 1):
        if step % sample_every == 0 or step == physics_steps:
            index = min(step // sample_every, len(wind["time"]) - 1)
            time = float(wind["time"][index])
            ref = reference_by_time[time]
            force = clear_and_apply_wind(model, data, model_config, aero, float(wind["wind_x"][index]))
            # The force is logged at the same 200 Hz sample at which it is applied.
            velocity = _body_velocity(model, data, quad_id)
            row = {
                "time": time, "scenario": scenario, "seed": -1 if wind["seed"] is None else int(wind["seed"]),
                "protocol_mode": "anchored_wind_validation", "wind_x": float(wind["wind_x"][index]), "wind_y": 0.0, "wind_z": 0.0,
                "x_ref": ref["x_ref"], "vx_ref": ref["vx_ref"], "ax_ref": ref["ax_ref"],
                "y_ref": ref["y_ref"], "z_ref": ref["z_ref"], "yaw_ref": ref["yaw_ref"],
                "uav_x": float(data.xpos[quad_id, 0]), "uav_y": float(data.xpos[quad_id, 1]), "uav_z": float(data.xpos[quad_id, 2]),
                "uav_vx": float(velocity[0]), "uav_vy": float(velocity[1]), "uav_vz": float(velocity[2]),
                "uav_qw": float(data.xquat[quad_id, 0]), "uav_qx": float(data.xquat[quad_id, 1]), "uav_qy": float(data.xquat[quad_id, 2]), "uav_qz": float(data.xquat[quad_id, 3]),
            }
            row.update({f"joint_{i}_angle": float(data.qpos[joint_qpos[i - 1]]) for i in range(1, model_config.n_links + 1)})
            row.update({f"joint_{i}_velocity": float(data.qvel[joint_qvel[i - 1]]) for i in range(1, model_config.n_links + 1)})
            tip = np.asarray(data.site_xpos[tip_id], dtype=float)
            row.update({
                "tip_x": float(tip[0]), "tip_y": float(tip[1]), "tip_z": float(tip[2]),
                "tip_relative_x": float(tip[0] - data.xpos[quad_id, 0]),
                "tip_equilibrium_relative_x": equilibrium_relative_x,
                "tip_displacement": float(tip[0] - data.xpos[quad_id, 0] - equilibrium_relative_x),
                "wind_force_quad_x": force["quadrotor_x"],
                "wind_force_cutter_x": force["cutter_x"],
                "wind_force_total_x": force["total_x"],
                "ax_cmd_raw": 0.0, "ax_cmd_limited": 0.0, "ax_saturated": False, "solve_time_ms": 0.0,
            })
            for i in range(1, model_config.n_links + 1):
                row[f"wind_force_link_{i}_x"] = force[f"link_{i}_x"]
            writer_rows.append(row)
        if step < physics_steps:
            # Every physical step starts with a clean applied-wrench buffer.
            current_index = min(step // sample_every, len(wind["time"]) - 1)
            clear_and_apply_wind(model, data, model_config, aero, float(wind["wind_x"][current_index]))
            mujoco.mj_step(model, data)
    with output_csv.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        for row in writer_rows:
            writer.writerow({column: _format_value(row[column]) for column in columns})
    metrics = compute_metrics(output_csv, settling_start_s=float(scenario_config[scenario]["settling_start_s"]))
    return metrics


def _format_value(value) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, str):
        return value
    return format(float(value), ".17g")


def _body_velocity(model, data, body_id: int) -> np.ndarray:
    jacp = np.zeros((3, model.nv), dtype=float)
    jacr = np.zeros((3, model.nv), dtype=float)
    mujoco.mj_jacBodyCom(model, data, jacp, jacr, body_id)
    return jacp @ data.qvel


def _read_reference(path: str | Path) -> list[dict[str, float]]:
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        return [{key: (float(value) if key not in {"event", "control_tick"} else (int(value) if key == "control_tick" else value)) for key, value in row.items()} for row in reader]
