"""Generate the frozen V9 identification bank without Development/Holdout access."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import mujoco
import numpy as np
import yaml

from uav_sway.control.base import ReferenceState
from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.control.state_reader import StateReader
from uav_sway.disturbances.aerodynamics import (
    body_com_velocity,
    box_projected_area,
    link_projected_area,
    load_aerodynamic_config,
)
from uav_sway.models.model_config import load_model_config
from uav_sway.v3.controllers import V3FullStateLQR
from uav_sway.v3.observation import V3StateReader, reference_for_target


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v9/r0"
OUT = ROOT / "reproducibility/v9/training"
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
DT = 0.001
OUTER_DT = 0.05
INNER_DT = 0.005
CONTRACT_HEAD = "858aab21860405d9d12f615f826147b1108e7586"


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
        newline="\n",
    )


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def wind_series(spec: dict, count: int) -> np.ndarray:
    kind = spec["kind"]
    result = np.zeros(count)
    if kind == "calm":
        return result
    if kind.startswith("constant_"):
        speed = 1.5 if kind == "constant_1p5" else 3.0
        result[int(round(1.0 / INNER_DT)) :] = speed
        return result
    if kind != "stochastic":
        raise ValueError(kind)
    rng = np.random.Generator(np.random.PCG64(int(spec["seed"])))
    alpha = math.exp(-INNER_DT / 0.8)
    for index in range(1, count):
        result[index] = np.clip(
            alpha * result[index - 1]
            + math.sqrt(1.0 - alpha * alpha) * 0.9 * float(rng.standard_normal()),
            -3.0,
            3.0,
        )
    return result


def apply_directional_wind(model, data, model_config, aerodynamic, direction, speed) -> None:
    data.xfrc_applied[:] = 0.0
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    body_specs = [
        ("quadrotor", aerodynamic.airframe_cd, "airframe", None),
        *[
            (f"link_{index}", aerodynamic.link_cd, "link", index)
            for index in range(1, model_config.n_links + 1)
        ],
        ("cutter", aerodynamic.cutter_cd, "cutter", None),
    ]
    for name, cd, kind, _ in body_specs:
        body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))
        rotation = np.asarray(data.xmat[body_id], dtype=float).reshape(3, 3)
        if kind == "airframe":
            area = box_projected_area(aerodynamic.airframe_dimensions, rotation, direction)
        elif kind == "cutter":
            area = box_projected_area(aerodynamic.cutter_dimensions, rotation, direction)
        else:
            area = link_projected_area(
                model_config.link_length,
                aerodynamic.link_diameter,
                rotation,
                direction,
            )
        velocity = body_com_velocity(model, data, body_id)
        relative = float(speed) - float(direction @ velocity)
        magnitude = 0.5 * aerodynamic.air_density * cd * area * abs(relative) * relative
        data.xfrc_applied[body_id, :3] += magnitude * direction


def target_delta(spec: dict, time_s: float) -> np.ndarray:
    amplitude = np.asarray(spec["amplitude_m"], dtype=float)
    frequency = np.asarray(spec["frequency_hz"], dtype=float)
    phase = np.asarray(spec["phase_rad"], dtype=float)
    raw = np.sin(2.0 * np.pi * frequency * time_s + phase) - np.sin(phase)
    envelope = min(1.0, max(0.0, time_s / 2.0))
    return amplitude * envelope * raw


def rpy(rotation: np.ndarray) -> tuple[float, float]:
    roll = float(math.atan2(rotation[2, 1], rotation[2, 2]))
    pitch = float(math.asin(np.clip(-rotation[2, 0], -1.0, 1.0)))
    return roll, pitch


def run_trajectory(spec: dict) -> dict:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    data.eq_active[:] = 0
    mujoco.mj_forward(model, data)
    model_config = load_model_config(ROOT / "configs/model_5link.yaml")
    s3 = yaml.safe_load((ROOT / "configs/s3_pid.yaml").read_text(encoding="utf-8"))
    aerodynamic = load_aerodynamic_config(ROOT / "configs/aerodynamics.yaml")
    full = read(ROOT / "reproducibility/v3/r1/full_lqr_freeze.json")["parameters"]
    outer = V3FullStateLQR(np.asarray(full["K"], dtype=float))
    outer.reset()
    quad_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    equilibrium_tip = np.asarray(data.site_xpos[tip_id], dtype=float).copy()
    equilibrium_relative = equilibrium_tip - np.asarray(data.xpos[quad_id], dtype=float)
    reader = V3StateReader(model)
    legacy = StateReader(model, model_config.n_links, float(equilibrium_relative[0]))
    total_mass = float(np.sum(model.body_mass))
    inertia = np.asarray(model.body_inertia[quad_id], dtype=float)
    inner = GeometricInnerLoop(
        total_mass,
        inertia,
        s3["attitude_natural_frequency_rad_s"],
        s3["attitude_damping_ratio"],
        s3["position_gains_y"][0],
        s3["position_gains_y"][1],
        s3["position_gains_z"][0],
        s3["position_gains_z"][1],
    )
    actuator_ids = {
        name: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name))
        for name in ("thrust_motor", "mx_motor", "my_motor", "mz_motor")
    }
    torque_limit = max(
        max(
            abs(model.actuator_ctrlrange[actuator_ids[name], 0]),
            abs(model.actuator_ctrlrange[actuator_ids[name], 1]),
        )
        for name in ("mx_motor", "my_motor", "mz_motor")
    )
    inner_count = int(round(float(spec["duration_s"]) / INNER_DT)) + 1
    winds = wind_series(spec["wind"], inner_count)
    outer_count = int(round(float(spec["duration_s"]) / OUTER_DT)) + 1
    state_error = np.zeros((outer_count, 20))
    zeta = np.zeros((outer_count, 6))
    chi = np.zeros((outer_count, 6))
    label_valid = np.zeros(outer_count, dtype=bool)
    commands = np.zeros((outer_count, 3))
    targets = np.zeros((outer_count, 3))
    actuation = np.zeros((outer_count, 6))
    safe_step = np.ones(outer_count, dtype=bool)
    safe = True
    max_abs_command = np.zeros(3)
    max_command_step = np.zeros(3)
    previous_command = np.zeros(3)
    previous_velocity = None
    previous_omega = None
    thrust_accumulator: list[np.ndarray] = []
    torque_accumulator: list[np.ndarray] = []
    command = np.zeros(3)
    reference = reference_for_target(equilibrium_tip, equilibrium_relative, 0.0)
    reference_state = ReferenceState(
        float(reference.uav_position_world[0]),
        0.0,
        0.0,
        float(reference.uav_position_world[1]),
        float(reference.uav_position_world[2]),
        0.0,
    )
    physics_steps = int(round(float(spec["duration_s"]) / DT))
    inner_stride = int(round(INNER_DT / DT))
    outer_stride = int(round(OUTER_DT / DT))
    outer_index = 0
    for step in range(physics_steps + 1):
        time_s = step * DT
        if step % inner_stride == 0:
            apply_directional_wind(
                model,
                data,
                model_config,
                aerodynamic,
                spec["wind"]["direction_world"],
                winds[min(step // inner_stride, len(winds) - 1)],
            )
        if step % outer_stride == 0:
            delta = target_delta(spec["reference"], time_s)
            target = equilibrium_tip + delta
            reference = reference_for_target(target, equilibrium_relative, time_s)
            reference_state = ReferenceState(
                float(reference.uav_position_world[0]),
                0.0,
                0.0,
                float(reference.uav_position_world[1]),
                float(reference.uav_position_world[2]),
                0.0,
            )
            observation = reader.read(model, data, reference)
            control_state = legacy.read(model, data)
            velocity = np.asarray(control_state.velocity, dtype=float)
            omega = np.asarray(control_state.body_angular_velocity, dtype=float)
            state_error[outer_index] = observation.full_state_error
            zeta[outer_index] = np.r_[velocity, omega]
            targets[outer_index] = target
            safe_step[outer_index] = safe
            if previous_velocity is not None and thrust_accumulator:
                mean_thrust_world = np.mean(thrust_accumulator, axis=0)
                mean_torque = np.mean(torque_accumulator, axis=0)
                acceleration = (velocity - previous_velocity) / OUTER_DT
                angular_acceleration = (omega - previous_omega) / OUTER_DT
                force_external = (
                    total_mass * (acceleration - np.asarray([0.0, 0.0, -9.81]))
                    - mean_thrust_world
                )
                torque_external = (
                    inertia * angular_acceleration
                    + np.cross(previous_omega, inertia * previous_omega)
                    - mean_torque
                )
                chi[outer_index] = np.r_[force_external, torque_external]
                actuation[outer_index] = np.r_[mean_thrust_world, mean_torque]
                label_valid[outer_index] = True
            previous_velocity = velocity.copy()
            previous_omega = omega.copy()
            thrust_accumulator.clear()
            torque_accumulator.clear()
            command = outer.command(observation, reference, OUTER_DT)
            commands[outer_index] = command
            max_abs_command = np.maximum(max_abs_command, np.abs(command))
            max_command_step = np.maximum(max_command_step, np.abs(command - previous_command))
            previous_command = command.copy()
            roll, pitch = rpy(control_state.rotation)
            safe = safe and bool(
                np.isfinite(np.r_[observation.full_state_error, command]).all()
                and control_state.position[2] > 0.05
                and observation.task_state.tip_position_world[2] > 0.05
                and np.max(np.abs(control_state.joint_angles)) < np.deg2rad(100.0)
                and abs(roll) < np.deg2rad(25.0)
                and abs(pitch) < np.deg2rad(25.0)
                and np.max(np.abs(command)) <= 2.0 + 1.0e-9
                and np.max(np.abs(command - commands[max(0, outer_index - 1)]))
                <= 0.25 + 1.0e-9
            )
            outer_index += 1
        if step % inner_stride == 0:
            control_state = legacy.read(model, data)
            output = inner.compute(
                control_state,
                reference_state,
                float(command[0]),
                (float(command[1]), float(command[2])),
            )
            thrust_raw = float(output["thrust_raw_N"])
            torque_raw = np.asarray(output["torque_raw_Nm"], dtype=float)
            thrust = float(
                np.clip(
                    thrust_raw,
                    *model.actuator_ctrlrange[actuator_ids["thrust_motor"]],
                )
            )
            torque = np.asarray(
                [
                    np.clip(
                        torque_raw[index],
                        *model.actuator_ctrlrange[actuator_ids[name]],
                    )
                    for index, name in enumerate(("mx_motor", "my_motor", "mz_motor"))
                ]
            )
            data.ctrl[:] = 0.0
            data.ctrl[actuator_ids["thrust_motor"]] = thrust
            for value, name in zip(torque, ("mx_motor", "my_motor", "mz_motor")):
                data.ctrl[actuator_ids[name]] = value
            rotation = np.asarray(control_state.rotation, dtype=float).reshape(3, 3)
            thrust_accumulator.append(rotation @ np.asarray([0.0, 0.0, thrust]))
            torque_accumulator.append(torque)
            safe = safe and bool(
                thrust_raw
                <= model.actuator_ctrlrange[actuator_ids["thrust_motor"], 1] + 1.0e-9
                and np.max(np.abs(torque_raw)) <= torque_limit + 1.0e-9
            )
        if step < physics_steps:
            mujoco.mj_step(model, data)
    if outer_index != outer_count:
        raise RuntimeError(f"outer count mismatch {outer_index} != {outer_count}")
    return {
        "trajectory_id": spec["trajectory_id"],
        "split": spec["split"],
        "state_error": state_error,
        "zeta": zeta,
        "chi": chi,
        "label_valid": label_valid,
        "commands": commands,
        "targets": targets,
        "actuation": actuation,
        "safe_step": safe_step,
        "safe": safe,
        "max_abs_command": max_abs_command,
        "max_command_step": max_command_step,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=16)
    args = parser.parse_args()
    manifest = read(R0 / "identification_manifest.json")
    if manifest["trajectory_count"] != 240:
        raise RuntimeError("identification contract drift")
    started = time.perf_counter()
    values: dict[str, dict] = {}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures = {
            pool.submit(run_trajectory, spec): spec["trajectory_id"]
            for spec in manifest["trajectories"]
        }
        for index, future in enumerate(as_completed(futures), 1):
            trajectory_id = futures[future]
            values[trajectory_id] = future.result()
            if index % 12 == 0 or index == len(futures):
                print(f"identification {index}/{len(futures)}", flush=True)
    ordered = [values[spec["trajectory_id"]] for spec in manifest["trajectories"]]
    OUT.mkdir(parents=True, exist_ok=True)
    bank = OUT / "training_bank.npz"
    np.savez_compressed(
        bank,
        trajectory_id=np.asarray([row["trajectory_id"] for row in ordered]),
        split=np.asarray([row["split"] for row in ordered]),
        state_error=np.asarray([row["state_error"] for row in ordered]),
        zeta=np.asarray([row["zeta"] for row in ordered]),
        chi=np.asarray([row["chi"] for row in ordered]),
        label_valid=np.asarray([row["label_valid"] for row in ordered]),
        command=np.asarray([row["commands"] for row in ordered]),
        target=np.asarray([row["targets"] for row in ordered]),
        actual_actuation=np.asarray([row["actuation"] for row in ordered]),
        safe_step=np.asarray([row["safe_step"] for row in ordered]),
        safe_trajectory=np.asarray([row["safe"] for row in ordered]),
        max_abs_command=np.asarray([row["max_abs_command"] for row in ordered]),
        max_command_step=np.asarray([row["max_command_step"] for row in ordered]),
    )
    summary = {
        "contract_head": CONTRACT_HEAD,
        "trajectory_count": len(ordered),
        "train_trajectories": sum(row["split"] == "TRAIN" for row in ordered),
        "validation_trajectories": sum(row["split"] == "VALIDATION" for row in ordered),
        "timesteps": int(sum(row["state_error"].shape[0] for row in ordered)),
        "valid_label_timesteps": int(sum(np.sum(row["label_valid"]) for row in ordered)),
        "safe_trajectories": int(sum(row["safe"] for row in ordered)),
        "safe_valid_label_timesteps": int(
            sum(np.sum(row["label_valid"] & row["safe_step"]) for row in ordered)
        ),
        "max_abs_command_m_s2": float(
            max(np.max(row["max_abs_command"]) for row in ordered)
        ),
        "max_command_step_m_s2": float(
            max(np.max(row["max_command_step"]) for row in ordered)
        ),
        "wind_truth_used_as_input": False,
        "wind_truth_saved_in_training_bank": False,
        "future_state_used_as_runtime_input": False,
        "predictor_fit_mask": "label_valid AND safe_step; unsafe tails retained only for audit",
        "label_reconstruction": "finite-difference rigid-body acceleration minus actual thrust/torque and nominal gravity/Coriolis",
        "split_unit": "whole trajectory",
        "holdout_accessed": False,
        "development_accessed": False,
        "workers": args.workers,
        "elapsed_s": time.perf_counter() - started,
        "training_bank_bytes": bank.stat().st_size,
        "training_bank_sha256": sha(bank),
        "identification_manifest_sha256": sha(R0 / "identification_manifest.json"),
    }
    write(OUT / "generation_summary.json", summary)
    write(
        OUT / "training_data_freeze.json",
        {
            "result": "V9_TRAINING_DATA_FROZEN",
            "bank": "reproducibility/v9/training/training_bank.npz",
            "bank_sha256": sha(bank),
            "manifest": "reproducibility/v9/r0/identification_manifest.json",
            "manifest_sha256": sha(R0 / "identification_manifest.json"),
            "trajectory_split_frozen": True,
            "predictor_fit_mask": "label_valid AND safe_step",
            "unsafe_data_policy": "preserved for audit and excluded from fitting/model selection",
            "training_allowed_after_this_freeze": True,
            "holdout_accessed": False,
        },
    )
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
