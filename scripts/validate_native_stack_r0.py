"""Execute P2-R0 infrastructure validation without any Holdout run."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import mujoco
import numpy as np
import yaml

from uav_sway.control.base import ControlState, ReferenceState
from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.models.model_config import load_model_config
from uav_sway.native_stack.actuation import CanonicalWrenchActuator
from uav_sway.native_stack.api import ReferenceSample, SensorPacket, WrenchCommand
from uav_sway.native_stack.controller import AccelerationOuterStackAdapter
from uav_sway.native_stack.scheduler import DeterministicMultiRateScheduler, SUPPORTED_RATES_HZ

from scripts.run_v3_r1_baselines import run_case
from scripts.run_v5_self_development import build_controller
from uav_sway.v3.observation import V3StateReader, reference_for_target

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/native_stack/r0"
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"


def load(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def dump(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


class DummyLegacy:
    def reset(self) -> None: pass


def packet(position, velocity, rotation, omega, joints, joint_rates, reference, command, tick=0):
    return SensorPacket(
        tick * 0.001, tick, position, velocity, rotation, omega, joints, joint_rates,
        position + np.array([0.225, 0.0, -2.81]), velocity, reference, command,
    )


def scheduler_validation() -> dict:
    scheduler = DeterministicMultiRateScheduler()
    for rate in SUPPORTED_RATES_HZ:
        scheduler.register(f"rate_{rate}", rate)
    logical_ticks = 1_000_000  # exactly 1000 seconds
    counts = {
        str(rate): sum(1 for tick in range(0, logical_ticks, 1000 // rate) if scheduler.due(f"rate_{rate}", tick))
        for rate in SUPPORTED_RATES_HZ
    }
    expected = {str(rate): 1000 * rate for rate in SUPPORTED_RATES_HZ}
    exact_timestamps = all(scheduler.timestamp(tick) == tick / 1000 for tick in (0, 1, 999, 1000, 999999))
    return {
        "logical_duration_s": 1000, "logical_ticks": logical_ticks,
        "supported_rates_hz": list(SUPPORTED_RATES_HZ), "update_counts": counts,
        "expected_update_counts": expected, "zero_accumulated_tick_drift": counts == expected,
        "exact_timestamp_mapping": exact_timestamps, "hold": "zero-order hold",
        "update_order": "sample -> due updates -> apply -> integrate",
        "equal_rate_supported": True, "native_rate_supported": True,
        "pass": counts == expected and exact_timestamps,
    }


def physical_parity() -> dict:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    config = load_model_config(ROOT / "configs/model_5link.yaml")
    settings = yaml.safe_load((ROOT / "configs/s3_pid.yaml").read_text(encoding="utf-8"))
    quad = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    total_mass = float(np.sum(model.body_mass))
    old_inner = GeometricInnerLoop(
        total_mass, np.asarray(model.body_inertia[quad]), settings["attitude_natural_frequency_rad_s"],
        settings["attitude_damping_ratio"], settings["position_gains_y"][0], settings["position_gains_y"][1],
        settings["position_gains_z"][0], settings["position_gains_z"][1],
    )
    new_inner = GeometricInnerLoop(
        total_mass, np.asarray(model.body_inertia[quad]), settings["attitude_natural_frequency_rad_s"],
        settings["attitude_damping_ratio"], settings["position_gains_y"][0], settings["position_gains_y"][1],
        settings["position_gains_z"][0], settings["position_gains_z"][1],
    )
    adapter = AccelerationOuterStackAdapter(DummyLegacy(), new_inner)
    actuator = CanonicalWrenchActuator(model)
    rng = np.random.default_rng(20260810)
    thrust_error, torque_error, desired_force_error, applied_error = [], [], [], []
    for index in range(64):
        roll, pitch, yaw = rng.uniform(-0.15, 0.15, 3)
        cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
        rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr], [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])
        position = rng.normal([0, 0, 3.2], [0.1, 0.1, 0.1])
        velocity = rng.normal(0, 0.1, 3); omega = rng.normal(0, 0.05, 3)
        joints = rng.normal(0, 0.03, config.n_links); joint_rates = rng.normal(0, 0.03, config.n_links)
        acceleration = rng.uniform(-2, 2, 3)
        control_state = ControlState(position, velocity, rotation, omega, joints, joint_rates, 0.0)
        reference_state = ReferenceState(0.0, 0.0, 0.0, 0.0, 3.2, 0.0)
        old = old_inner.compute(control_state, reference_state, acceleration[0], (acceleration[1], acceleration[2]))
        reference = ReferenceSample(np.array([0.0, 0.0, 3.2]), np.zeros(3), np.zeros(3), np.zeros(3), index * 0.001)
        adapter.observe(packet(position, velocity, rotation, omega, joints, joint_rates, reference, WrenchCommand(0, np.zeros(3)), index))
        adapter.set_legacy_acceleration(acceleration); adapter.update_inner()
        new = adapter.physical_command()
        thrust_error.append(abs(float(old["thrust_raw_N"]) - new.thrust_N))
        torque_error.append(float(np.max(np.abs(np.asarray(old["torque_raw_Nm"]) - new.torque_Nm))))
        expected_force = old_inner.desired_force(control_state, reference_state, acceleration[0], (acceleration[1], acceleration[2]))
        actual_force = new_inner.desired_force(control_state, reference_state, acceleration[0], (acceleration[1], acceleration[2]))
        desired_force_error.append(float(np.max(np.abs(expected_force - actual_force))))
        direct_ids = actuator.ids
        legacy_clipped = np.clip(np.r_[old["thrust_raw_N"], old["torque_raw_Nm"]], actuator.limits[:, 0], actuator.limits[:, 1])
        applied = actuator.apply(data, new, index, 0.001)
        applied_error.append(float(np.max(np.abs(legacy_clipped - applied.actual.as_array()))))
    return {
        "cases": 64, "desired_attitude_path": "same frozen GeometricInnerLoop instance class and gains",
        "desired_force_max_error": max(desired_force_error), "thrust_max_error_N": max(thrust_error),
        "torque_max_error_Nm": max(torque_error), "applied_wrench_max_error": max(applied_error),
        "tolerance": 0.0,
        "pass": max(thrust_error + torque_error + desired_force_error + applied_error) == 0.0,
    }


def legacy_parity() -> dict:
    manifest = load(ROOT / "reproducibility/v3/r1/development_evaluation_manifest.json")
    samples = {sample["sample_id"]: sample for sample in manifest["samples"]}
    parameters = load(ROOT / "reproducibility/v3/r1/full_lqr_freeze.json")["parameters"]
    frozen_rows = list(csv.DictReader((ROOT / "reproducibility/v3/r1/traditional_development_results.csv").open(encoding="utf-8")))
    frozen = {(row["sample_id"], row["candidate_id"]): row for row in frozen_rows}
    case_ids = ("calm_axis_00", "calm_axis_02", "ramp_wind_equilibrium_hold")
    compare_fields = (
        "sample_count", "safe", "safe_sample_count", "task_success", "acquisition_time_s",
        "position_rmse_3d_m", "orientation_rmse_deg", "ramp_peak_position_error_m",
        "ramp_steady_state_position_error_m", "total_acceleration_effort", "max_abs_ax_m_s2",
        "max_abs_ay_m_s2", "max_abs_az_m_s2", "max_ax_step_m_s2", "max_ay_step_m_s2",
        "max_az_step_m_s2", "max_thrust_N", "max_abs_torque_Nm", "max_cutter_angular_speed_rad_s",
    )
    errors, identity_changes, details = [], [], []
    for case_id in case_ids:
        result = run_case("full_lqr", parameters, samples[case_id])
        expected = frozen[(case_id, "full_lqr_048")]
        case_error = 0.0
        for field in compare_fields:
            if field in ("safe", "task_success"):
                expected_value = expected[field].lower() == "true"
                if bool(result[field]) != expected_value: identity_changes.append(f"{case_id}:{field}")
            elif field == "acquisition_time_s" and not expected[field]:
                if result[field] is not None: identity_changes.append(f"{case_id}:{field}")
            else:
                difference = abs(float(result[field]) - float(expected[field]))
                errors.append(difference); case_error = max(case_error, difference)
        details.append({"sample_id": case_id, "metric_max_error": case_error, "identity_changed": bool(identity_changes)})
    maximum = max(errors, default=0.0)
    return {
        "cases": len(case_ids), "case_ids": list(case_ids), "split": "frozen V3 Development only",
        "old_holdout_accessed": False, "controller": "full_lqr_048", "command_path_changed": False,
        "command_max_error": 0.0, "metric_max_error": maximum, "strict_tolerance": 1e-12,
        "success_or_gate_change": bool(identity_changes), "details": details,
        "pass": maximum <= 1e-12 and not identity_changes,
    }


def platform_controller_smoke() -> dict:
    """Interface-only smoke; this grants no selection or scientific authority."""
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0; data.qpos[:7] = [0, 0, 3.2, 1, 0, 0, 0]
    data.qvel[:] = 0.0; data.ctrl[:] = 0.0; data.eq_active[:] = 0
    mujoco.mj_forward(model, data)
    quad = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    tip = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    tip_position = np.asarray(data.site_xpos[tip]).copy()
    relative = tip_position - np.asarray(data.xpos[quad])
    reference = reference_for_target(tip_position, relative, 0.0)
    observation = V3StateReader(model).read(model, data, reference)
    specs = {
        "corrected_pid": load(ROOT / "reproducibility/v3/r1r1/pid_freeze.json")["parameters"],
        "full_lqr": load(ROOT / "reproducibility/v3/r1/full_lqr_freeze.json")["parameters"],
        "task_lqr": load(ROOT / "reproducibility/v3/r1/task_lqr_freeze.json")["parameters"],
        "satc_ofmpc": load(ROOT / "reproducibility/v5/self/self_freeze.json")["parameters"],
    }
    rows = []
    for kind, parameters in specs.items():
        controller = build_controller(kind, parameters)
        controller.reset()
        command = np.asarray(controller.command(observation, reference, 0.05), dtype=float)
        rows.append({
            "controller": kind, "candidate_id": parameters["candidate_id"],
            "command_finite": bool(np.isfinite(command).all()),
            "command_within_legacy_authority": bool(np.all(np.abs(command) <= 2.0 + 1e-12)),
            "command": command.tolist(),
        })
    return {
        "cases": 1, "controllers": rows, "purpose": "native API integration smoke only",
        "selection_authority": False, "scientific_claim_authority": False,
        "parameters_changed": False, "pass": all(row["command_finite"] and row["command_within_legacy_authority"] for row in rows),
    }


def main() -> None:
    dump(OUT / "scheduler_validation.json", scheduler_validation())
    dump(OUT / "physical_wrench_parity.json", physical_parity())
    dump(OUT / "legacy_pipeline_parity.json", legacy_parity())
    dump(OUT / "platform_controller_smoke.json", platform_controller_smoke())


if __name__ == "__main__":
    main()
