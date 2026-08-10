"""Noncausal Development-only feasibility oracle for P2-R0G.

This module is intentionally outside the benchmark controller registry.  It
uses information forbidden to controllers and therefore has no comparison or
selection authority.  Its only question is whether a frozen Development case
can reach its endpoint under the unchanged plant and actuator limits.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import perf_counter_ns
from typing import Any

import mujoco
import numpy as np
from udaan.control.quadrotor import GeometricAttitudeController
from udaan.manif import SO3, TSO3

from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind_world
from uav_sway.models.model_config import load_model_config

from .actuation import CanonicalWrenchActuator
from .api import ReferenceSample, WrenchCommand
from .case_semantics.resolver import NativeCaseResolver, ResolvedNativeCase, ResolvedReference
from .logging import NativeCommandLogger
from .r1r1_controllers import GRAVITY, INERTIA, MASS_KG, UAV_MINUS_TIP_TRIM
from .safety import evaluate_safety_v2
from .sensors import NativeSensorReader


ROOT = Path(__file__).resolve().parents[3]
ORACLE_AUTHORITY = "NONCAUSAL_DIAGNOSTIC_FEASIBILITY_ORACLE_NO_SELECTION_AUTHORITY"


@dataclass(frozen=True)
class OracleParameters:
    setpoint_transition_s: float = 4.0
    translation_natural_frequency_rad_s: float = 1.2
    translation_damping_ratio: float = 1.0
    tip_correction_gain: float = 0.65
    tip_velocity_correction_s: float = 0.30
    tip_correction_limit_m: float = 0.75
    acceleration_limit_m_s2: float = 4.0
    attitude_natural_frequency_rad_s: float = 8.0
    attitude_damping_ratio: float = 1.0
    outer_rate_hz: int = 200
    inner_rate_hz: int = 1000


def _quintic(progress: float, duration: float) -> tuple[float, float, float]:
    u = float(np.clip(progress, 0.0, 1.0))
    position = 10*u**3 - 15*u**4 + 6*u**5
    velocity = (30*u**2 - 60*u**3 + 30*u**4) / duration
    acceleration = (60*u - 180*u**2 + 120*u**3) / duration**2
    if progress <= 0.0 or progress >= 1.0:
        velocity = 0.0
        acceleration = 0.0
    return position, velocity, acceleration


class OfflineOracleReference:
    """Full-future reference; setpoint steps are shaped without changing cases."""

    def __init__(self, case: ResolvedNativeCase, parameters: OracleParameters) -> None:
        self.case = case
        self.parameters = parameters
        self.original = ResolvedReference(case)
        self.start = np.asarray(case.target["initial_cutter_target_world_m"], dtype=float)
        self.goal = np.asarray(case.target["final_cutter_target_world_m"], dtype=float)

    def sample(self, time_s: float) -> ReferenceSample:
        if self.case.identity["task_family"] != "setpoint":
            return self.original.sample(time_s)
        duration = self.parameters.setpoint_transition_s
        progress = (time_s - self.case.issue_offset_s) / duration
        p, v, a = _quintic(progress, duration)
        delta = self.goal - self.start
        return ReferenceSample(
            self.start + p * delta, v * delta, a * delta, np.zeros(3), time_s,
        )


class FixedComputedForceOracle:
    def __init__(self, parameters: OracleParameters) -> None:
        self.parameters = parameters
        self.attitude = GeometricAttitudeController(inertia=np.diag(INERTIA))
        wn = parameters.attitude_natural_frequency_rad_s
        zeta = parameters.attitude_damping_ratio
        self.attitude._gains.kp = INERTIA * wn**2
        self.attitude._gains.kd = 2.0 * zeta * INERTIA * wn
        self.desired_acceleration = np.zeros(3)

    def outer(self, packet, oracle_reference: ReferenceSample) -> None:
        tip_error = oracle_reference.position_world - packet.cutter_tip_position_world
        tip_velocity_error = oracle_reference.velocity_world - packet.cutter_tip_velocity_world
        correction = (
            self.parameters.tip_correction_gain * tip_error
            + self.parameters.tip_velocity_correction_s * tip_velocity_error
        )
        norm = float(np.linalg.norm(correction))
        if norm > self.parameters.tip_correction_limit_m:
            correction *= self.parameters.tip_correction_limit_m / norm
        uav_target = oracle_reference.position_world + UAV_MINUS_TIP_TRIM + correction
        position_error = uav_target - packet.uav_position_world
        velocity_error = oracle_reference.velocity_world - packet.uav_velocity_world
        wn = self.parameters.translation_natural_frequency_rad_s
        acceleration = (
            oracle_reference.acceleration_world
            + wn**2 * position_error
            + 2.0 * self.parameters.translation_damping_ratio * wn * velocity_error
        )
        norm = float(np.linalg.norm(acceleration))
        if norm > self.parameters.acceleration_limit_m_s2:
            acceleration *= self.parameters.acceleration_limit_m_s2 / norm
        self.desired_acceleration = acceleration

    def command(self, packet, external_force_world: np.ndarray) -> WrenchCommand:
        desired_force = MASS_KG * (self.desired_acceleration + np.array([0.0, 0.0, GRAVITY])) - external_force_world
        thrust, torque = self.attitude.compute(
            0.0,
            (SO3(packet.rotation_world_from_body), TSO3(packet.body_angular_velocity)),
            desired_force,
        )
        return WrenchCommand(float(thrust), np.asarray(torque, dtype=float))


def evaluate_oracle_case(identity: dict[str, Any]) -> dict[str, Any]:
    parameters = OracleParameters()
    resolver = NativeCaseResolver()
    case = resolver.resolve(identity)
    if case.split != "development" or not case.execution["execution_allowed"]:
        raise PermissionError("oracle is restricted to executable Development identities")
    if not resolver.verify_fingerprint(case):
        raise AssertionError("Development semantic fingerprint mismatch")

    model = mujoco.MjModel.from_xml_path(str(ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0
    data.eq_active[:] = 0
    mujoco.mj_forward(model, data)

    model_config = load_model_config(ROOT / "configs/model_5link.yaml")
    aerodynamic = load_aerodynamic_config(ROOT / "configs/aerodynamics.yaml")
    signals = resolver.canonical_signals(case)
    original_reference = ResolvedReference(case)
    oracle_reference = OfflineOracleReference(case, parameters)
    controller = FixedComputedForceOracle(parameters)
    sensor_reader = NativeSensorReader(model)
    actuator = CanonicalWrenchActuator(model)
    logger = NativeCommandLogger(float(model.opt.timestep))
    quad = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    tip = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    origin_uav = np.asarray(data.xpos[quad]).copy()
    origin_tip = np.asarray(data.site_xpos[tip]).copy()
    previous = WrenchCommand(0.0, np.zeros(3))
    reasons: set[str] = set()
    metric_records: list[tuple[Any, ...]] = []
    outer_ms: list[float] = []
    inner_ms: list[float] = []
    max_external_force = 0.0
    outer_period = int(round(1000 / parameters.outer_rate_hz))
    inner_period = int(round(1000 / parameters.inner_rate_hz))
    ticks = int(round(case.duration_s / float(model.opt.timestep)))

    for tick in range(ticks):
        time_s = tick * float(model.opt.timestep)
        wind_index = min(tick, len(signals["wind_world"]) - 1)
        wind = clear_and_apply_wind_world(
            model, data, model_config, aerodynamic, signals["wind_world"][wind_index],
        )
        external_force = np.asarray(wind["total_world"], dtype=float)
        max_external_force = max(max_external_force, float(np.linalg.norm(external_force)))
        original = original_reference.sample(time_s)
        shaped = oracle_reference.sample(time_s)
        packet = sensor_reader.read(model, data, original, previous, tick, float(model.opt.timestep))
        components: list[str] = []
        outer_elapsed = 0.0
        inner_elapsed = 0.0
        if tick % outer_period == 0:
            started = perf_counter_ns()
            controller.outer(packet, shaped)
            outer_elapsed = (perf_counter_ns() - started) * 1e-6
            outer_ms.append(outer_elapsed)
            components.append("outer")
        started = perf_counter_ns()
        requested = controller.command(packet, external_force)
        inner_elapsed = (perf_counter_ns() - started) * 1e-6
        if tick % inner_period == 0:
            inner_ms.append(inner_elapsed)
            components.append("inner")
        applied = actuator.apply(data, requested, tick, float(model.opt.timestep))
        logger.append(applied, tuple(components), outer_elapsed, inner_elapsed)
        safe, tick_reasons = evaluate_safety_v2(packet, applied, origin_uav, origin_tip)
        if not safe:
            reasons.update(tick_reasons)
        previous = applied.actual
        if tick % 10 == 0:
            rotation = packet.rotation_world_from_body
            attitude_angle = float(np.arccos(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)))
            cutter_orientation = attitude_angle + abs(float(np.sum(packet.joint_position)))
            metric_records.append((
                time_s, packet.cutter_tip_position_world.copy(), packet.cutter_tip_velocity_world.copy(),
                original.position_world.copy(), original.velocity_world.copy(), cutter_orientation,
            ))
        mujoco.mj_step(model, data)

    time = np.asarray([row[0] for row in metric_records])
    position = np.asarray([row[1] for row in metric_records])
    velocity = np.asarray([row[2] for row in metric_records])
    reference = np.asarray([row[3] for row in metric_records])
    reference_velocity = np.asarray([row[4] for row in metric_records])
    orientation = np.asarray([row[5] for row in metric_records])
    position_error = np.linalg.norm(position - reference, axis=1)
    velocity_error = np.linalg.norm(velocity - reference_velocity, axis=1)
    mask = time >= case.issue_offset_s
    commands = np.asarray([row.actual for row in logger.records])
    thrust_saturated = np.asarray([row.thrust_saturated for row in logger.records])
    torque_saturated = np.asarray([any(row.torque_saturated) for row in logger.records])
    final_position_error = float(position_error[-1])
    final_velocity_error = float(velocity_error[-1])
    safe = not reasons
    success = safe and final_position_error <= 0.5 and final_velocity_error <= 0.5
    max_thrust = float(np.max(commands[:, 0]))
    max_torque = np.max(np.abs(commands[:, 1:]), axis=0)
    if success:
        mechanism = "ENDPOINT_REACHED"
    elif not safe:
        mechanism = "SAFETY_OR_AUTHORITY_LIMIT"
    elif final_position_error > 0.5 and final_velocity_error > 0.5:
        mechanism = "POSITION_AND_VELOCITY_ENDPOINT"
    elif final_position_error > 0.5:
        mechanism = "POSITION_ENDPOINT"
    else:
        mechanism = "VELOCITY_ENDPOINT"
    return {
        "sample_id": case.sample_id,
        "task_family": case.identity["task_family"],
        "wind_kind": case.identity["wind_kind"],
        "wind_direction": case.identity["wind_direction"],
        "safe": safe,
        "success": success,
        "safety_reasons": sorted(reasons),
        "failure_mechanism": mechanism,
        "endpoint_position_error_m": final_position_error,
        "endpoint_velocity_error_m_s": final_velocity_error,
        "position_rmse_m": float(np.sqrt(np.mean(position_error[mask] ** 2))),
        "velocity_rmse_m_s": float(np.sqrt(np.mean(velocity_error[mask] ** 2))),
        "orientation_rmse_rad": float(np.sqrt(np.mean(orientation[mask] ** 2))),
        "max_requested_or_applied_thrust_N": max_thrust,
        "max_requested_or_applied_torque_Nm": max_torque.tolist(),
        "thrust_saturation_rate": float(np.mean(thrust_saturated)),
        "torque_saturation_rate": float(np.mean(torque_saturated)),
        "max_true_external_force_N": max_external_force,
        "runtime_mean_ms": float(np.mean(outer_ms + inner_ms)),
        "runtime_p95_ms": float(np.percentile(outer_ms + inner_ms, 95)),
        "semantic_fingerprint": case.case_semantic_fingerprint,
        "execution_authority": ORACLE_AUTHORITY,
    }
