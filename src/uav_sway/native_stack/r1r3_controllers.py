"""Governance-v2 Traditional and SATC controllers for P2-R1R3.

The Traditional implementations remain causal classical feedback laws.  The
SATC implementation is intentionally unavailable to the runner until the
Traditional gate has been frozen by the orchestration layer.
"""

from __future__ import annotations

from typing import Any

import numpy as np
from scipy.linalg import solve_continuous_are

from .api import SensorPacket
from .r1r1_controllers import NativeGains, NativeWrenchController, UAV_MINUS_TIP_TRIM


def _native_gains(parameters: dict[str, Any], outer_rate_hz: int) -> NativeGains:
    return NativeGains(
        kp=tuple(parameters.get("kp", (0.0, 0.0, 0.0))),
        kd=tuple(parameters.get("kd", (0.0, 0.0, 0.0))),
        ki=tuple(parameters.get("ki", (0.0, 0.0, 0.0))),
        attitude_wn=float(parameters["attitude_wn"]),
        attitude_zeta=float(parameters.get("attitude_zeta", 0.95)),
        integral_limit=float(parameters["integral_limit"]),
        acceleration_limit=float(parameters["acceleration_limit"]),
        acceleration_slew_per_outer_update=float(parameters["acceleration_slew_per_s"]) / outer_rate_hz,
        swing_angle_gain=float(parameters["swing_angle_gain"]),
        swing_rate_gain=float(parameters["swing_rate_gain"]),
        transient_coordination=float(parameters.get("transient_coordination", 0.0)),
        constraint_margin=float(parameters.get("constraint_margin", 0.85)),
        tip_correction_kp=float(parameters["tip_correction_kp"]),
        tip_correction_kd=float(parameters["tip_correction_kd"]),
        tip_correction_limit_m=float(parameters["tip_correction_limit_m"]),
    )


class GovernanceV2PID(NativeWrenchController):
    """Independent-axis PID/PD servo with causal feedforward and anti-windup."""

    architecture = "governance_v2_independent_axis_PID_PD_SO3_wrench"

    def __init__(self, parameters: dict[str, Any], method_id: str, outer_rate_hz: int) -> None:
        self.parameters = dict(parameters)
        super().__init__(_native_gains(parameters, outer_rate_hz), method_id, 1.0 / outer_rate_hz)

    def _task_acceleration(self, packet: SensorPacket) -> np.ndarray:
        p = self.parameters
        tip_error = packet.reference.position_world - packet.cutter_tip_position_world
        relative_tip_velocity = packet.cutter_tip_velocity_world - packet.uav_velocity_world
        correction = np.clip(
            p["tip_correction_kp"] * tip_error - p["tip_correction_kd"] * relative_tip_velocity,
            -p["tip_correction_limit_m"], p["tip_correction_limit_m"],
        )
        target = packet.reference.position_world + UAV_MINUS_TIP_TRIM + correction
        error = target - packet.uav_position_world
        velocity_error = p["reference_velocity_gain"] * packet.reference.velocity_world - packet.uav_velocity_world
        setpoint_mode = (
            np.linalg.norm(packet.reference.velocity_world) <= p.get("setpoint_velocity_threshold", 0.0)
            and np.linalg.norm(packet.reference.acceleration_world) <= p.get("setpoint_acceleration_threshold", 0.0)
        )
        kp_scale = p.get("setpoint_terminal_kp_scale", 1.0) if setpoint_mode else 1.0
        kd_scale = p.get("setpoint_terminal_kd_scale", 1.0) if setpoint_mode else 1.0
        proposed = np.clip(self._integral + error * self.outer_dt, -p["integral_limit"], p["integral_limit"])
        acceleration = (
            p["reference_acceleration_gain"] * packet.reference.acceleration_world
            + kp_scale * np.asarray(p["kp"]) * error
            + kd_scale * np.asarray(p["kd"]) * velocity_error
            + np.asarray(p["ki"]) * proposed
        )
        swing = float(np.sum(packet.joint_position))
        swing_rate = float(np.sum(packet.joint_velocity))
        acceleration[0] -= p["swing_angle_gain"] * swing + p["swing_rate_gain"] * swing_rate
        limit = float(p["acceleration_limit"])
        norm = float(np.linalg.norm(acceleration))
        if norm <= limit:
            self._integral = proposed
        elif norm > 0.0:
            acceleration *= limit / norm
        return acceleration

    def diagnostics(self) -> dict[str, Any]:
        return {**super().diagnostics(), "parameters": self.parameters}


class GovernanceV2LQI(NativeWrenchController):
    """Causal task-reference LQI servo followed by the frozen SO(3) wrench layer.

    The gains are generated from three independent augmented double-integrator
    CARE problems.  This avoids the direct nonlinear torque command that caused
    the R1R2 physical-LQI runaway while retaining a classical LQI identity.
    """

    architecture = "governance_v2_9_state_translational_LQI_SO3_wrench"

    def __init__(self, parameters: dict[str, Any], method_id: str, outer_rate_hz: int) -> None:
        self.parameters = dict(parameters)
        super().__init__(_native_gains(parameters, outer_rate_hz), method_id, 1.0 / outer_rate_hz)
        q_position = np.asarray(parameters["q_position"], dtype=float)
        q_velocity = np.asarray(parameters["q_velocity"], dtype=float)
        q_integral = np.asarray(parameters["q_integral"], dtype=float)
        r_acceleration = np.asarray(parameters["r_acceleration"], dtype=float)
        a = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
        b = np.array([[0.0], [1.0], [0.0]])
        gains = []
        poles = []
        for axis in range(3):
            q = np.diag([q_position[axis], q_velocity[axis], q_integral[axis]])
            r = np.array([[r_acceleration[axis]]])
            solution = solve_continuous_are(a, b, q, r)
            gain = np.linalg.solve(r, b.T @ solution).reshape(3)
            gains.append(gain)
            poles.append(np.linalg.eigvals(a - b @ gain.reshape(1, 3)))
        self._lqi_gain = np.asarray(gains)
        self.audit = {
            "state_schema": ["task_position_error", "task_velocity_error", "integral_task_error"],
            "per_axis_gain": self._lqi_gain.tolist(),
            "closed_loop_poles": [[[float(v.real), float(v.imag)] for v in axis] for axis in poles],
            "causal": True,
            "online_optimization": False,
        }

    def _task_acceleration(self, packet: SensorPacket) -> np.ndarray:
        p = self.parameters
        tip_error = packet.reference.position_world - packet.cutter_tip_position_world
        relative_tip_velocity = packet.cutter_tip_velocity_world - packet.uav_velocity_world
        correction = np.clip(
            p["tip_correction_kp"] * tip_error - p["tip_correction_kd"] * relative_tip_velocity,
            -p["tip_correction_limit_m"], p["tip_correction_limit_m"],
        )
        target = packet.reference.position_world + UAV_MINUS_TIP_TRIM + correction
        position_error = packet.uav_position_world - target
        velocity_error = packet.uav_velocity_world - p["reference_velocity_gain"] * packet.reference.velocity_world
        proposed = np.clip(
            self._integral + position_error * self.outer_dt,
            -p["integral_limit"], p["integral_limit"],
        )
        state = np.column_stack((position_error, velocity_error, proposed))
        setpoint_mode = (
            np.linalg.norm(packet.reference.velocity_world) <= p.get("setpoint_velocity_threshold", 0.0)
            and np.linalg.norm(packet.reference.acceleration_world) <= p.get("setpoint_acceleration_threshold", 0.0)
        )
        gain = self._lqi_gain.copy()
        if setpoint_mode:
            gain[:, 0] *= p.get("setpoint_terminal_kp_scale", 1.0)
            gain[:, 1] *= p.get("setpoint_terminal_kd_scale", 1.0)
        acceleration = -np.sum(gain * state, axis=1)
        acceleration += p["reference_acceleration_gain"] * packet.reference.acceleration_world
        swing = float(np.sum(packet.joint_position))
        swing_rate = float(np.sum(packet.joint_velocity))
        acceleration[0] -= p["swing_angle_gain"] * swing + p["swing_rate_gain"] * swing_rate
        limit = float(p["acceleration_limit"])
        norm = float(np.linalg.norm(acceleration))
        if norm <= limit:
            self._integral = proposed
        elif norm > 0.0:
            acceleration *= limit / norm
        return acceleration

    def diagnostics(self) -> dict[str, Any]:
        return {**super().diagnostics(), "parameters": self.parameters, "lqi_audit": self.audit}


class GovernanceV2SATC(GovernanceV2PID):
    """Shock-aware, constraint-aware causal SATC-Native challenger."""

    architecture = "governance_v2_shock_constraint_conflict_SATC_native_wrench"

    def _task_acceleration(self, packet: SensorPacket) -> np.ndarray:
        acceleration = super()._task_acceleration(packet)
        p = self.parameters
        conflict = packet.reference.velocity_world - packet.cutter_tip_velocity_world
        shock = np.tanh(
            p["shock_jerk_weight"] * np.linalg.norm(packet.reference.jerk_world)
            + p["shock_conflict_weight"] * np.linalg.norm(conflict)
        )
        predicted = conflict + p["prediction_horizon_s"] * (
            packet.reference.acceleration_world - acceleration
        )
        coordination = p["transient_coordination"] * shock * predicted
        acceleration = acceleration + coordination
        limit = p["acceleration_limit"] * p["constraint_margin"]
        norm = float(np.linalg.norm(acceleration))
        if norm > limit:
            acceleration *= limit / norm
        return acceleration
