"""Development-informed Traditional recovery controllers for P2-R1R2."""

from __future__ import annotations

from typing import Any

import numpy as np
from scipy.linalg import solve_continuous_are

from .api import SensorPacket, WrenchCommand
from .controller import NativeStackController
from .r1r1_controllers import (
    GRAVITY, INERTIA, MASS_KG, UAV_MINUS_TIP_TRIM,
    NativeGains, NativePID, physical_hover_model,
)


def _angles(rotation: np.ndarray) -> np.ndarray:
    return np.array([
        np.arctan2(rotation[2, 1], rotation[2, 2]),
        np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0)),
        np.arctan2(rotation[1, 0], rotation[0, 0]),
    ])


class R1R2NativePID(NativePID):
    """Independent-axis causal PID with anti-windup and reference feedforward."""

    architecture = "r1r2_independent_axis_task_PID_reference_feedforward_SO3_wrench"

    def __init__(self, parameters: dict[str, Any], method_id: str, outer_rate_hz: int) -> None:
        self.parameters = dict(parameters)
        gains = NativeGains(
            kp=tuple(parameters["kp"]), kd=tuple(parameters["kd"]), ki=tuple(parameters["ki"]),
            attitude_wn=parameters["attitude_wn"], attitude_zeta=parameters["attitude_zeta"],
            integral_limit=parameters["integral_limit"], acceleration_limit=parameters["acceleration_limit"],
            acceleration_slew_per_outer_update=parameters["acceleration_slew_per_s"] / outer_rate_hz,
            swing_angle_gain=parameters["swing_angle_gain"], swing_rate_gain=parameters["swing_rate_gain"],
            tip_correction_kp=parameters["tip_correction_kp"], tip_correction_kd=parameters["tip_correction_kd"],
            tip_correction_limit_m=parameters["tip_correction_limit_m"],
        )
        super().__init__(gains, method_id, outer_dt=1.0 / outer_rate_hz)

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
        proposed = np.clip(self._integral + error * self.outer_dt, -p["integral_limit"], p["integral_limit"])
        acceleration = (
            p["reference_acceleration_gain"] * packet.reference.acceleration_world
            + np.asarray(p["kp"]) * error
            + np.asarray(p["kd"]) * velocity_error
            + np.asarray(p["ki"]) * proposed
        )
        swing = float(np.sum(packet.joint_position)); swing_rate = float(np.sum(packet.joint_velocity))
        acceleration[0] -= p["swing_angle_gain"] * swing + p["swing_rate_gain"] * swing_rate
        norm = float(np.linalg.norm(acceleration))
        if norm <= p["acceleration_limit"]:
            self._integral = proposed
        elif norm > 0.0:
            acceleration *= p["acceleration_limit"] / norm
        return acceleration

    def diagnostics(self) -> dict[str, Any]:
        return {**super().diagnostics(), "r1r2_parameters": self.parameters}


class R1R2ServoLQR(NativeStackController):
    """Physical 12-state LQR with three task integrators and causal feedforward."""

    architecture = "r1r2_15_state_physical_servo_LQI"
    task_space = False

    def __init__(self, parameters: dict[str, Any], method_id: str, outer_rate_hz: int) -> None:
        self.parameters = dict(parameters); self.method_id = method_id
        self.outer_dt = 1.0 / outer_rate_hz; self._sensor_packet: SensorPacket | None = None
        self._integral = np.zeros(3); self._wrench = WrenchCommand(MASS_KG * GRAVITY, np.zeros(3))
        a, b = physical_hover_model(); c = np.zeros((3, 12)); c[:, :3] = np.eye(3)
        a_aug = np.block([[a, np.zeros((12, 3))], [c, np.zeros((3, 3))]])
        b_aug = np.vstack([b, np.zeros((3, 4))])
        q = np.diag(
            [parameters["q_position"]] * 3 + [parameters["q_velocity"]] * 3
            + [parameters["q_attitude"]] * 3 + [parameters["q_rate"]] * 3
            + [parameters["q_integral"]] * 3
        )
        r = np.diag([parameters["r_thrust"], parameters["r_torque"], parameters["r_torque"], parameters["r_torque"]])
        riccati = solve_continuous_are(a_aug, b_aug, q, r)
        self._gain = np.linalg.solve(r, b_aug.T @ riccati)
        ctrb = np.hstack([np.linalg.matrix_power(a_aug, i) @ b_aug for i in range(15)])
        poles = np.linalg.eigvals(a_aug - b_aug @ self._gain)
        self.audit = {
            "state_dimension": 15, "physical_input_dimension": 4,
            "augmentation": "three integral position/task-error states",
            "controllability_rank": int(np.linalg.matrix_rank(ctrb)),
            "stabilizable": bool(np.all(np.real(poles) < 0.0)),
            "closed_loop_max_real_pole": float(np.max(np.real(poles))),
        }

    def reset(self) -> None:
        self._sensor_packet = None; self._integral[:] = 0.0
        self._wrench = WrenchCommand(MASS_KG * GRAVITY, np.zeros(3))

    def _tracking(self, p: SensorPacket) -> tuple[np.ndarray, np.ndarray]:
        target = p.reference.position_world + UAV_MINUS_TIP_TRIM
        return p.uav_position_world - target, p.uav_velocity_world - p.reference.velocity_world

    def update_high_level(self) -> None:
        if self._sensor_packet is None:
            raise RuntimeError("observe must precede update")
        p = self._sensor_packet; position_error, velocity_error = self._tracking(p)
        proposed = np.clip(
            self._integral + position_error * self.outer_dt,
            -self.parameters["integral_limit"], self.parameters["integral_limit"],
        )
        desired_attitude = np.array([
            -self.parameters["reference_acceleration_gain"] * p.reference.acceleration_world[1] / GRAVITY,
            self.parameters["reference_acceleration_gain"] * p.reference.acceleration_world[0] / GRAVITY,
            0.0,
        ])
        state = np.r_[position_error, velocity_error, _angles(p.rotation_world_from_body) - desired_attitude,
                      p.body_angular_velocity, proposed]
        delta = -self._gain @ state
        delta[0] += MASS_KG * self.parameters["reference_acceleration_gain"] * p.reference.acceleration_world[2]
        delta[2] -= self.parameters["swing_angle_gain"] * float(np.sum(p.joint_position))
        delta[2] -= self.parameters["swing_rate_gain"] * float(np.sum(p.joint_velocity))
        margin = self.parameters["constraint_margin"]
        requested = np.r_[MASS_KG * GRAVITY + delta[0], delta[1:]]
        limits = np.array([[0.0, 285.74568], [-25.0, 25.0], [-25.0, 25.0], [-12.0, 12.0]])
        center = limits.mean(axis=1); half = (limits[:, 1] - limits[:, 0]) * .5 * margin
        clipped = np.clip(requested, center - half, center + half)
        if np.array_equal(clipped, requested):
            self._integral = proposed
        self._wrench = WrenchCommand(float(clipped[0]), clipped[1:])

    def physical_command(self) -> WrenchCommand:
        return self._wrench

    def diagnostics(self) -> dict[str, Any]:
        return {"method_id": self.method_id, "architecture": self.architecture,
                "parameters": self.parameters, "linear_model_audit": self.audit,
                "integral": self._integral.copy()}


class R1R2FullLQR(R1R2ServoLQR):
    architecture = "r1r2_12_state_physical_Full_LQR_plus_3_state_LQI_servo"


class R1R2TaskLQR(R1R2ServoLQR):
    architecture = "r1r2_output_weighted_task_LQT_plus_3_state_LQI_servo"
    task_space = True

    def _tracking(self, p: SensorPacket) -> tuple[np.ndarray, np.ndarray]:
        return (p.cutter_tip_position_world - p.reference.position_world,
                p.cutter_tip_velocity_world - p.reference.velocity_world)

