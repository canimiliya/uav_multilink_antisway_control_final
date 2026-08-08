"""Shared 3-D position stabilization and Udaan geometric attitude wrapper."""

from __future__ import annotations

import numpy as np
from udaan.control.quadrotor import GeometricAttitudeController
from udaan.manif import SO3, TSO3

from .base import ControlState, ReferenceState
from uav_sway.task_space.state import CutterTaskState
from uav_sway.task_space.v2_reference import Shared3DControlLimits


class GeometricInnerLoop:
    def __init__(self, total_mass: float, inertia_diagonal: np.ndarray,
                 attitude_natural_frequency: float = 4.0,
                 attitude_damping_ratio: float = 0.9,
                 ay_kp: float = 1.5, ay_kd: float = 2.0,
                 az_kp: float = 4.0, az_kd: float = 3.5,
                 shared_limits: Shared3DControlLimits | None = None):
        self.total_mass = float(total_mass)
        self.inertia_diagonal = np.asarray(inertia_diagonal, dtype=float).copy()
        self.ay_kp, self.ay_kd = float(ay_kp), float(ay_kd)
        self.az_kp, self.az_kd = float(az_kp), float(az_kd)
        self.shared_limits = shared_limits
        self._previous_shared_ay = 0.0
        self._previous_shared_az = 0.0
        wn = float(attitude_natural_frequency)
        zeta = float(attitude_damping_ratio)
        self.k_r = self.inertia_diagonal * wn**2
        self.k_omega = 2.0 * zeta * self.inertia_diagonal * wn
        self.controller = GeometricAttitudeController(inertia=np.diag(self.inertia_diagonal))
        # The Udaan controller is reused, but its small-airframe defaults are
        # replaced by gains derived from the frozen M400 inertia.
        self.controller._gains.kp = self.k_r.copy()
        self.controller._gains.kd = self.k_omega.copy()

    def reset(self) -> None:
        self._previous_shared_ay = 0.0
        self._previous_shared_az = 0.0

    def shared_yz_command(self, state: ControlState, reference: ReferenceState,
                          task_state: CutterTaskState | None = None,
                          tip_target_world: np.ndarray | None = None) -> tuple[float, float]:
        """Compute the shared y/z command once per outer update.

        The formal V2-R1R1 path supplies the measured cutter task state and the
        external cutter target.  The UAV-state fallback is retained for older
        callers outside the frozen V2 runner.
        """
        if (task_state is None) != (tip_target_world is None):
            raise ValueError("task_state and tip_target_world must be supplied together")
        if task_state is not None and tip_target_world is not None:
            target = np.asarray(tip_target_world, dtype=float).reshape(3)
            if not np.isfinite(target).all():
                raise ValueError("tip target must be finite")
            error = np.asarray(task_state.tip_position_world, dtype=float) - target
            velocity = np.asarray(task_state.tip_velocity_world, dtype=float)
            desired_ay = -self.ay_kp * error[1] - self.ay_kd * velocity[1]
            desired_az = -self.az_kp * error[2] - self.az_kd * velocity[2]
        else:
            desired_ay = -self.ay_kp * (state.position[1] - reference.y_ref) - self.ay_kd * state.velocity[1]
            desired_az = -self.az_kp * (state.position[2] - reference.z_ref) - self.az_kd * state.velocity[2]
        if self.shared_limits is not None:
            desired_ay, desired_az = self.shared_limits.apply(
                desired_ay, desired_az, self._previous_shared_ay, self._previous_shared_az
            )
            self._previous_shared_ay = desired_ay
            self._previous_shared_az = desired_az
        return float(desired_ay), float(desired_az)

    def desired_force(self, state: ControlState, reference: ReferenceState, ax_limited: float,
                      shared_yz: tuple[float, float] | None = None) -> np.ndarray:
        if shared_yz is None:
            shared_yz = self.shared_yz_command(state, reference)
        desired_ay, desired_az = shared_yz
        acceleration = np.array([float(ax_limited), desired_ay, desired_az])
        return self.total_mass * (acceleration + np.array([0.0, 0.0, 9.81]))

    def compute(self, state: ControlState, reference: ReferenceState, ax_limited: float,
                shared_yz: tuple[float, float] | None = None) -> dict[str, np.ndarray | float]:
        desired_force = self.desired_force(state, reference, ax_limited, shared_yz)
        thrust, torque = self.controller.compute(
            float(0.0),
            (SO3(state.rotation), TSO3(state.body_angular_velocity)),
            desired_force,
        )
        return {
            "desired_force_world": desired_force,
            "thrust_raw_N": float(thrust),
            "torque_raw_Nm": np.asarray(torque, dtype=float).copy(),
        }
