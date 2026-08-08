"""Shared 3-D position stabilization and Udaan geometric attitude wrapper."""

from __future__ import annotations

import numpy as np
from udaan.control.quadrotor import GeometricAttitudeController
from udaan.manif import SO3, TSO3

from .base import ControlState, ReferenceState


class GeometricInnerLoop:
    def __init__(self, total_mass: float, inertia_diagonal: np.ndarray,
                 attitude_natural_frequency: float = 4.0,
                 attitude_damping_ratio: float = 0.9,
                 ay_kp: float = 1.5, ay_kd: float = 2.0,
                 az_kp: float = 4.0, az_kd: float = 3.5):
        self.total_mass = float(total_mass)
        self.inertia_diagonal = np.asarray(inertia_diagonal, dtype=float).copy()
        self.ay_kp, self.ay_kd = float(ay_kp), float(ay_kd)
        self.az_kp, self.az_kd = float(az_kp), float(az_kd)
        wn = float(attitude_natural_frequency)
        zeta = float(attitude_damping_ratio)
        self.k_r = self.inertia_diagonal * wn**2
        self.k_omega = 2.0 * zeta * self.inertia_diagonal * wn
        self.controller = GeometricAttitudeController(inertia=np.diag(self.inertia_diagonal))
        # The Udaan controller is reused, but its small-airframe defaults are
        # replaced by gains derived from the frozen M400 inertia.
        self.controller._gains.kp = self.k_r.copy()
        self.controller._gains.kd = self.k_omega.copy()

    def desired_force(self, state: ControlState, reference: ReferenceState, ax_limited: float) -> np.ndarray:
        acceleration = np.array([
            float(ax_limited),
            -self.ay_kp * (state.position[1] - reference.y_ref) - self.ay_kd * state.velocity[1],
            -self.az_kp * (state.position[2] - reference.z_ref) - self.az_kd * state.velocity[2],
        ])
        return self.total_mass * (acceleration + np.array([0.0, 0.0, 9.81]))

    def compute(self, state: ControlState, reference: ReferenceState, ax_limited: float) -> dict[str, np.ndarray | float]:
        desired_force = self.desired_force(state, reference, ax_limited)
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
