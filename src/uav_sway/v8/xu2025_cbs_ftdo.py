"""Xu et al. (2025) composite backstepping plus finite-time DO adaptation."""

from __future__ import annotations

from dataclasses import dataclass
import time

import numpy as np

from uav_sway.v3.controllers import V3ControllerDiagnostics, _V3ControllerBase
from uav_sway.v3.observation import V3Observation, V3Reference


def _unit(value: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    value = np.asarray(value, dtype=float).reshape(3)
    norm = float(np.linalg.norm(value))
    if norm <= 1.0e-10:
        return np.asarray(fallback, dtype=float).reshape(3).copy()
    return value / norm


def _smooth_sign(value: np.ndarray, boundary: float) -> np.ndarray:
    value = np.asarray(value, dtype=float)
    return value / np.maximum(np.abs(value), float(boundary))


def _signed_sqrt(value: np.ndarray) -> np.ndarray:
    value = np.asarray(value, dtype=float)
    return np.sign(value) * np.sqrt(np.abs(value))


def _rotation_from_roll_pitch(roll: float, pitch: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    return ry @ rx


@dataclass(frozen=True)
class Xu2025Diagnostics(V3ControllerDiagnostics):
    modal_coordinates: np.ndarray
    reconstructed_joint_angles: np.ndarray
    load_direction: np.ndarray
    load_angular_velocity: np.ndarray
    desired_load_direction: np.ndarray
    desired_load_angular_velocity: np.ndarray
    force_disturbance_hat: np.ndarray
    angular_disturbance_hat: np.ndarray
    parallel_specific_force: np.ndarray
    perpendicular_specific_force: np.ndarray
    solve_time_ms: float
    limiter_mismatch: float


class Xu2025CBSFTDO(_V3ControllerBase):
    """Fair 20 Hz realization of source Eqs. (20)-(41).

    The source point-load cable state is obtained from the two plant-only
    modes frozen by V8-R0.  The four cascaded loops, both finite-time
    disturbance observers, and both command filters are retained.  The final
    thrust vector is mass-normalized to the benchmark's common world-frame
    acceleration interface; thrust/torque actuation remains in the common
    geometric inner loop.
    """

    GRAVITY = np.asarray([0.0, 0.0, -9.81])

    def __init__(self, parameters: dict) -> None:
        super().__init__()
        self.parameters = dict(parameters)
        self.modal_basis = np.asarray(parameters["modal_basis"], dtype=float).reshape(5, 2)
        self.modal_projection = np.asarray(parameters["modal_projection"], dtype=float).reshape(2, 5)
        if not np.isfinite(self.modal_basis).all() or not np.isfinite(self.modal_projection).all():
            raise ValueError("modal reduction matrices must be finite")
        self.axis_scale = np.asarray(parameters["axis_scale"], dtype=float).reshape(3)
        if np.any(self.axis_scale <= 0.0) or not np.isfinite(self.axis_scale).all():
            raise ValueError("axis_scale must be positive and finite")
        for name in (
            "position_gain", "velocity_gain", "direction_gain", "angular_gain",
            "direction_filter_gain", "angular_filter_gain", "force_observer_k1",
            "force_observer_k2", "angular_observer_k1", "angular_observer_k2",
            "observer_boundary", "observer_clip", "effective_length_m",
            "observer_compensation_scale", "angular_coupling_scale",
        ):
            value = float(parameters[name])
            if not np.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and nonnegative")
        self.reset()

    def reset(self) -> None:
        super().reset()
        self.filtered_direction = np.asarray([0.0, 0.0, -1.0])
        self.filtered_angular_velocity = np.zeros(3)
        self.previous_load_direction: np.ndarray | None = None
        self.velocity_hat = np.zeros(3)
        self.force_disturbance_hat = np.zeros(3)
        self.angular_velocity_hat = np.zeros(3)
        self.angular_disturbance_hat = np.zeros(3)
        self.observer_initialized = False
        self.previous_specific_acceleration = np.zeros(3)
        self.previous_known_angular_acceleration = np.zeros(3)
        z3 = np.zeros(3)
        self.diagnostics = Xu2025Diagnostics(
            z3, z3, z3, np.zeros(3, dtype=bool), np.zeros(3, dtype=bool), 0.0, z3,
            np.zeros(2), np.zeros(5), z3, z3, z3, z3, z3, z3, z3, z3, 0.0, 0.0,
        )

    def _modal_load_direction(self, state: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        joint_angles = np.asarray(state[10:15], dtype=float)
        modal_coordinates = self.modal_projection @ joint_angles
        reconstructed = self.modal_basis @ modal_coordinates
        cumulative = np.cumsum(reconstructed)
        relative_body = np.asarray([0.0, 0.0, -0.24])
        for angle in cumulative:
            relative_body += np.asarray([-0.5 * np.sin(angle), 0.0, -0.5 * np.cos(angle)])
        angle = float(cumulative[-1])
        # Paper q points to the payload center, not the offset cutting tip.
        relative_body += np.asarray([-0.07 * np.sin(angle), 0.0, -0.07 * np.cos(angle)])
        rotation = _rotation_from_roll_pitch(float(state[6]), float(state[8]))
        return _unit(rotation @ relative_body, np.asarray([0.0, 0.0, -1.0])), modal_coordinates, reconstructed

    def _update_observers(
        self,
        load_velocity: np.ndarray,
        load_angular_velocity: np.ndarray,
        dt: float,
    ) -> None:
        if not self.observer_initialized:
            self.velocity_hat = load_velocity.copy()
            self.angular_velocity_hat = load_angular_velocity.copy()
            self.observer_initialized = True
        boundary = float(self.parameters["observer_boundary"])
        clip = float(self.parameters["observer_clip"])

        velocity_error = self.velocity_hat - load_velocity
        self.velocity_hat += dt * (
            self.previous_specific_acceleration
            + self.force_disturbance_hat
            - float(self.parameters["force_observer_k1"]) * _signed_sqrt(velocity_error)
        )
        self.force_disturbance_hat = np.clip(
            self.force_disturbance_hat
            - dt * float(self.parameters["force_observer_k2"]) * _smooth_sign(velocity_error, boundary),
            -clip,
            clip,
        )

        angular_error = self.angular_velocity_hat - load_angular_velocity
        self.angular_velocity_hat += dt * (
            self.previous_known_angular_acceleration
            + self.angular_disturbance_hat
            - float(self.parameters["angular_observer_k1"]) * _signed_sqrt(angular_error)
        )
        self.angular_disturbance_hat = np.clip(
            self.angular_disturbance_hat
            - dt * float(self.parameters["angular_observer_k2"]) * _smooth_sign(angular_error, boundary),
            -clip,
            clip,
        )

    def command(self, observation: V3Observation, reference: V3Reference, dt: float = 0.05) -> np.ndarray:
        dt = float(dt)
        if abs(dt - 0.05) > 1.0e-12:
            raise ValueError("XU2025 adaptation requires the frozen 20 Hz outer rate")
        started = time.perf_counter_ns()
        load_direction, modal_coordinates, reconstructed = self._modal_load_direction(observation.full_state_error)
        if self.previous_load_direction is None:
            load_angular_velocity = np.zeros(3)
        else:
            direction_rate = (load_direction - self.previous_load_direction) / dt
            direction_rate -= load_direction * float(load_direction @ direction_rate)
            load_angular_velocity = np.cross(load_direction, direction_rate)
        self.previous_load_direction = load_direction.copy()

        load_position = np.asarray(observation.task_state.tip_position_world, dtype=float)
        load_velocity = np.asarray(observation.task_state.tip_velocity_world, dtype=float)
        self._update_observers(load_velocity, load_angular_velocity, dt)

        kp = float(self.parameters["position_gain"]) * self.axis_scale
        kv = float(self.parameters["velocity_gain"]) * self.axis_scale
        position_error = load_position - reference.tip_position_world
        virtual_velocity = reference.uav_velocity_world - kp * position_error
        velocity_error = load_velocity - virtual_velocity
        virtual_acceleration = -kp * (load_velocity - reference.uav_velocity_world)
        disturbance_scale = float(self.parameters["observer_compensation_scale"])
        target_specific_force = (
            -kv * velocity_error
            + virtual_acceleration
            - self.GRAVITY
            - disturbance_scale * self.force_disturbance_hat
        )
        target_direction = _unit(-target_specific_force, np.asarray([0.0, 0.0, -1.0]))

        kf = float(self.parameters["direction_filter_gain"])
        desired_omega = -kf * np.cross(target_direction, self.filtered_direction)
        self.filtered_direction = _unit(
            self.filtered_direction + dt * np.cross(desired_omega, self.filtered_direction),
            target_direction,
        )
        desired_omega = -kf * np.cross(target_direction, self.filtered_direction)
        direction_error = np.cross(self.filtered_direction, load_direction)
        angular_target = desired_omega - float(self.parameters["direction_gain"]) * direction_error
        filtered_angular_dot = -float(self.parameters["angular_filter_gain"]) * (
            self.filtered_angular_velocity - angular_target
        )
        self.filtered_angular_velocity += dt * filtered_angular_dot
        angular_error = load_angular_velocity - self.filtered_angular_velocity
        angular_nominal = -float(self.parameters["angular_gain"]) * angular_error
        angular_drive = (
            angular_nominal
            + filtered_angular_dot
            - disturbance_scale * self.angular_disturbance_hat
        )
        length = float(self.parameters["effective_length_m"])
        angular_scale = float(self.parameters["angular_coupling_scale"])
        skew_force = -length * angular_scale * angular_drive
        perpendicular_force = -np.cross(load_direction, skew_force)
        parallel_force = (
            load_direction * float(load_direction @ target_specific_force)
            + length * float(load_angular_velocity @ load_angular_velocity) * load_direction
        )
        total_specific_thrust = parallel_force + perpendicular_force
        raw = total_specific_thrust + self.GRAVITY
        amplitude = np.clip(raw, -2.0, 2.0)
        final = self.limiter.limit(raw).as_array()

        self.previous_specific_acceleration = final.copy()
        self.previous_known_angular_acceleration = (
            -np.cross(load_direction, perpendicular_force) / max(length, 1.0e-6)
        )
        elapsed_ms = (time.perf_counter_ns() - started) / 1.0e6
        self.diagnostics = Xu2025Diagnostics(
            raw.copy(), amplitude.copy(), final.copy(), np.abs(raw) > 2.0 + 1.0e-12,
            np.abs(final - amplitude) > 1.0e-12, float(np.linalg.norm(observation.full_state_error)),
            np.zeros(3), modal_coordinates.copy(), reconstructed.copy(), load_direction.copy(),
            load_angular_velocity.copy(), self.filtered_direction.copy(),
            self.filtered_angular_velocity.copy(), self.force_disturbance_hat.copy(),
            self.angular_disturbance_hat.copy(), parallel_force.copy(), perpendicular_force.copy(),
            elapsed_ms, float(np.max(np.abs(final - raw))),
        )
        return final.copy()
