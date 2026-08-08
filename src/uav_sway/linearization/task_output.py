"""Task-output map and Jacobian for the S6T1 task-space LQR."""

from __future__ import annotations

import mujoco
import numpy as np

from uav_sway.control.base import ReferenceState
from uav_sway.models.state_io import MujocoStateSnapshot, restore_state
from uav_sway.task_space.reference import EquilibriumTaskPose
from uav_sway.task_space.state import CutterTaskSpaceReader

from .reduced_state import ReducedStateLayout


def signed_cutter_planar_angle(cutter_axis_world: np.ndarray) -> float:
    """Return the signed y-axis planar angle of the cutter x-axis."""

    direction = np.asarray(cutter_axis_world, dtype=float).reshape(3)
    return float(np.arctan2(-direction[2], direction[0]))


def _id(model, object_type, name: str) -> int:
    value = int(mujoco.mj_name2id(model, object_type, name))
    if value < 0:
        raise KeyError(name)
    return value


class TaskOutputMap:
    """Evaluate the four local task outputs from the S4 reduced state."""

    def __init__(self, model, equilibrium_snapshot: MujocoStateSnapshot,
                 equilibrium_pose: EquilibriumTaskPose):
        self.model = model
        self.snapshot = equilibrium_snapshot
        self.pose = equilibrium_pose
        self.layout = ReducedStateLayout(model)
        self.reader = CutterTaskSpaceReader(model)
        self.tip_site_id = _id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip")
        self.cutter_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "cutter")
        self.data = mujoco.MjData(model)

    def from_mujoco(self, data, reference: ReferenceState) -> np.ndarray:
        tip_position, tip_velocity, axis, angular_velocity = self.kinematics_from_mujoco(data)
        return np.asarray([
            tip_position[0] - float(reference.x_ref) - float(self.pose.tip_relative_position_m[0]),
            tip_velocity[0] - float(reference.vx_ref),
            signed_cutter_planar_angle(axis),
            angular_velocity[1],
        ], dtype=float)

    def kinematics_from_mujoco(self, data) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        jacp = np.zeros((3, self.model.nv), dtype=float)
        jacr = np.zeros((3, self.model.nv), dtype=float)
        mujoco.mj_jacSite(self.model, data, jacp, jacr, self.tip_site_id)
        body_jacp = np.zeros((3, self.model.nv), dtype=float)
        body_jacr = np.zeros((3, self.model.nv), dtype=float)
        mujoco.mj_jacBody(self.model, data, body_jacp, body_jacr, self.cutter_body_id)
        rotation = np.asarray(data.xmat[self.cutter_body_id], dtype=float).reshape(3, 3)
        axis = rotation @ np.asarray([1.0, 0.0, 0.0])
        tip_position = np.asarray(data.site_xpos[self.tip_site_id], dtype=float)
        tip_velocity = jacp @ np.asarray(data.qvel, dtype=float)
        angular_velocity = body_jacr @ np.asarray(data.qvel, dtype=float)
        return tip_position, tip_velocity, axis, angular_velocity

    def from_reduced_state(self, state: np.ndarray, reference: ReferenceState | None = None) -> np.ndarray:
        reference = reference or ReferenceState(0.0, 0.0, 0.0, 0.0, 3.2, 0.0)
        restore_state(self.model, self.data, self.snapshot)
        self.layout.inject(self.model, self.data, self.snapshot, np.asarray(state, dtype=float), reference)
        return self.from_mujoco(self.data, reference)


def identify_task_output_jacobian(task_map: TaskOutputMap, state_epsilon: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Use mirrored central differences of the nonlinear MuJoCo output map."""

    epsilon = np.asarray(state_epsilon, dtype=float).reshape(16)
    if np.any(epsilon <= 0.0) or not np.isfinite(epsilon).all():
        raise ValueError("task output state epsilon must be positive and finite")
    zero = np.zeros(16, dtype=float)
    reference = ReferenceState(0.0, 0.0, 0.0, 0.0, 3.2, 0.0)
    y0 = task_map.from_reduced_state(zero, reference)
    c = np.empty((4, 16), dtype=float)
    for i, eps in enumerate(epsilon):
        plus = zero.copy(); plus[i] = eps
        minus = zero.copy(); minus[i] = -eps
        c[:, i] = (task_map.from_reduced_state(plus, reference) - task_map.from_reduced_state(minus, reference)) / (2.0 * eps)
    if not np.isfinite(y0).all() or not np.isfinite(c).all():
        raise FloatingPointError("task-output identification produced non-finite values")
    return c, y0


def validate_task_output_local(task_map: TaskOutputMap, c_task: np.ndarray,
                               state_epsilon: np.ndarray, multipliers=(2, 5, 10),
                               sample_count: int = 64, seed: int = 20260808) -> dict:
    """Compare mirrored nonlinear outputs with ``C_task @ delta``."""

    rng = np.random.default_rng(seed)
    c_task = np.asarray(c_task, dtype=float).reshape(4, 16)
    epsilon = np.asarray(state_epsilon, dtype=float).reshape(16)
    results = {}
    for multiplier in multipliers:
        errors = []
        scale = epsilon * multiplier
        for _ in range(sample_count):
            direction = rng.uniform(-1.0, 1.0, 16) * scale
            for sign in (1.0, -1.0):
                actual = task_map.from_reduced_state(sign * direction)
                predicted = c_task @ (sign * direction)
                errors.append(actual - predicted)
        array = np.asarray(errors, dtype=float)
        normalized = array / np.asarray([max(scale[0], 1e-12), max(scale[1], 1e-12), max(scale[4], 1e-12), max(scale[5], 1e-12)])
        results[f"{multiplier}x_epsilon"] = {
            "multiplier": multiplier,
            "mirror_samples": True,
            "sample_count": int(len(array)),
            "absolute_rmse": np.sqrt(np.mean(array ** 2, axis=0)).tolist(),
            "maximum_absolute_error": np.max(np.abs(array), axis=0).tolist(),
            "normalized_rmse": np.sqrt(np.mean(normalized ** 2, axis=0)).tolist(),
            "finite": bool(np.isfinite(array).all()),
        }
    normalized = np.asarray([v["normalized_rmse"] for v in results.values()], dtype=float)
    return {
        "seed": seed,
        "state_epsilon": epsilon.tolist(),
        "multipliers": list(multipliers),
        "by_multiplier": results,
        "finite": bool(all(v["finite"] for v in results.values())),
        "pass": bool(all(v["finite"] for v in results.values()) and float(np.max(normalized)) < 0.25),
        "validation_reference": "C_task @ delta_x",
    }
