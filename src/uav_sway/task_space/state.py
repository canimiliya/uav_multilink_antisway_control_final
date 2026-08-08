"""MuJoCo-backed cutter task-space state extraction.

The cutter tip velocity is deliberately obtained from the site Jacobian.  A
finite difference of logged tip positions is not part of the formal state
contract.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np


def _id(model, object_type, name: str) -> int:
    value = int(mujoco.mj_name2id(model, object_type, name))
    if value < 0:
        raise KeyError(name)
    return value


def _unit(vector: np.ndarray, name: str) -> np.ndarray:
    value = np.asarray(vector, dtype=float).reshape(3)
    norm = float(np.linalg.norm(value))
    if not np.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError(f"{name} must have a finite non-zero norm")
    return value / norm


@dataclass(frozen=True)
class CutterTaskState:
    """World-frame state exposed to task-space evaluators and controllers."""

    tip_position_world: np.ndarray
    tip_velocity_world: np.ndarray
    cutter_angular_velocity_world: np.ndarray
    cutter_axis_world: np.ndarray
    cutter_rotation_world: np.ndarray

    def __post_init__(self) -> None:
        position = np.asarray(self.tip_position_world, dtype=float).reshape(3)
        velocity = np.asarray(self.tip_velocity_world, dtype=float).reshape(3)
        angular_velocity = np.asarray(self.cutter_angular_velocity_world, dtype=float).reshape(3)
        axis = _unit(self.cutter_axis_world, "cutter_axis_world")
        rotation = np.asarray(self.cutter_rotation_world, dtype=float).reshape(3, 3)
        if not all(np.isfinite(value).all() for value in (position, velocity, angular_velocity, axis, rotation)):
            raise ValueError("CutterTaskState must be finite")
        object.__setattr__(self, "tip_position_world", position.copy())
        object.__setattr__(self, "tip_velocity_world", velocity.copy())
        object.__setattr__(self, "cutter_angular_velocity_world", angular_velocity.copy())
        object.__setattr__(self, "cutter_axis_world", axis.copy())
        object.__setattr__(self, "cutter_rotation_world", rotation.copy())


class CutterTaskSpaceReader:
    """Read ``cutter_tip`` position/velocity and ``cutter`` orientation."""

    cutter_axis_local = np.asarray([1.0, 0.0, 0.0], dtype=float)

    def __init__(self, model) -> None:
        self.tip_site_id = _id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip")
        self.cutter_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "cutter")

    def read(self, model, data) -> CutterTaskState:
        jacp = np.zeros((3, model.nv), dtype=float)
        jacr = np.zeros((3, model.nv), dtype=float)
        mujoco.mj_jacSite(model, data, jacp, jacr, self.tip_site_id)
        rotation = np.asarray(data.xmat[self.cutter_body_id], dtype=float).reshape(3, 3)
        axis = rotation @ self.cutter_axis_local
        return CutterTaskState(
            tip_position_world=np.asarray(data.site_xpos[self.tip_site_id], dtype=float),
            tip_velocity_world=jacp @ np.asarray(data.qvel, dtype=float),
            cutter_angular_velocity_world=jacr @ np.asarray(data.qvel, dtype=float),
            cutter_axis_world=axis,
            cutter_rotation_world=rotation,
        )

    def position_jacobian(self, model, data) -> np.ndarray:
        jacp = np.zeros((3, model.nv), dtype=float)
        jacr = np.zeros((3, model.nv), dtype=float)
        mujoco.mj_jacSite(model, data, jacp, jacr, self.tip_site_id)
        return jacp
