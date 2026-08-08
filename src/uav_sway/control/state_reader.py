"""Name-based extraction of the complete S3 controller state."""

from __future__ import annotations

import mujoco
import numpy as np

from .base import ControlState


def _id(model, object_type, name: str) -> int:
    value = int(mujoco.mj_name2id(model, object_type, name))
    if value < 0:
        raise KeyError(name)
    return value


class StateReader:
    def __init__(self, model, n_links: int, equilibrium_relative_x: float = 0.0):
        self.quad_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
        self.tip_site_id = _id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip")
        self.joint_ids = [_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}") for i in range(1, n_links + 1)]
        self.qpos_addresses = [int(model.jnt_qposadr[j]) for j in self.joint_ids]
        self.qvel_addresses = [int(model.jnt_dofadr[j]) for j in self.joint_ids]
        self.equilibrium_relative_x = float(equilibrium_relative_x)

    def read(self, model, data) -> ControlState:
        quad_x = float(data.xpos[self.quad_body_id, 0])
        tip_relative_x = float(data.site_xpos[self.tip_site_id, 0] - quad_x)
        return ControlState(
            position=np.asarray(data.xpos[self.quad_body_id], dtype=float).copy(),
            velocity=self._body_velocity(model, data, self.quad_body_id),
            rotation=np.asarray(data.xmat[self.quad_body_id], dtype=float).reshape(3, 3).copy(),
            body_angular_velocity=np.asarray(data.qvel[3:6], dtype=float).copy(),
            joint_angles=np.asarray([data.qpos[a] for a in self.qpos_addresses], dtype=float),
            joint_velocities=np.asarray([data.qvel[a] for a in self.qvel_addresses], dtype=float),
            tip_displacement=tip_relative_x - self.equilibrium_relative_x,
        )

    @staticmethod
    def _body_velocity(model, data, body_id: int) -> np.ndarray:
        jacp = np.zeros((3, model.nv), dtype=float)
        jacr = np.zeros((3, model.nv), dtype=float)
        mujoco.mj_jacBodyCom(model, data, jacp, jacr, body_id)
        return jacp @ data.qvel
