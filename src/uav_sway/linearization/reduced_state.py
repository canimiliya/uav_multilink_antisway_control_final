"""Name-based 16-dimensional state extraction and injection."""

from __future__ import annotations

import mujoco
import numpy as np

from uav_sway.models.state_io import MujocoStateSnapshot


STATE_NAMES = [
    "position_error_x", "velocity_error_x", "altitude_error", "vertical_velocity",
    "pitch", "body_pitch_rate", *[f"joint_{i}_angle" for i in range(1, 6)],
    *[f"joint_{i}_velocity" for i in range(1, 6)],
]


def _id(model, object_type, name: str) -> int:
    value = int(mujoco.mj_name2id(model, object_type, name))
    if value < 0:
        raise KeyError(name)
    return value


def _pitch(rotation: np.ndarray) -> float:
    return float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0)))


def _local_y_quaternion(angle: float) -> np.ndarray:
    return np.asarray([np.cos(angle / 2.0), 0.0, np.sin(angle / 2.0), 0.0], dtype=float)


class ReducedStateLayout:
    def __init__(self, model):
        self.quad_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
        self.free_joint_id = _id(model, mujoco.mjtObj.mjOBJ_JOINT, "quadrotor_free")
        self.free_qposadr = int(model.jnt_qposadr[self.free_joint_id])
        self.free_qveladr = int(model.jnt_dofadr[self.free_joint_id])
        self.joint_ids = [_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}") for i in range(1, 6)]
        self.joint_qposadr = [int(model.jnt_qposadr[j]) for j in self.joint_ids]
        self.joint_qveladr = [int(model.jnt_dofadr[j]) for j in self.joint_ids]
        if model.nq != 12 or model.nv != 11:
            raise ValueError("S4 nominal model must have nq=12 and nv=11")

    def extract(self, model, data, reference) -> np.ndarray:
        del model
        position = np.asarray(data.xpos[self.quad_body_id], dtype=float)
        rotation = np.asarray(data.xmat[self.quad_body_id], dtype=float).reshape(3, 3)
        qv = data.qvel
        result = np.zeros(16, dtype=float)
        result[0] = position[0] - float(reference.x_ref)
        result[1] = qv[self.free_qveladr] - float(reference.vx_ref)
        result[2] = position[2] - float(reference.z_ref)
        result[3] = qv[self.free_qveladr + 2]
        result[4] = _pitch(rotation)
        result[5] = qv[self.free_qveladr + 4]
        result[6:11] = [data.qpos[a] for a in self.joint_qposadr]
        result[11:16] = [data.qvel[a] for a in self.joint_qveladr]
        return result

    def inject(self, model, data, equilibrium_snapshot: MujocoStateSnapshot,
               state: np.ndarray, reference) -> None:
        state = np.asarray(state, dtype=float)
        if state.shape != (16,) or not np.isfinite(state).all():
            raise ValueError("reduced state must be a finite 16-vector")
        data.qpos[:] = equilibrium_snapshot.qpos
        data.qvel[:] = equilibrium_snapshot.qvel
        data.ctrl[:] = equilibrium_snapshot.ctrl
        data.eq_active[:] = equilibrium_snapshot.eq_active
        base = self.free_qposadr
        data.qpos[base + 0] = float(reference.x_ref) + state[0]
        data.qpos[base + 1] = equilibrium_snapshot.qpos[base + 1] + float(reference.y_ref)
        data.qpos[base + 2] = float(reference.z_ref) + state[2]
        q_local = _local_y_quaternion(float(state[4]))
        q_out = np.zeros(4, dtype=float)
        mujoco.mju_mulQuat(q_out, equilibrium_snapshot.qpos[base + 3:base + 7], q_local)
        data.qpos[base + 3:base + 7] = q_out
        vbase = self.free_qveladr
        data.qvel[vbase + 0] = float(reference.vx_ref) + state[1]
        data.qvel[vbase + 1] = equilibrium_snapshot.qvel[vbase + 1]
        data.qvel[vbase + 2] = state[3]
        data.qvel[vbase + 3] = equilibrium_snapshot.qvel[vbase + 3]
        data.qvel[vbase + 4] = state[5]
        data.qvel[vbase + 5] = equilibrium_snapshot.qvel[vbase + 5]
        for i, address in enumerate(self.joint_qposadr):
            data.qpos[address] = state[6 + i]
        for i, address in enumerate(self.joint_qveladr):
            data.qvel[address] = state[11 + i]
        mujoco.mj_forward(model, data)


def extract_reduced_state(model, data, reference) -> np.ndarray:
    return ReducedStateLayout(model).extract(model, data, reference)


def inject_reduced_state(model, data, equilibrium_snapshot, state, reference) -> None:
    ReducedStateLayout(model).inject(model, data, equilibrium_snapshot, state, reference)
