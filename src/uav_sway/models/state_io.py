"""Complete MuJoCo state snapshot and restore helpers."""

from dataclasses import dataclass

import mujoco
import numpy as np


@dataclass
class MujocoStateSnapshot:
    qpos: np.ndarray
    qvel: np.ndarray
    act: np.ndarray
    ctrl: np.ndarray
    time: float
    mocap_pos: np.ndarray
    mocap_quat: np.ndarray
    userdata: np.ndarray
    eq_active: np.ndarray


def _copy_field(data, name: str) -> np.ndarray:
    value = getattr(data, name, None)
    return np.array(value, dtype=float, copy=True) if value is not None else np.empty(0, dtype=float)


def capture_state(model, data) -> MujocoStateSnapshot:
    return MujocoStateSnapshot(
        qpos=_copy_field(data, "qpos"), qvel=_copy_field(data, "qvel"),
        act=_copy_field(data, "act"), ctrl=_copy_field(data, "ctrl"),
        time=float(data.time), mocap_pos=_copy_field(data, "mocap_pos"),
        mocap_quat=_copy_field(data, "mocap_quat"), userdata=_copy_field(data, "userdata"),
        eq_active=_copy_field(data, "eq_active"),
    )


def _restore_field(data, name: str, value: np.ndarray) -> None:
    target = getattr(data, name, None)
    if target is not None and target.size:
        target[...] = value


def restore_state(model, data, snapshot: MujocoStateSnapshot) -> None:
    _restore_field(data, "qpos", snapshot.qpos)
    _restore_field(data, "qvel", snapshot.qvel)
    _restore_field(data, "act", snapshot.act)
    _restore_field(data, "ctrl", snapshot.ctrl)
    data.time = snapshot.time
    _restore_field(data, "mocap_pos", snapshot.mocap_pos)
    _restore_field(data, "mocap_quat", snapshot.mocap_quat)
    _restore_field(data, "userdata", snapshot.userdata)
    _restore_field(data, "eq_active", snapshot.eq_active)
    mujoco.mj_forward(model, data)
