"""Full-state and cutter-task observations for the V3 outer controllers."""

from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np

from uav_sway.task_space.state import CutterTaskSpaceReader, CutterTaskState


def _id(model, object_type, name: str) -> int:
    value = int(mujoco.mj_name2id(model, object_type, name))
    if value < 0:
        raise KeyError(name)
    return value


def _pitch(rotation: np.ndarray) -> float:
    return float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0)))


@dataclass(frozen=True)
class V3Reference:
    """Causal world-frame UAV and cutter references."""

    uav_position_world: np.ndarray
    uav_velocity_world: np.ndarray
    tip_position_world: np.ndarray
    target_time_s: float

    def __post_init__(self) -> None:
        fields = (
            ("uav_position_world", self.uav_position_world),
            ("uav_velocity_world", self.uav_velocity_world),
            ("tip_position_world", self.tip_position_world),
        )
        for name, value in fields:
            array = np.asarray(value, dtype=float).reshape(3)
            if not np.isfinite(array).all():
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, array.copy())
        if not np.isfinite(self.target_time_s):
            raise ValueError("target_time_s must be finite")


@dataclass(frozen=True)
class V3Observation:
    """Causal 20-state error and the measured cutter task state."""

    full_state_error: np.ndarray
    uav_position_world: np.ndarray
    uav_velocity_world: np.ndarray
    task_state: CutterTaskState

    def __post_init__(self) -> None:
        state = np.asarray(self.full_state_error, dtype=float).reshape(20)
        if not np.isfinite(state).all():
            raise ValueError("V3 full state must be finite")
        object.__setattr__(self, "full_state_error", state.copy())
        for name in ("uav_position_world", "uav_velocity_world"):
            value = np.asarray(getattr(self, name), dtype=float).reshape(3)
            if not np.isfinite(value).all():
                raise ValueError(f"{name} must be finite")
            object.__setattr__(self, name, value.copy())


class V3StateReader:
    """Read the frozen R0 20-state schema directly from MuJoCo."""

    STATE_NAMES = (
        "ex", "evx", "ey", "evy", "ez", "evz",
        "roll", "roll_rate", "pitch", "pitch_rate",
        "q1", "q2", "q3", "q4", "q5",
        "qdot1", "qdot2", "qdot3", "qdot4", "qdot5",
    )

    def __init__(self, model) -> None:
        self.quad_body_id = _id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
        self.joint_ids = [_id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}") for i in range(1, 6)]
        self.joint_qposadr = [int(model.jnt_qposadr[j]) for j in self.joint_ids]
        self.joint_qveladr = [int(model.jnt_dofadr[j]) for j in self.joint_ids]
        self.task_reader = CutterTaskSpaceReader(model)

    def read(self, model, data, reference: V3Reference) -> V3Observation:
        position = np.asarray(data.xpos[self.quad_body_id], dtype=float)
        rotation = np.asarray(data.xmat[self.quad_body_id], dtype=float).reshape(3, 3)
        jacp = np.zeros((3, model.nv), dtype=float)
        jacr = np.zeros((3, model.nv), dtype=float)
        mujoco.mj_jacBodyCom(model, data, jacp, jacr, self.quad_body_id)
        velocity = jacp @ np.asarray(data.qvel, dtype=float)
        state = np.zeros(20, dtype=float)
        state[0] = position[0] - reference.uav_position_world[0]
        state[1] = velocity[0] - reference.uav_velocity_world[0]
        state[2] = position[1] - reference.uav_position_world[1]
        state[3] = velocity[1] - reference.uav_velocity_world[1]
        state[4] = position[2] - reference.uav_position_world[2]
        state[5] = velocity[2] - reference.uav_velocity_world[2]
        state[6] = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
        state[7] = float(data.qvel[3])
        state[8] = _pitch(rotation)
        state[9] = float(data.qvel[4])
        state[10:15] = [data.qpos[address] for address in self.joint_qposadr]
        state[15:20] = [data.qvel[address] for address in self.joint_qveladr]
        return V3Observation(state, position, velocity, self.task_reader.read(model, data))


def map_tip_target_to_reference(tip_target_world: np.ndarray, equilibrium_relative_tip: np.ndarray) -> np.ndarray:
    """Map an external cutter-tip target to a UAV position reference."""

    target = np.asarray(tip_target_world, dtype=float).reshape(3)
    relative = np.asarray(equilibrium_relative_tip, dtype=float).reshape(3)
    if not np.isfinite(target).all() or not np.isfinite(relative).all():
        raise ValueError("tip target and equilibrium geometry must be finite")
    return target - relative


def reference_for_target(tip_target_world: np.ndarray, equilibrium_relative_tip: np.ndarray, time_s: float) -> V3Reference:
    uav = map_tip_target_to_reference(tip_target_world, equilibrium_relative_tip)
    return V3Reference(uav, np.zeros(3, dtype=float), np.asarray(tip_target_world, dtype=float), float(time_s))
