"""Causal SensorPacket construction from the frozen MuJoCo state."""

from __future__ import annotations

import mujoco
import numpy as np

from uav_sway.task_space.state import CutterTaskSpaceReader

from .api import ReferenceSample, SensorPacket, WrenchCommand


class NativeSensorReader:
    def __init__(self, model) -> None:
        self.quad_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
        self.joint_ids = tuple(
            int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{index}"))
            for index in range(1, 6)
        )
        if self.quad_id < 0 or any(index < 0 for index in self.joint_ids):
            raise ValueError("frozen state schema is absent")
        self.qpos_addresses = tuple(int(model.jnt_qposadr[index]) for index in self.joint_ids)
        self.qvel_addresses = tuple(int(model.jnt_dofadr[index]) for index in self.joint_ids)
        self.task_reader = CutterTaskSpaceReader(model)

    def read(
        self, model, data, reference: ReferenceSample, previous_command: WrenchCommand,
        tick: int, physics_dt_s: float,
    ) -> SensorPacket:
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacBodyCom(model, data, jacp, jacr, self.quad_id)
        task = self.task_reader.read(model, data)
        return SensorPacket(
            time_s=float(tick * physics_dt_s), tick=int(tick),
            uav_position_world=np.asarray(data.xpos[self.quad_id]),
            uav_velocity_world=jacp @ np.asarray(data.qvel),
            rotation_world_from_body=np.asarray(data.xmat[self.quad_id]).reshape(3, 3),
            body_angular_velocity=jacr @ np.asarray(data.qvel),
            joint_position=np.asarray([data.qpos[address] for address in self.qpos_addresses]),
            joint_velocity=np.asarray([data.qvel[address] for address in self.qvel_addresses]),
            cutter_tip_position_world=task.tip_position_world,
            cutter_tip_velocity_world=task.tip_velocity_world,
            reference=reference, previous_applied_command=previous_command,
        )
