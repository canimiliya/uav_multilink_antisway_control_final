"""Canonical direct-wrench bridge to the unchanged MuJoCo plant."""

from __future__ import annotations

import mujoco
import numpy as np

from .api import AppliedPhysicalCommand, WrenchCommand


class CanonicalWrenchActuator:
    ACTUATOR_NAMES = ("thrust_motor", "mx_motor", "my_motor", "mz_motor")

    def __init__(self, model) -> None:
        self.ids = tuple(
            int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name))
            for name in self.ACTUATOR_NAMES
        )
        if any(index < 0 for index in self.ids):
            raise ValueError("canonical direct-wrench actuators are absent")
        self.limits = np.asarray(model.actuator_ctrlrange[list(self.ids)], dtype=float).copy()
        expected = np.asarray([[0.0, 285.74568], [-25.0, 25.0], [-25.0, 25.0], [-12.0, 12.0]])
        if not np.array_equal(self.limits, expected):
            raise ValueError(f"frozen actuator limits changed: {self.limits!r}")
        # MuJoCo dyntype==0 is direct/no actuator dynamics.
        if np.any(np.asarray(model.actuator_dyntype[list(self.ids)], dtype=int) != 0):
            raise ValueError("v1 only supports the audited direct actuator dynamics")

    def clip(self, command: WrenchCommand) -> WrenchCommand:
        values = np.clip(command.as_array(), self.limits[:, 0], self.limits[:, 1])
        return WrenchCommand(float(values[0]), values[1:])

    def apply(self, data, command: WrenchCommand, tick: int, physics_dt_s: float) -> AppliedPhysicalCommand:
        clipped = self.clip(command)
        data.ctrl[:] = 0.0
        data.ctrl[list(self.ids)] = clipped.as_array()
        # With dyntype=none and unit direct gears, ctrl is the actual applied
        # generalized wrench command for these four actuators.
        actual = WrenchCommand(float(data.ctrl[self.ids[0]]), data.ctrl[list(self.ids[1:])])
        saturated = ~np.isclose(command.as_array(), clipped.as_array(), rtol=0.0, atol=0.0)
        return AppliedPhysicalCommand(
            command, clipped, actual, bool(saturated[0]), saturated[1:],
            int(tick), float(tick * physics_dt_s),
        )
