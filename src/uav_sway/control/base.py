"""Shared controller state and protocol definitions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class ReferenceState:
    x_ref: float
    vx_ref: float
    ax_ref: float
    y_ref: float
    z_ref: float
    yaw_ref: float


@dataclass(frozen=True)
class ControlState:
    position: np.ndarray
    velocity: np.ndarray
    rotation: np.ndarray
    body_angular_velocity: np.ndarray
    joint_angles: np.ndarray
    joint_velocities: np.ndarray
    tip_displacement: float

    @property
    def uav_x(self) -> float:
        return float(self.position[0])

    @property
    def uav_vx(self) -> float:
        return float(self.velocity[0])


class SwayController(Protocol):
    def reset(self, state: ControlState, reference: ReferenceState) -> None: ...

    def command(self, state: ControlState, reference: ReferenceState, dt: float) -> float: ...
