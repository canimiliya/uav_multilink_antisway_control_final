"""Native controller contract and frozen legacy acceleration wrapper."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

import numpy as np

from uav_sway.control.base import ControlState, ReferenceState
from uav_sway.control.geometric_inner_loop import GeometricInnerLoop

from .api import DiagnosticTruthPacket, SensorPacket, WrenchCommand


class NativeStackController(ABC):
    """Runtime base class that rejects ownership of offline truth packets."""

    def __setattr__(self, name: str, value: Any) -> None:
        if isinstance(value, DiagnosticTruthPacket):
            raise TypeError("runtime controllers cannot own DiagnosticTruthPacket")
        super().__setattr__(name, value)

    @abstractmethod
    def reset(self) -> None: ...

    def observe(self, sensor_packet: SensorPacket) -> None:
        self._sensor_packet = sensor_packet

    def update_high_level(self) -> None:
        return None

    def update_inner(self) -> None:
        return None

    @abstractmethod
    def physical_command(self) -> WrenchCommand: ...

    def diagnostics(self) -> dict[str, Any]:
        return {}


class AccelerationOuterStackAdapter(NativeStackController):
    """Map an unchanged legacy [ax, ay, az] output through the old inner loop."""

    def __init__(self, legacy_controller: Any, inner_loop: GeometricInnerLoop) -> None:
        self.legacy_controller = legacy_controller
        self.inner_loop = inner_loop
        self._sensor_packet: SensorPacket | None = None
        self._acceleration = np.zeros(3)
        self._wrench = WrenchCommand(0.0, np.zeros(3))

    def reset(self) -> None:
        self.legacy_controller.reset()
        self.inner_loop.reset()
        self._sensor_packet = None
        self._acceleration[:] = 0.0
        self._wrench = WrenchCommand(0.0, np.zeros(3))

    def set_legacy_acceleration(self, acceleration: np.ndarray) -> None:
        value = np.asarray(acceleration, dtype=float).reshape(3)
        if not np.isfinite(value).all():
            raise ValueError("legacy acceleration must be finite")
        self._acceleration = value.copy()

    def update_inner(self) -> None:
        if self._sensor_packet is None:
            raise RuntimeError("observe must precede update_inner")
        packet = self._sensor_packet
        state = ControlState(
            packet.uav_position_world,
            packet.uav_velocity_world,
            packet.rotation_world_from_body,
            packet.body_angular_velocity,
            packet.joint_position,
            packet.joint_velocity,
            0.0,
        )
        reference = ReferenceState(
            float(packet.reference.position_world[0]),
            float(packet.reference.velocity_world[0]),
            float(packet.reference.acceleration_world[0]),
            float(packet.reference.position_world[1]),
            float(packet.reference.position_world[2]),
            float(packet.time_s),
        )
        output = self.inner_loop.compute(
            state, reference, float(self._acceleration[0]),
            (float(self._acceleration[1]), float(self._acceleration[2])),
        )
        self._wrench = WrenchCommand(float(output["thrust_raw_N"]), output["torque_raw_Nm"])

    def physical_command(self) -> WrenchCommand:
        return self._wrench

    def diagnostics(self) -> dict[str, Any]:
        return {"legacy_acceleration_m_s2": self._acceleration.copy()}
