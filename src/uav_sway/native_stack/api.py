"""Typed causal interfaces for Native-Stack Benchmark v1."""

from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Any

import numpy as np


def _vector(value: Any, size: int, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=float).reshape(size).copy()
    if not np.isfinite(result).all():
        raise ValueError(f"{name} must be finite")
    result.setflags(write=False)
    return result


@dataclass(frozen=True, slots=True)
class WrenchCommand:
    """Requested body-z thrust and body-frame torque."""

    thrust_N: float
    torque_Nm: np.ndarray

    def __post_init__(self) -> None:
        if not np.isfinite(self.thrust_N):
            raise ValueError("thrust_N must be finite")
        object.__setattr__(self, "thrust_N", float(self.thrust_N))
        object.__setattr__(self, "torque_Nm", _vector(self.torque_Nm, 3, "torque_Nm"))

    def as_array(self) -> np.ndarray:
        return np.r_[self.thrust_N, self.torque_Nm]


@dataclass(frozen=True, slots=True)
class AppliedPhysicalCommand:
    requested: WrenchCommand
    clipped: WrenchCommand
    actual: WrenchCommand
    thrust_saturated: bool
    torque_saturated: np.ndarray
    application_tick: int
    application_time_s: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "torque_saturated", _vector(self.torque_saturated, 3, "torque_saturated").astype(bool))
        if self.application_tick < 0 or not np.isfinite(self.application_time_s):
            raise ValueError("invalid application time")


@dataclass(frozen=True, slots=True)
class ReferenceSample:
    """Current causal cutter-tip reference; no preview samples are included."""
    position_world: np.ndarray
    velocity_world: np.ndarray
    acceleration_world: np.ndarray
    jerk_world: np.ndarray
    time_s: float

    def __post_init__(self) -> None:
        for name in ("position_world", "velocity_world", "acceleration_world", "jerk_world"):
            object.__setattr__(self, name, _vector(getattr(self, name), 3, name))
        if not np.isfinite(self.time_s):
            raise ValueError("time_s must be finite")


@dataclass(frozen=True, slots=True)
class SensorPacket:
    """Complete information available to every runtime controller.

    There is deliberately no wind, future reference, external force, or split
    metadata field.
    """

    time_s: float
    tick: int
    uav_position_world: np.ndarray
    uav_velocity_world: np.ndarray
    rotation_world_from_body: np.ndarray
    body_angular_velocity: np.ndarray
    joint_position: np.ndarray
    joint_velocity: np.ndarray
    cutter_tip_position_world: np.ndarray
    cutter_tip_velocity_world: np.ndarray
    reference: ReferenceSample  # cutter-tip reference at current time only
    previous_applied_command: WrenchCommand

    def __post_init__(self) -> None:
        if self.tick < 0 or not np.isfinite(self.time_s):
            raise ValueError("invalid sensor timestamp")
        for name, size in (
            ("uav_position_world", 3), ("uav_velocity_world", 3),
            ("body_angular_velocity", 3), ("joint_position", 5),
            ("joint_velocity", 5), ("cutter_tip_position_world", 3),
            ("cutter_tip_velocity_world", 3),
        ):
            object.__setattr__(self, name, _vector(getattr(self, name), size, name))
        rotation = np.asarray(self.rotation_world_from_body, dtype=float).reshape(3, 3).copy()
        if not np.isfinite(rotation).all():
            raise ValueError("rotation must be finite")
        rotation.setflags(write=False)
        object.__setattr__(self, "rotation_world_from_body", rotation)

    @classmethod
    def field_names(cls) -> tuple[str, ...]:
        return tuple(field.name for field in fields(cls))


@dataclass(frozen=True, slots=True)
class DiagnosticTruthPacket:
    """Offline-only truth; forbidden from runtime controller ownership."""

    time_s: float
    wind_velocity_world: np.ndarray
    external_force_world: np.ndarray
    external_torque_body: np.ndarray

    def __post_init__(self) -> None:
        for name in ("wind_velocity_world", "external_force_world", "external_torque_body"):
            object.__setattr__(self, name, _vector(getattr(self, name), 3, name))
        if not np.isfinite(self.time_s):
            raise ValueError("time_s must be finite")
