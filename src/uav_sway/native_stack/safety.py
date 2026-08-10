"""Preregistered Native Benchmark safety-v2 envelope."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .api import AppliedPhysicalCommand, SensorPacket


@dataclass(frozen=True, slots=True)
class SafetyV2Limits:
    minimum_height_m: float = 0.05
    maximum_joint_angle_rad: float = float(np.deg2rad(100.0))
    maximum_roll_pitch_rad: float = float(np.deg2rad(25.0))
    maximum_uav_horizontal_displacement_m: float = 8.0
    maximum_tip_horizontal_displacement_m: float = 10.0


def evaluate_safety_v2(
    packet: SensorPacket, applied: AppliedPhysicalCommand, origin_uav: np.ndarray,
    origin_tip: np.ndarray, limits: SafetyV2Limits = SafetyV2Limits(),
) -> tuple[bool, tuple[str, ...]]:
    reasons: list[str] = []
    values = np.r_[
        packet.uav_position_world, packet.uav_velocity_world, packet.joint_position,
        packet.joint_velocity, packet.cutter_tip_position_world, applied.actual.as_array(),
    ]
    if not np.isfinite(values).all(): reasons.append("non_finite")
    if packet.uav_position_world[2] <= limits.minimum_height_m: reasons.append("uav_height")
    if packet.cutter_tip_position_world[2] <= limits.minimum_height_m: reasons.append("tip_height")
    if np.max(np.abs(packet.joint_position)) >= limits.maximum_joint_angle_rad: reasons.append("joint_angle")
    rotation = packet.rotation_world_from_body
    roll = float(np.arctan2(rotation[2, 1], rotation[2, 2]))
    pitch = float(np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if max(abs(roll), abs(pitch)) >= limits.maximum_roll_pitch_rad: reasons.append("attitude")
    uav_horizontal = float(np.linalg.norm(packet.uav_position_world[:2] - np.asarray(origin_uav)[:2]))
    tip_horizontal = float(np.linalg.norm(packet.cutter_tip_position_world[:2] - np.asarray(origin_tip)[:2]))
    if uav_horizontal > limits.maximum_uav_horizontal_displacement_m: reasons.append("uav_workspace")
    if tip_horizontal > limits.maximum_tip_horizontal_displacement_m: reasons.append("tip_workspace")
    if not np.array_equal(applied.actual.as_array(), applied.clipped.as_array()): reasons.append("actuator_application")
    return not reasons, tuple(reasons)
