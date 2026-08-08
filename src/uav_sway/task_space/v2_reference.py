"""V2 cutter-target to UAV-reference mapping and shared 3-D limits."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CutterTargetMapper:
    """Map an external cutter position target using frozen equilibrium geometry."""

    tip_relative_equilibrium_m: np.ndarray
    cutter_axis_equilibrium: np.ndarray

    def __post_init__(self) -> None:
        relative = np.asarray(self.tip_relative_equilibrium_m, dtype=float).reshape(3)
        axis = np.asarray(self.cutter_axis_equilibrium, dtype=float).reshape(3)
        if not np.isfinite(relative).all() or not np.isfinite(axis).all():
            raise ValueError("V2 equilibrium geometry must be finite")
        norm = float(np.linalg.norm(axis))
        if norm <= 1.0e-12:
            raise ValueError("V2 equilibrium cutter axis must be non-zero")
        object.__setattr__(self, "tip_relative_equilibrium_m", relative.copy())
        object.__setattr__(self, "cutter_axis_equilibrium", axis / norm)

    def uav_reference_from_tip_target(self, tip_target_world_m: np.ndarray) -> np.ndarray:
        target = np.asarray(tip_target_world_m, dtype=float).reshape(3)
        if not np.isfinite(target).all():
            raise ValueError("cutter target must be finite")
        return target - self.tip_relative_equilibrium_m

    def tip_target_from_uav_reference(self, uav_reference_world_m: np.ndarray) -> np.ndarray:
        reference = np.asarray(uav_reference_world_m, dtype=float).reshape(3)
        if not np.isfinite(reference).all():
            raise ValueError("UAV reference must be finite")
        return reference + self.tip_relative_equilibrium_m


@dataclass(frozen=True)
class Shared3DControlLimits:
    """Uniform V2 y/z acceleration and per-update slew limits."""

    ay_abs_max_m_s2: float = 2.0
    az_abs_max_m_s2: float = 2.0
    axis_slew_max_m_s2_per_update: float = 0.25

    def apply(self, desired_ay: float, desired_az: float,
              previous_ay: float = 0.0, previous_az: float = 0.0) -> tuple[float, float]:
        values = np.asarray([desired_ay, desired_az, previous_ay, previous_az], dtype=float)
        if not np.isfinite(values).all():
            raise ValueError("shared y/z acceleration values must be finite")
        ay = float(np.clip(desired_ay, -self.ay_abs_max_m_s2, self.ay_abs_max_m_s2))
        az = float(np.clip(desired_az, -self.az_abs_max_m_s2, self.az_abs_max_m_s2))
        ay = float(previous_ay + np.clip(ay - previous_ay, -self.axis_slew_max_m_s2_per_update, self.axis_slew_max_m_s2_per_update))
        az = float(previous_az + np.clip(az - previous_az, -self.axis_slew_max_m_s2_per_update, self.axis_slew_max_m_s2_per_update))
        return ay, az
