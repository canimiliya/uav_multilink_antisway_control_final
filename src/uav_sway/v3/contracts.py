"""Frozen V3 full-3D controller interface.

The contract is deliberately independent from the V1/V2 ``SwayController``
protocol.  It provides validation and common command limiting only; it does
not select, tune, or execute a controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol, Sequence

import numpy as np


@dataclass(frozen=True)
class V3AccelerationCommand:
    """World-frame outer-loop acceleration command in m/s^2."""

    ax: float
    ay: float
    az: float

    def __post_init__(self) -> None:
        values = np.asarray([self.ax, self.ay, self.az], dtype=float)
        if values.shape != (3,) or not np.isfinite(values).all():
            raise ValueError("V3 acceleration command must be a finite 3-vector")
        object.__setattr__(self, "ax", float(self.ax))
        object.__setattr__(self, "ay", float(self.ay))
        object.__setattr__(self, "az", float(self.az))

    def as_array(self) -> np.ndarray:
        return np.asarray([self.ax, self.ay, self.az], dtype=float)

    @classmethod
    def from_array(cls, value: Sequence[float] | np.ndarray) -> "V3AccelerationCommand":
        array = np.asarray(value, dtype=float)
        if array.shape != (3,):
            raise ValueError("V3 command shape must be (3,)")
        return cls(*array.tolist())


class V3Controller(Protocol):
    """Protocol required of every future V3 controller."""

    def command(self, state: object, reference: object, dt: float) -> np.ndarray: ...


@dataclass
class V3AccelerationLimiter:
    """Shared amplitude-then-slew limiter for all three command axes."""

    absolute_limit_m_s2: float = 2.0
    slew_limit_m_s2_per_update: float = 0.25
    previous: np.ndarray | None = None

    def __post_init__(self) -> None:
        if self.absolute_limit_m_s2 <= 0.0 or self.slew_limit_m_s2_per_update < 0.0:
            raise ValueError("invalid V3 command limits")
        if self.previous is None:
            self.previous = np.zeros(3, dtype=float)
        else:
            self.previous = np.asarray(self.previous, dtype=float).reshape(3).copy()
        if not np.isfinite(self.previous).all():
            raise ValueError("previous V3 command must be finite")

    def reset(self, value: Sequence[float] | np.ndarray = (0.0, 0.0, 0.0)) -> None:
        value = np.asarray(value, dtype=float)
        if value.shape != (3,) or not np.isfinite(value).all():
            raise ValueError("V3 limiter reset value must be a finite 3-vector")
        self.previous = value.copy()

    def limit(self, command: Sequence[float] | np.ndarray) -> V3AccelerationCommand:
        raw = np.asarray(command, dtype=float)
        if raw.shape != (3,) or not np.isfinite(raw).all():
            raise ValueError("V3 command shape must be (3,)")
        amplitude = np.clip(raw, -self.absolute_limit_m_s2, self.absolute_limit_m_s2)
        delta = np.clip(
            amplitude - self.previous,
            -self.slew_limit_m_s2_per_update,
            self.slew_limit_m_s2_per_update,
        )
        self.previous = self.previous + delta
        return V3AccelerationCommand.from_array(self.previous)


V3_INNER_LOOP_CONTRACT = {
    "name": "frozen_udaan_geometric_attitude_inner_loop",
    "force_mapping": "F_des = m * (a_des + g * e_z)",
    "outer_period_s": 0.05,
    "shared_across": ["3D Task PID", "3D Full-State LQR", "3D Task-Weighted LQR", "Self-Advanced", "Paper-Advanced"],
    "per_method_gain_overrides": False,
    "plant_safety_limits_inherited": True,
}
