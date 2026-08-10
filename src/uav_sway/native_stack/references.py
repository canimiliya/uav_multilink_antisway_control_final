"""Deterministic causal setpoint and smooth cutter reference generators."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .api import ReferenceSample


def _quintic_scalars(s: float, duration: float) -> tuple[float, float, float, float]:
    s = float(np.clip(s, 0.0, 1.0))
    p = 10*s**3 - 15*s**4 + 6*s**5
    v = (30*s**2 - 60*s**3 + 30*s**4) / duration
    a = (60*s - 180*s**2 + 120*s**3) / duration**2
    j = (60 - 360*s + 360*s**2) / duration**3
    if s <= 0.0 or s >= 1.0:
        v = a = 0.0
        j = 0.0
    return p, v, a, j


@dataclass(frozen=True, slots=True)
class MinimumJerkReference:
    start: np.ndarray
    goal: np.ndarray
    start_time_s: float
    duration_s: float

    def sample(self, time_s: float) -> ReferenceSample:
        if self.duration_s <= 0:
            raise ValueError("duration_s must be positive")
        start = np.asarray(self.start, dtype=float).reshape(3)
        delta = np.asarray(self.goal, dtype=float).reshape(3) - start
        p, v, a, j = _quintic_scalars((time_s - self.start_time_s) / self.duration_s, self.duration_s)
        return ReferenceSample(start + p*delta, v*delta, a*delta, j*delta, float(time_s))


@dataclass(frozen=True, slots=True)
class ApproachStopReference(MinimumJerkReference):
    """A quintic approach whose endpoint velocity and acceleration are zero."""


class WaypointReference:
    def __init__(self, waypoints: np.ndarray, times_s: np.ndarray) -> None:
        self.waypoints = np.asarray(waypoints, dtype=float)
        self.times_s = np.asarray(times_s, dtype=float)
        if self.waypoints.ndim != 2 or self.waypoints.shape[1] != 3:
            raise ValueError("waypoints must be Nx3")
        if len(self.waypoints) != len(self.times_s) or len(self.times_s) < 2:
            raise ValueError("waypoint/timestamp mismatch")
        if np.any(np.diff(self.times_s) <= 0):
            raise ValueError("times must be strictly increasing")

    def sample(self, time_s: float) -> ReferenceSample:
        index = int(np.clip(np.searchsorted(self.times_s, time_s, side="right") - 1, 0, len(self.times_s) - 2))
        generator = MinimumJerkReference(
            self.waypoints[index], self.waypoints[index + 1],
            float(self.times_s[index]), float(self.times_s[index + 1] - self.times_s[index]),
        )
        return generator.sample(float(time_s))
