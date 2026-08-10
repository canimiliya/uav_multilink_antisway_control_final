"""Uniform physical command and controller timing records."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .api import AppliedPhysicalCommand


@dataclass(frozen=True, slots=True)
class CommandLogRecord:
    tick: int
    time_s: float
    requested: tuple[float, float, float, float]
    clipped: tuple[float, float, float, float]
    actual: tuple[float, float, float, float]
    thrust_saturated: bool
    torque_saturated: tuple[bool, bool, bool]
    command_rate_per_s: tuple[float, float, float, float]
    component_updates: tuple[str, ...]
    outer_solve_ms: float
    inner_solve_ms: float
    total_controller_ms: float


class NativeCommandLogger:
    def __init__(self, physics_dt_s: float) -> None:
        self.physics_dt_s = float(physics_dt_s)
        self.records: list[CommandLogRecord] = []

    def append(
        self, command: AppliedPhysicalCommand, component_updates: tuple[str, ...],
        outer_solve_ms: float = 0.0, inner_solve_ms: float = 0.0,
    ) -> CommandLogRecord:
        current = command.actual.as_array()
        previous = self.records[-1].actual if self.records else tuple(current)
        elapsed = max(self.physics_dt_s, command.application_time_s - (self.records[-1].time_s if self.records else command.application_time_s))
        rate = (current - np.asarray(previous)) / elapsed if self.records else np.zeros(4)
        record = CommandLogRecord(
            tick=command.application_tick, time_s=command.application_time_s,
            requested=tuple(float(x) for x in command.requested.as_array()),
            clipped=tuple(float(x) for x in command.clipped.as_array()),
            actual=tuple(float(x) for x in current),
            thrust_saturated=command.thrust_saturated,
            torque_saturated=tuple(bool(x) for x in command.torque_saturated),
            command_rate_per_s=tuple(float(x) for x in rate),
            component_updates=tuple(component_updates),
            outer_solve_ms=float(outer_solve_ms), inner_solve_ms=float(inner_solve_ms),
            total_controller_ms=float(outer_solve_ms + inner_solve_ms),
        )
        self.records.append(record)
        return record

    def timing_summary(self, component_rate_hz: int) -> dict[str, float | int | bool]:
        values = np.asarray([record.total_controller_ms for record in self.records], dtype=float)
        if not len(values):
            values = np.zeros(1)
        deadline_ms = 1000.0 / int(component_rate_hz)
        return {
            "mean_ms": float(np.mean(values)), "p95_ms": float(np.percentile(values, 95)),
            "p99_ms": float(np.percentile(values, 99)), "max_ms": float(np.max(values)),
            "deadline_ms": deadline_ms,
            "deadline_miss_count": int(np.count_nonzero(values > deadline_ms)),
            "deadline_met": bool(np.all(values <= deadline_ms)),
        }
