"""Integer-tick deterministic multi-rate scheduler."""

from __future__ import annotations

from dataclasses import dataclass


SUPPORTED_RATES_HZ = (20, 50, 100, 200, 500, 1000)


@dataclass(frozen=True, slots=True)
class ComponentSchedule:
    name: str
    rate_hz: int
    stride_ticks: int
    phase_tick: int = 0


class DeterministicMultiRateScheduler:
    """Schedules updates before command application at exact integer ticks.

    Updated component outputs use zero-order hold until their next scheduled
    tick. Physics integration happens after all due updates and command
    application at the same tick.
    """

    def __init__(self, physics_rate_hz: int = 1000) -> None:
        if physics_rate_hz != 1000:
            raise ValueError("Native-Stack Benchmark v1 freezes physics at 1000 Hz")
        self.physics_rate_hz = int(physics_rate_hz)
        self.physics_dt_s = 1.0 / self.physics_rate_hz
        self._components: dict[str, ComponentSchedule] = {}

    def register(self, name: str, rate_hz: int, phase_tick: int = 0) -> ComponentSchedule:
        if rate_hz not in SUPPORTED_RATES_HZ or self.physics_rate_hz % rate_hz:
            raise ValueError(f"unsupported controller rate: {rate_hz}")
        stride = self.physics_rate_hz // rate_hz
        if not 0 <= phase_tick < stride:
            raise ValueError("phase_tick must be inside one component period")
        if name in self._components:
            raise ValueError(f"duplicate component: {name}")
        schedule = ComponentSchedule(str(name), int(rate_hz), stride, int(phase_tick))
        self._components[name] = schedule
        return schedule

    def due(self, name: str, tick: int) -> bool:
        schedule = self._components[name]
        return tick >= schedule.phase_tick and (tick - schedule.phase_tick) % schedule.stride_ticks == 0

    def due_components(self, tick: int) -> tuple[str, ...]:
        if tick < 0:
            raise ValueError("tick must be nonnegative")
        return tuple(name for name in self._components if self.due(name, tick))

    def timestamp(self, tick: int) -> float:
        return int(tick) / self.physics_rate_hz

    @property
    def components(self) -> tuple[ComponentSchedule, ...]:
        return tuple(self._components.values())
