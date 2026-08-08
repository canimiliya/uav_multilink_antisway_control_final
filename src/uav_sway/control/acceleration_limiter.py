"""The frozen S3 amplitude-then-slew acceleration limiter."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AccelerationLimitDiagnostics:
    raw: float
    amplitude_limited: float
    limited: float
    saturated: bool
    slew_limited: bool


class AccelerationLimiter:
    def __init__(self, ax_min: float = -2.0, ax_max: float = 2.0,
                 slew_limit: float = 0.25):
        if ax_min >= ax_max or slew_limit < 0:
            raise ValueError("invalid acceleration limits")
        self.ax_min = float(ax_min)
        self.ax_max = float(ax_max)
        self.slew_limit = float(slew_limit)
        self.previous = 0.0
        self.diagnostics = AccelerationLimitDiagnostics(0.0, 0.0, 0.0, False, False)

    def reset(self, value: float = 0.0) -> None:
        self.previous = float(value)
        self.diagnostics = AccelerationLimitDiagnostics(self.previous, self.previous, self.previous, False, False)

    def limit(self, raw: float) -> float:
        raw = float(raw)
        amplitude = min(self.ax_max, max(self.ax_min, raw))
        delta = amplitude - self.previous
        limited = self.previous + min(self.slew_limit, max(-self.slew_limit, delta))
        limited = min(self.ax_max, max(self.ax_min, limited))
        self.diagnostics = AccelerationLimitDiagnostics(
            raw, amplitude, float(limited), bool(amplitude != raw or limited != amplitude),
            bool(limited != amplitude),
        )
        self.previous = float(limited)
        return self.previous
