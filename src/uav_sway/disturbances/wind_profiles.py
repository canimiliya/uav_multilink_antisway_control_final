"""Deterministic one-dimensional world-x wind profiles for S2."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


GENERATOR_VERSION = "s2-wind-profiles-r1"


@dataclass(frozen=True)
class WindSeries:
    time: np.ndarray
    wind_x: np.ndarray
    profile: str
    seed: int | None
    dt: float
    duration: float


def sample_times(duration: float, dt: float) -> np.ndarray:
    count = int(round(duration / dt))
    if not np.isclose(count * dt, duration, atol=1e-12):
        raise ValueError("duration must be an integer multiple of dt")
    return np.arange(count + 1, dtype=float) * dt


def load_wind_config(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        return yaml.safe_load(stream)


def generate_wind_profile(
    profile: str,
    config: dict,
    seed: int | None = None,
    dt: float = 0.005,
) -> WindSeries:
    if profile not in config:
        raise KeyError(profile)
    spec = config[profile]
    duration = float(spec["duration_s"])
    dt_value = float(spec.get("dt_s", dt))
    time = sample_times(duration, dt_value)
    kind = spec["type"]
    wind = np.zeros_like(time)
    if kind == "constant":
        wind[time >= float(spec["start_time_s"])] = float(spec["speed_x_m_s"])
    elif kind == "one_cosine":
        start = float(spec["start_time_s"])
        end = start + float(spec["gust_duration_s"])
        mask = (time >= start) & (time <= end)
        tau = (time[mask] - start) / (end - start)
        wind[mask] = float(spec["peak_speed_x_m_s"]) * 0.5 * (1.0 - np.cos(2.0 * np.pi * tau))
        wind[(time < start) | (time > end)] = 0.0
    elif kind == "low_pass_gaussian":
        if seed is None:
            raise ValueError("low_frequency_random requires an explicit seed")
        rng = np.random.Generator(np.random.PCG64(seed))
        alpha = float(np.exp(-dt_value / float(spec["time_constant_s"])))
        sigma = float(spec["sigma_x_m_s"])
        mean = float(spec["mean_x_m_s"])
        previous = float(spec["initial_value_x_m_s"])
        wind[0] = previous
        for index in range(1, len(wind)):
            previous = alpha * previous + np.sqrt(1.0 - alpha * alpha) * sigma * float(rng.standard_normal())
            wind[index] = previous + mean
        wind = np.clip(wind, -float(spec["clip_abs_x_m_s"]), float(spec["clip_abs_x_m_s"]))
    else:
        raise ValueError(f"unsupported wind type: {kind}")
    if not np.isfinite(wind).all():
        raise ValueError("wind profile contains non-finite values")
    if "clip_abs_x_m_s" in spec and np.max(np.abs(wind)) > float(spec["clip_abs_x_m_s"]) + 1e-12:
        raise ValueError("wind profile exceeds configured clip")
    return WindSeries(time=time, wind_x=wind, profile=profile, seed=seed, dt=dt_value, duration=duration)
