"""Independent metrics computed directly from the raw S2 run CSV."""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np


def load_run_csv(path: str | Path) -> tuple[list[str], dict[str, np.ndarray]]:
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        columns = reader.fieldnames or []
        rows = list(reader)
    if not rows:
        raise ValueError("empty run CSV")
    numeric: dict[str, np.ndarray] = {}
    for column in columns:
        if column in {"scenario", "protocol_mode"}:
            continue
        if column == "ax_saturated":
            numeric[column] = np.asarray([row[column].lower() == "true" for row in rows], dtype=bool)
        else:
            numeric[column] = np.asarray([float(row[column]) for row in rows], dtype=float)
    numeric["scenario"] = np.asarray([row["scenario"] for row in rows], dtype=object)
    numeric["protocol_mode"] = np.asarray([row["protocol_mode"] for row in rows], dtype=object)
    return columns, numeric


def _integral(values: np.ndarray, time: np.ndarray) -> float:
    if len(time) < 2 or time[-1] <= time[0]:
        return 0.0
    return float(np.trapezoid(values, time))


def control_rate_proxy(time: np.ndarray, command: np.ndarray) -> float:
    """Compute sum((diff(command) / diff(time))**2 * diff(time))."""
    time = np.asarray(time, dtype=float)
    command = np.asarray(command, dtype=float)
    if time.shape != command.shape:
        raise ValueError("time and command must have the same shape")
    if len(time) < 2:
        return 0.0
    dt = np.diff(time)
    if np.any(dt <= 0.0):
        raise ValueError("time must be strictly increasing")
    du = np.diff(command)
    rate = du / dt
    return float(np.sum(rate**2 * dt))


def control_rate_formula_audit() -> dict:
    """Return production-computed examples for the frozen control-rate formula."""
    uniform_time = np.asarray([0.0, 1.0, 2.0])
    uniform_command = np.asarray([1.0, 2.0, 0.0])
    nonuniform_time = np.asarray([0.0, 0.5, 2.0])
    nonuniform_command = np.asarray([0.0, 1.0, 4.0])
    constant_command = np.asarray([2.0, 2.0, 2.0, 2.0])
    return {
        "metric": "control_rate_proxy",
        "formula": "sum((diff(u)/diff(t))^2 * diff(t))",
        "uniform_case": {
            "time": uniform_time.tolist(),
            "command": uniform_command.tolist(),
            "expected": 5.0,
            "computed": control_rate_proxy(uniform_time, uniform_command),
        },
        "nonuniform_case": {
            "time": nonuniform_time.tolist(),
            "command": nonuniform_command.tolist(),
            "expected": 8.0,
            "computed": control_rate_proxy(nonuniform_time, nonuniform_command),
        },
        "constant_case_computed": control_rate_proxy(
            np.arange(len(constant_command), dtype=float), constant_command
        ),
    }


def _settling(time: np.ndarray, signal: np.ndarray, start_time: float, band: float = 0.05, hold_time: float = 1.0) -> tuple[bool, float | None]:
    start_indices = np.flatnonzero(time >= start_time)
    if len(start_indices) == 0:
        return False, None
    for index in start_indices:
        end_indices = np.flatnonzero(time <= time[index] + hold_time + 1e-12)
        end_indices = end_indices[end_indices >= index]
        if len(end_indices) and time[end_indices[-1]] - time[index] >= hold_time - 1e-9:
            if np.all(np.abs(signal[index : end_indices[-1] + 1]) < band):
                return True, float(time[index])
    return False, None


def compute_metrics(path: str | Path, settling_start_s: float = 0.0) -> dict:
    columns, values = load_run_csv(path)
    time = values["time"]
    duration = float(time[-1] - time[0])
    displacement = values["tip_displacement"]
    initial_time = float(time[0])
    finite = all(np.isfinite(value).all() for key, value in values.items() if value.dtype != object and value.dtype != bool)
    joint_columns = sorted((column for column in columns if column.startswith("joint_") and column.endswith("_angle")), key=lambda value: int(value.split("_")[1]))
    joint_max = float(max(np.max(np.abs(values[column])) for column in joint_columns)) if joint_columns else 0.0
    position_error_sq = (values["uav_x"] - values["x_ref"]) ** 2 + (values["uav_y"] - values["y_ref"]) ** 2 + (values["uav_z"] - values["z_ref"]) ** 2
    dt = np.diff(time)
    if len(dt) and np.any(dt <= 0):
        raise ValueError("time must be strictly increasing")
    if duration <= 0:
        raise ValueError("run duration must be positive")
    settled, settling_time = _settling(time, displacement, settling_start_s)
    return {
        "source_csv": str(path),
        "sample_count": int(len(time)),
        "duration_s": duration,
        "tip_max_abs_m": float(np.max(np.abs(displacement))),
        "tip_rms_m": float(np.sqrt(_integral(displacement**2, time) / duration)),
        "uav_position_rmse_m": float(np.sqrt(_integral(position_error_sq, time) / duration)),
        "settled": bool(settled),
        "settling_time_s": settling_time,
        "settling_band_m": 0.05,
        "settling_hold_time_s": 1.0,
        "control_energy_proxy": _integral(values["ax_cmd_limited"] ** 2, time),
        "control_rate_proxy": control_rate_proxy(time, values["ax_cmd_limited"]),
        "solve_time_mean_ms": float(np.mean(values["solve_time_ms"])),
        "solve_time_p95_ms": float(np.percentile(values["solve_time_ms"], 95)),
        "solve_time_max_ms": float(np.max(values["solve_time_ms"])),
        "saturation_rate": float(np.mean(values["ax_saturated"])),
        "finite_outputs": bool(finite),
        "minimum_tip_height_m": float(np.min(values["tip_z"])),
        "maximum_abs_joint_angle_rad": joint_max,
        "protocol_mode": str(values["protocol_mode"][0]),
        "scenario": str(values["scenario"][0]),
        "seed": int(values["seed"][0]),
    }
