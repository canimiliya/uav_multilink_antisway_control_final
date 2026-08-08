"""Task-space metrics and acquisition contract for S6T0."""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from .metrics import control_rate_proxy


POSITION_TOLERANCE_M = 0.05
ORIENTATION_TOLERANCE_DEG = 5.0
TIP_SPEED_TOLERANCE_M_S = 0.10
HOLD_TIME_S = 1.0


def _integral(values: np.ndarray, time: np.ndarray) -> float:
    return float(np.trapezoid(values, time)) if hasattr(np, "trapezoid") else float(np.trapz(values, time))


def _finite(values: dict[str, np.ndarray]) -> bool:
    return all(np.isfinite(value).all() for value in values.values() if value.dtype != object and value.dtype != bool)


def task_acquisition_mask(position_error_xz_m: np.ndarray, orientation_error_deg: np.ndarray, tip_speed_m_s: np.ndarray) -> np.ndarray:
    return (position_error_xz_m <= POSITION_TOLERANCE_M) & (orientation_error_deg <= ORIENTATION_TOLERANCE_DEG) & (tip_speed_m_s <= TIP_SPEED_TOLERANCE_M_S)


def first_continuous_acquisition(time: np.ndarray, mask: np.ndarray, hold_time_s: float = HOLD_TIME_S, start_time_s: float | None = None) -> tuple[bool, float | None]:
    time = np.asarray(time, dtype=float)
    mask = np.asarray(mask, dtype=bool)
    if len(time) != len(mask) or len(time) == 0 or np.any(np.diff(time) <= 0.0):
        raise ValueError("time and acquisition mask must be non-empty and strictly increasing")
    start = 0 if start_time_s is None else int(np.searchsorted(time, start_time_s, side="left"))
    for index in range(start, len(time)):
        if not mask[index]:
            continue
        end = int(np.searchsorted(time, time[index] + hold_time_s - 1.0e-12, side="left"))
        if end < len(time) and bool(np.all(mask[index : end + 1])) and time[end] - time[index] >= hold_time_s - 1.0e-9:
            return True, float(time[index])
    return False, None


def _gust_window(values: dict[str, np.ndarray]) -> np.ndarray:
    if "wind_x" not in values or ("scenario" in values and str(values["scenario"][0]) not in {"gust_micro_adjust", "one_cosine_gust"}):
        return np.zeros(len(values["time"]), dtype=bool)
    return np.abs(values["wind_x"]) > 1.0e-9


def compute_task_metrics(path: str | Path) -> dict:
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError("empty task-space CSV")
    columns = list(rows[0])
    text = {"scenario", "controller", "protocol_mode", "reference_event"}
    boolean = {"ax_saturated", "anchor_active", "ax_slew_limited", "inner_loop_saturated"}
    values = {
        column: np.asarray(
            [row[column] for row in rows],
            dtype=object if column in text else bool if column in boolean else float,
        )
        if column not in boolean
        else np.asarray([row[column].lower() == "true" for row in rows], dtype=bool)
        for column in columns
    }
    time = values["time"]
    if len(time) < 2 or np.any(np.diff(time) <= 0.0):
        raise ValueError("task-space time must be strictly increasing")
    duration = float(time[-1] - time[0])
    position_xz = values["task_position_error_xz_m"]
    orientation_deg = values["orientation_error_deg"]
    speed = values["tip_speed_m_s"]
    task_start_time = float(time[0])
    if "reference_event" in values:
        non_hover = np.flatnonzero(values["reference_event"] != "hover")
        if len(non_hover):
            task_start_time = float(time[non_hover[0]])
    acquire, acquisition_timestamp = first_continuous_acquisition(
        time,
        task_acquisition_mask(position_xz, orientation_deg, speed),
        start_time_s=task_start_time,
    )
    acquisition_time = (
        None
        if acquisition_timestamp is None
        else float(acquisition_timestamp - task_start_time)
    )
    gust = _gust_window(values)
    gust_last = int(np.flatnonzero(gust)[-1]) if np.any(gust) else None
    gust_peak_position = float(np.max(position_xz[gust])) if gust_last is not None else None
    gust_peak_orientation = float(np.max(orientation_deg[gust])) if gust_last is not None else None
    recovery_time = None
    if gust_last is not None:
        recovered, recovered_at = first_continuous_acquisition(time, task_acquisition_mask(position_xz, orientation_deg, speed), start_time_s=float(time[gust_last]))
        if recovered and recovered_at is not None:
            recovery_time = float(recovered_at - time[gust_last])
    ax = values["ax_cmd_limited"]
    position_3d = values["position_error_3d_m"]
    uav_x_error = values["uav_x"] - values["x_ref"]
    uav_z_error = values["uav_z"] - values["z_ref"]
    numeric = {key: value for key, value in values.items() if value.dtype != object}
    return {
        "source_csv": str(path), "sample_count": int(len(time)), "duration_s": duration,
        "tip_task_position_rmse_m": float(np.sqrt(_integral(position_xz ** 2, time) / duration)),
        "tip_task_x_rmse_m": float(np.sqrt(_integral(values["tip_task_x_error_m"] ** 2, time) / duration)),
        "tip_task_z_rmse_m": float(np.sqrt(_integral(values["tip_task_z_error_m"] ** 2, time) / duration)),
        "cutter_orientation_rmse_deg": float(np.sqrt(_integral(orientation_deg ** 2, time) / duration)),
        "cutter_orientation_max_deg": float(np.max(orientation_deg)),
        "tip_speed_rms_m_s": float(np.sqrt(_integral(speed ** 2, time) / duration)),
        "task_acquired": bool(acquire),
        "task_start_time_s": task_start_time,
        "task_acquisition_timestamp_s": acquisition_timestamp,
        "task_acquisition_time_s": acquisition_time,
        "final_tip_position_error_m": float(position_3d[-1]), "final_orientation_error_deg": float(orientation_deg[-1]),
        "gust_peak_tip_position_error_m": gust_peak_position, "gust_peak_orientation_error_deg": gust_peak_orientation,
        "gust_recovery_time_s": recovery_time,
        "control_energy_proxy": _integral(ax ** 2, time),
        "control_rate_proxy": control_rate_proxy(time, ax),
        "solve_time_mean_ms": float(np.mean(values["solve_time_ms"])), "solve_time_p95_ms": float(np.percentile(values["solve_time_ms"], 95)),
        "uav_x": float(values["uav_x"][-1]), "uav_z": float(values["uav_z"][-1]),
        "uav_position_rmse": float(np.sqrt(_integral(uav_x_error ** 2 + uav_z_error ** 2, time) / duration)),
        "uav_x_rmse_m": float(np.sqrt(_integral(uav_x_error ** 2, time) / duration)),
        "uav_z_rmse_m": float(np.sqrt(_integral(uav_z_error ** 2, time) / duration)),
        "uav_metrics_secondary": True, "finite_outputs": _finite(numeric),
        "controller": str(values["controller"][0]), "scenario": str(values["scenario"][0]),
        "legacy_tip_rms_m": float(np.sqrt(_integral(values["tip_displacement"] ** 2, time) / duration)),
        "legacy_x_position_rmse_m": float(np.sqrt(_integral(uav_x_error ** 2, time) / duration)),
        "legacy_z_position_rmse_m": float(np.sqrt(_integral(uav_z_error ** 2, time) / duration)),
    }
