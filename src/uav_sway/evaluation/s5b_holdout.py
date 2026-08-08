"""Independent raw-data utilities for the S5B holdout audit."""

from __future__ import annotations

import csv
import gzip
from pathlib import Path

import numpy as np


def _open_text(path: str | Path):
    path = Path(path)
    if path.suffix == ".gz":
        return gzip.open(path, "rt", encoding="utf-8", newline="")
    return path.open("r", encoding="utf-8", newline="")


def load_raw_csv(path: str | Path) -> tuple[list[str], dict[str, np.ndarray]]:
    """Load a controlled CSV (or lossless gzip-compressed CSV) without metrics JSON."""
    with _open_text(path) as stream:
        reader = csv.DictReader(stream)
        columns = list(reader.fieldnames or [])
        rows = list(reader)
    if not rows:
        raise ValueError(f"empty raw CSV: {path}")
    bool_columns = {
        "ax_saturated", "ax_slew_limited", "anchor_active",
        "observer_enabled", "disturbance_compensation",
    }
    text_columns = {"scenario", "protocol_mode", "controller", "controller_mode"}
    values: dict[str, np.ndarray] = {}
    for column in columns:
        if column in text_columns:
            values[column] = np.asarray([row[column] for row in rows], dtype=object)
        elif column in bool_columns:
            values[column] = np.asarray([row[column].strip().lower() == "true" for row in rows], dtype=bool)
        else:
            values[column] = np.asarray([float(row[column]) for row in rows], dtype=float)
    return columns, values


def _integral(values: np.ndarray, time: np.ndarray) -> float:
    return float(np.trapezoid(np.asarray(values, dtype=float), np.asarray(time, dtype=float)))


def _direct_metrics(path: str | Path, settling_start_s: float) -> dict:
    columns, values = load_raw_csv(path)
    time = values["time"]
    if np.any(~np.isfinite(time)) or np.any(np.diff(time) <= 0.0):
        raise ValueError(f"invalid time in {path}")
    duration = float(time[-1] - time[0])
    x_error = values["uav_x"] - values["x_ref"]
    z_error = values["uav_z"] - values["z_ref"]
    tip = values["tip_displacement"]
    joint_columns = sorted(
        (name for name in columns if name.startswith("joint_") and name.endswith("_angle")),
        key=lambda name: int(name.split("_")[1]),
    )
    all_numeric = [v for v in values.values() if v.dtype.kind in "fiu"]
    finite = bool(all(np.isfinite(v).all() for v in all_numeric))
    settle_mask = time >= float(settling_start_s)
    settled = False
    settling_time = None
    if np.any(settle_mask):
        indices = np.flatnonzero(settle_mask)
        for start in indices:
            end_time = time[start] + 1.0
            end = int(np.searchsorted(time, end_time, side="left"))
            if end > start and end <= len(time) and np.all(np.abs(tip[start:end]) < 0.05):
                settled = True
                settling_time = float(time[start])
                break
    max_joint = float(max(np.max(np.abs(values[name])) for name in joint_columns))
    ax = values["ax_cmd_limited"]
    thrust = values["thrust_cmd_limited_N"]
    torque = np.column_stack([
        values["mx_cmd_limited_Nm"], values["my_cmd_limited_Nm"], values["mz_cmd_limited_Nm"]
    ])
    rotor = values.get("rotor_motor_max_abs_cmd", np.zeros_like(time))
    return {
        "source_csv": str(path),
        "sample_count": int(len(time)),
        "duration_s": duration,
        "tip_rms_m": float(np.sqrt(_integral(tip**2, time) / duration)),
        "tip_max_abs_m": float(np.max(np.abs(tip))),
        "x_rmse_m": float(np.sqrt(_integral(x_error**2, time) / duration)),
        "z_rmse_m": float(np.sqrt(_integral(z_error**2, time) / duration)),
        "final_x_error_m": float(x_error[-1]),
        "settled": bool(settled),
        "settling_time_s": settling_time,
        "control_energy": _integral(ax**2, time),
        "control_rate": float(np.sum((np.diff(ax) / np.diff(time))**2 * np.diff(time))),
        "solve_time_mean_ms": float(np.mean(values["solve_time_ms"])),
        "solve_time_p95_ms": float(np.percentile(values["solve_time_ms"], 95)),
        "solve_time_max_ms": float(np.max(values["solve_time_ms"])),
        "saturation_rate": float(np.mean(values["ax_saturated"])),
        "finite_outputs": finite,
        "minimum_uav_height_m": float(np.min(values["uav_z"])),
        "minimum_tip_height_m": float(np.min(values["tip_z"])),
        "maximum_abs_joint_angle_rad": max_joint,
        "maximum_abs_roll_rad": float(np.max(np.abs(values["roll_rad"]))),
        "maximum_abs_pitch_rad": float(np.max(np.abs(values["pitch_rad"]))),
        "maximum_abs_ax_m_s2": float(np.max(np.abs(ax))),
        "maximum_ax_step_change_m_s2": float(np.max(np.abs(np.diff(ax)))),
        "thrust_min_N": float(np.min(thrust)),
        "thrust_max_N": float(np.max(thrust)),
        "torque_max_abs_Nm": float(np.max(np.abs(torque))),
        "rotor_motor_max_abs_cmd": float(np.max(np.abs(rotor))),
        "anchor_active_any": bool(np.any(values["anchor_active"])),
        "observer_enabled": bool(np.any(values.get("observer_enabled", np.zeros_like(time, dtype=bool)))),
        "controller_mode": str(values["controller_mode"][0]) if "controller_mode" in values else "unknown",
    }


def safety_check(metrics: dict) -> tuple[bool, list[str]]:
    reasons: list[str] = []
    if not metrics["finite_outputs"]:
        reasons.append("finite_outputs")
    if metrics["anchor_active_any"]:
        reasons.append("anchor_active")
    if metrics["minimum_uav_height_m"] <= 0.05:
        reasons.append("uav_height")
    if metrics["minimum_tip_height_m"] <= 0.05:
        reasons.append("tip_height")
    if metrics["maximum_abs_joint_angle_rad"] >= np.deg2rad(100.0):
        reasons.append("joint_range")
    if metrics["maximum_abs_roll_rad"] >= np.deg2rad(25.0):
        reasons.append("roll_range")
    if metrics["maximum_abs_pitch_rad"] >= np.deg2rad(25.0):
        reasons.append("pitch_range")
    if metrics["maximum_abs_ax_m_s2"] > 2.0 + 1e-8:
        reasons.append("ax_limit")
    if metrics["maximum_ax_step_change_m_s2"] > 0.25 + 1e-8:
        reasons.append("ax_slew")
    if metrics["thrust_min_N"] < -1e-8 or metrics["thrust_max_N"] > 285.74568 + 1e-8:
        reasons.append("thrust_limit")
    if metrics["torque_max_abs_Nm"] > 25.0 + 1e-8:
        reasons.append("torque_limit")
    if metrics["rotor_motor_max_abs_cmd"] != 0.0:
        reasons.append("rotor_motors")
    return not reasons, reasons


def percentile_summary(values: np.ndarray) -> dict[str, float]:
    values = np.asarray(values, dtype=float)
    return {
        "mean": float(np.mean(values)),
        "std": float(np.std(values, ddof=1)) if values.size > 1 else 0.0,
        "median": float(np.median(values)),
        "p25": float(np.percentile(values, 25)),
        "p75": float(np.percentile(values, 75)),
        "p95": float(np.percentile(values, 95)),
    }


def bootstrap_mean_ci(values: np.ndarray, rng: np.random.Generator, count: int = 10000) -> list[float]:
    values = np.asarray(values, dtype=float)
    samples = rng.choice(values, size=(count, values.size), replace=True)
    means = np.mean(samples, axis=1)
    return [float(np.percentile(means, 2.5)), float(np.percentile(means, 97.5))]


__all__ = ["load_raw_csv", "_direct_metrics", "safety_check", "percentile_summary", "bootstrap_mean_ci"]
