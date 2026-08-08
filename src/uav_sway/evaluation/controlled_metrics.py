"""Metrics recomputed directly from an S3 raw controlled CSV."""

from __future__ import annotations

import csv
from pathlib import Path

import numpy as np

from .metrics import _integral, _settling, control_rate_proxy


BOOL_COLUMNS = {"ax_saturated", "ax_slew_limited", "inner_loop_saturated", "anchor_active", "observer_enabled", "disturbance_compensation"}
TEXT_COLUMNS = {"scenario", "protocol_mode", "controller", "controller_mode"}


def load_controlled_csv(path: str | Path) -> tuple[list[str], dict[str, np.ndarray]]:
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        columns = reader.fieldnames or []
        rows = list(reader)
    if not rows:
        raise ValueError("empty controlled CSV")
    values: dict[str, np.ndarray] = {}
    for column in columns:
        if column in TEXT_COLUMNS:
            values[column] = np.asarray([row[column] for row in rows], dtype=object)
        elif column in BOOL_COLUMNS:
            values[column] = np.asarray([row[column].lower() == "true" for row in rows], dtype=bool)
        else:
            values[column] = np.asarray([float(row[column]) for row in rows], dtype=float)
    return columns, values


def compute_controlled_metrics(path: str | Path, settling_start_s: float = 0.0) -> dict:
    columns, values = load_controlled_csv(path)
    time = values["time"]
    duration = float(time[-1] - time[0])
    if duration <= 0.0 or np.any(np.diff(time) <= 0.0):
        raise ValueError("controlled time must be strictly increasing")
    finite = all(np.isfinite(value).all() for value in values.values() if value.dtype != object and value.dtype != bool)
    tip = values["tip_displacement"]
    pos_error_x = values["uav_x"] - values["x_ref"]
    z_error = values["uav_z"] - values["z_ref"]
    joint_columns = sorted((c for c in columns if c.startswith("joint_") and c.endswith("_angle")), key=lambda c: int(c.split("_")[1]))
    max_joint = float(max(np.max(np.abs(values[c])) for c in joint_columns)) if joint_columns else 0.0
    settled, settling_time = _settling(time, tip, settling_start_s, band=0.05, hold_time=1.0)
    return {
        "source_csv": str(path), "sample_count": int(len(time)), "duration_s": duration,
        "tip_max_abs_m": float(np.max(np.abs(tip))), "tip_rms_m": float(np.sqrt(_integral(tip**2, time) / duration)),
        "uav_position_rmse_m": float(np.sqrt(_integral(pos_error_x**2 + (values["uav_y"] - values["y_ref"])**2 + z_error**2, time) / duration)),
        "x_position_rmse_m": float(np.sqrt(_integral(pos_error_x**2, time) / duration)),
        "z_position_rmse_m": float(np.sqrt(_integral(z_error**2, time) / duration)),
        "final_x_error_m": float(pos_error_x[-1]), "final_z_error_m": float(z_error[-1]),
        "settled": bool(settled), "settling_time_s": settling_time, "settling_band_m": 0.05, "settling_hold_time_s": 1.0,
        "control_energy_proxy": _integral(values["ax_cmd_limited"]**2, time),
        "control_rate_proxy": control_rate_proxy(time, values["ax_cmd_limited"]),
        "solve_time_mean_ms": float(np.mean(values["solve_time_ms"])), "solve_time_p95_ms": float(np.percentile(values["solve_time_ms"], 95)), "solve_time_max_ms": float(np.max(values["solve_time_ms"])),
        "saturation_rate": float(np.mean(values["ax_saturated"])), "finite_outputs": bool(finite),
        "minimum_tip_height_m": float(np.min(values["tip_z"])), "minimum_uav_height_m": float(np.min(values["uav_z"])), "maximum_abs_joint_angle_rad": max_joint,
        "maximum_abs_roll_rad": float(np.max(np.abs(values["roll_rad"]))), "maximum_abs_pitch_rad": float(np.max(np.abs(values["pitch_rad"]))),
        "maximum_abs_ax_m_s2": float(np.max(np.abs(values["ax_cmd_limited"]))),
        "maximum_ax_step_change_m_s2": float(np.max(np.abs(np.diff(values["ax_cmd_limited"])))) if len(time) > 1 else 0.0,
        "thrust_saturation_rate": float(np.mean(values["thrust_cmd_limited_N"] != values["thrust_cmd_raw_N"])),
        "torque_saturation_rate": float(np.mean(np.any(np.column_stack([values["mx_cmd_limited_Nm"] != values["mx_cmd_raw_Nm"], values["my_cmd_limited_Nm"] != values["my_cmd_raw_Nm"], values["mz_cmd_limited_Nm"] != values["mz_cmd_raw_Nm"]]), axis=1))),
        "anchor_active_any": bool(np.any(values["anchor_active"])), "controller": str(values["controller"][0]), "protocol_mode": str(values["protocol_mode"][0]),
    }
