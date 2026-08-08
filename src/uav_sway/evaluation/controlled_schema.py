"""Frozen S3 controlled-run CSV column contract."""

from __future__ import annotations

from .schema import schema_columns


def controlled_schema_columns(n_links: int) -> list[str]:
    additions = [
        "controller", "anchor_active", "position_error_x", "velocity_error_x", "pid_integral_x",
        "ax_reference_feedforward", "ax_pid_feedback", "ax_cmd_amplitude_limited", "ax_slew_limited",
        "thrust_cmd_raw_N", "thrust_cmd_limited_N", "mx_cmd_raw_Nm", "my_cmd_raw_Nm", "mz_cmd_raw_Nm",
        "mx_cmd_limited_Nm", "my_cmd_limited_Nm", "mz_cmd_limited_Nm", "inner_loop_saturated",
        "roll_rad", "pitch_rad", "yaw_rad",
    ]
    return schema_columns(n_links) + additions


def controlled_schema_description(n_links: int) -> dict:
    return {
        "columns": controlled_schema_columns(n_links),
        "protocol_mode": "free_flight_controlled",
        "controller": "pid",
        "anchor_active": False,
        "types": {"seed": "integer", "anchor_active": "boolean", "ax_saturated": "boolean", "ax_slew_limited": "boolean", "inner_loop_saturated": "boolean"},
    }
