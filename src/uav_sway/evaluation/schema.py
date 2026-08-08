"""Frozen column order for S2 raw run CSV files."""

BASE_COLUMNS = [
    "time", "scenario", "seed", "protocol_mode", "wind_x", "wind_y", "wind_z",
    "x_ref", "vx_ref", "ax_ref", "y_ref", "z_ref", "yaw_ref", "uav_x", "uav_y", "uav_z", "uav_vx", "uav_vy", "uav_vz",
    "uav_qw", "uav_qx", "uav_qy", "uav_qz",
]
TAIL_COLUMNS = [
    "tip_x", "tip_y", "tip_z", "tip_relative_x", "tip_equilibrium_relative_x", "tip_displacement",
    "wind_force_quad_x", "wind_force_cutter_x", "wind_force_total_x",
    "ax_cmd_raw", "ax_cmd_limited", "ax_saturated", "solve_time_ms",
]


def schema_columns(n_links: int) -> list[str]:
    if n_links < 1:
        raise ValueError("n_links must be positive")
    joints = [f"joint_{i}_angle" for i in range(1, n_links + 1)]
    velocities = [f"joint_{i}_velocity" for i in range(1, n_links + 1)]
    link_forces = [f"wind_force_link_{i}_x" for i in range(1, n_links + 1)]
    return BASE_COLUMNS + joints + velocities + TAIL_COLUMNS[:6] + ["wind_force_quad_x"] + link_forces + ["wind_force_cutter_x", "wind_force_total_x", "ax_cmd_raw", "ax_cmd_limited", "ax_saturated", "solve_time_ms"]


def schema_description(n_links: int) -> dict:
    return {"columns": schema_columns(n_links), "protocol_mode": "anchored_wind_validation", "controller": "none", "types": {"seed": "integer", "ax_saturated": "boolean"}}
