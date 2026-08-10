"""World-x quadratic drag proxies; all coefficients are simulation assumptions."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
import yaml


@dataclass(frozen=True)
class AerodynamicConfig:
    air_density: float
    airframe_cd: float
    airframe_dimensions: tuple[float, float, float]
    link_cd: float
    link_diameter: float
    cutter_cd: float
    cutter_dimensions: tuple[float, float, float]
    apply_torque: bool
    wind_axis: np.ndarray


def load_aerodynamic_config(path: str | Path) -> AerodynamicConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return AerodynamicConfig(
        air_density=float(raw["air_density_kg_m3"]),
        airframe_cd=float(raw["airframe"]["cd"]),
        airframe_dimensions=tuple(float(x) for x in raw["airframe"]["dimensions_xyz_m"]),
        link_cd=float(raw["link"]["cd"]),
        link_diameter=float(raw["link"]["diameter_m"]),
        cutter_cd=float(raw["cutter"]["cd"]),
        cutter_dimensions=tuple(float(x) for x in raw["cutter"]["dimensions_xyz_m"]),
        apply_torque=bool(raw["apply_aerodynamic_torque"]),
        wind_axis=np.asarray(raw["wind_axis_world"], dtype=float),
    )


def box_projected_area(dimensions_xyz: tuple[float, float, float], rotation: np.ndarray, wind_axis: np.ndarray) -> float:
    dimensions = np.asarray(dimensions_xyz, dtype=float)
    axis = np.asarray(wind_axis, dtype=float) / np.linalg.norm(wind_axis)
    return float(
        dimensions[1] * dimensions[2] * abs(axis @ rotation[:, 0])
        + dimensions[0] * dimensions[2] * abs(axis @ rotation[:, 1])
        + dimensions[0] * dimensions[1] * abs(axis @ rotation[:, 2])
    )


def link_projected_area(length: float, diameter: float, rotation: np.ndarray, wind_axis: np.ndarray) -> float:
    axis = np.asarray(wind_axis, dtype=float) / np.linalg.norm(wind_axis)
    link_axis_world = rotation[:, 2]
    radius = diameter / 2.0
    dot = float(np.clip(link_axis_world @ axis, -1.0, 1.0))
    return float(diameter * length * np.sqrt(max(0.0, 1.0 - dot * dot)) + np.pi * radius * radius * abs(dot))


def body_com_velocity(model: mujoco.MjModel, data: mujoco.MjData, body_id: int) -> np.ndarray:
    jacp = np.zeros((3, model.nv), dtype=float)
    jacr = np.zeros((3, model.nv), dtype=float)
    mujoco.mj_jacBodyCom(model, data, jacp, jacr, body_id)
    return jacp @ data.qvel


def quadratic_wind_force(wind_x: float, body_velocity_x: float, rho: float, cd: float, area: float) -> float:
    relative = float(wind_x) - float(body_velocity_x)
    return float(0.5 * rho * cd * area * abs(relative) * relative)


def compute_body_wind_forces(model, data, config: AerodynamicConfig, model_config, wind_x: float) -> dict[str, float]:
    wind_axis = config.wind_axis
    results: dict[str, float] = {}
    body_specs = [("quadrotor", config.airframe_cd, "airframe", None), *[(f"link_{i}", config.link_cd, "link", i) for i in range(1, model_config.n_links + 1)], ("cutter", config.cutter_cd, "cutter", None)]
    for name, cd, kind, index in body_specs:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise KeyError(name)
        velocity = body_com_velocity(model, data, body_id)
        rotation = np.asarray(data.xmat[body_id], dtype=float).reshape(3, 3)
        if kind == "airframe":
            area = box_projected_area(config.airframe_dimensions, rotation, wind_axis)
        elif kind == "cutter":
            area = box_projected_area(config.cutter_dimensions, rotation, wind_axis)
        else:
            area = link_projected_area(model_config.link_length, config.link_diameter, rotation, wind_axis)
        force = quadratic_wind_force(wind_x, velocity[0], config.air_density, cd, area)
        data.xfrc_applied[body_id, 0] += force
        results[f"{name}_x"] = force
    results["total_x"] = float(sum(results.values()))
    return results


def compute_body_wind_forces_world(
    model, data, config: AerodynamicConfig, model_config, wind_velocity_world: np.ndarray,
) -> dict[str, np.ndarray | float]:
    """Apply the frozen distributed drag model along any world-frame direction.

    This is the coordinate-general form of :func:`compute_body_wind_forces`:
    the same bodies, coefficients, projected-area formulas, COM application
    points, and no-aerodynamic-torque assumption are preserved.
    """
    wind = np.asarray(wind_velocity_world, dtype=float).reshape(3)
    if not np.isfinite(wind).all():
        raise ValueError("wind_velocity_world must be finite")
    speed = float(np.linalg.norm(wind))
    if speed <= 1e-15:
        axis = np.array([1.0, 0.0, 0.0])
    else:
        axis = wind / speed
    results: dict[str, np.ndarray | float] = {}
    body_specs = [
        ("quadrotor", config.airframe_cd, "airframe"),
        *[(f"link_{i}", config.link_cd, "link") for i in range(1, model_config.n_links + 1)],
        ("cutter", config.cutter_cd, "cutter"),
    ]
    total = np.zeros(3, dtype=float)
    for name, cd, kind in body_specs:
        body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))
        if body_id < 0:
            raise KeyError(name)
        velocity = body_com_velocity(model, data, body_id)
        rotation = np.asarray(data.xmat[body_id], dtype=float).reshape(3, 3)
        if kind == "airframe":
            area = box_projected_area(config.airframe_dimensions, rotation, axis)
        elif kind == "cutter":
            area = box_projected_area(config.cutter_dimensions, rotation, axis)
        else:
            area = link_projected_area(model_config.link_length, config.link_diameter, rotation, axis)
        relative_axis_speed = float(axis @ (wind - velocity))
        scalar = 0.5 * config.air_density * cd * area * abs(relative_axis_speed) * relative_axis_speed
        force = scalar * axis
        data.xfrc_applied[body_id, :3] += force
        results[name] = force.copy()
        total += force
    results["total_world"] = total
    return results
