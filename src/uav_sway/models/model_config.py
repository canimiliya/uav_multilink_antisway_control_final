"""Validated YAML configuration for the generated planar chain and payload."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass(frozen=True)
class AirframeConfig:
    manufacturer: str
    model: str
    parameter_date: str
    parameter_type: str
    mass_kg: float
    max_takeoff_mass_kg: float
    max_payload_kg: float
    dimensions_m: tuple[float, float, float]
    diagonal_wheelbase_m: float
    propeller_diameter_m: float
    rotor_xy_coordinate_abs_m: float
    inertia_diagonal_kg_m2: tuple[float, float, float]
    inertia_method: str
    inertia_source: str
    inertia_is_measured: bool
    suspension_mount_body_m: tuple[float, float, float]
    suspension_mount_source: str
    show_dimension_envelope: bool
    visual_geometry_source: str
    max_wind_resistance_m_s: float
    max_pitch_angle_deg: float

    def validate(self) -> None:
        if self.mass_kg <= 0 or self.max_takeoff_mass_kg <= 0 or self.max_payload_kg <= 0:
            raise ValueError("airframe masses and limits must be positive")
        if len(self.dimensions_m) != 3 or not all(x > 0 for x in self.dimensions_m):
            raise ValueError("airframe dimensions must be three positive values")
        if self.diagonal_wheelbase_m <= 0 or self.propeller_diameter_m <= 0:
            raise ValueError("airframe geometry must be positive")
        if len(self.inertia_diagonal_kg_m2) != 3 or not all(x > 0 for x in self.inertia_diagonal_kg_m2):
            raise ValueError("airframe inertia must be three positive values")


@dataclass(frozen=True)
class PayloadConfig:
    name: str
    mass_kg: float
    mass_source: str
    shape: str
    dimensions_xyz_m: tuple[float, float, float]
    half_extents_xyz_m: tuple[float, float, float]
    long_axis: str
    tip_local_position_m: tuple[float, float, float]
    geometry_is_measured: bool
    geometry_basis: str
    geometry_source: str
    attachment_local_position_m: tuple[float, float, float]
    geom_center_local_position_m: tuple[float, float, float]

    def validate(self) -> None:
        if self.mass_kg <= 0 or self.shape != "box":
            raise ValueError("payload must be a positive-mass box")
        if not np.allclose(np.asarray(self.half_extents_xyz_m), np.asarray(self.dimensions_xyz_m) / 2):
            raise ValueError("payload half extents must equal dimensions / 2")
        if self.long_axis != "x":
            raise ValueError("the frozen demo cutter long axis is x")
        if not np.allclose(self.attachment_local_position_m, [0.0, 0.0, 0.0]):
            raise ValueError("cutter attachment must be at the top-center connection point")
        if not np.allclose(self.geom_center_local_position_m, [0.0, 0.0, -0.07]):
            raise ValueError("horizontal cutter geometry center is frozen at z=-0.07 m")
        expected_tip = [self.half_extents_xyz_m[0], 0.0, self.geom_center_local_position_m[2]]
        if not np.allclose(self.tip_local_position_m, expected_tip):
            raise ValueError("cutter_tip must be at the positive-x end of the box")


@dataclass(frozen=True)
class ModelConfig:
    n_links: int
    total_length: float
    total_link_mass_kg: float
    link_mass_source: str
    hinge_axis: tuple[float, float, float]
    hinge_damping: float
    hinge_frictionloss: float
    joint_range_deg: tuple[float, float]
    airframe: AirframeConfig
    payload: PayloadConfig

    @property
    def link_length(self) -> float:
        return self.total_length / self.n_links

    @property
    def joint_range_rad(self) -> tuple[float, float]:
        return tuple(float(np.deg2rad(x)) for x in self.joint_range_deg)

    def validate(self) -> None:
        if self.n_links not in (4, 5, 6):
            raise ValueError("n_links must be 4, 5, or 6")
        if self.total_length <= 0:
            raise ValueError("total_length must be positive")
        if self.total_link_mass_kg <= 0:
            raise ValueError("total_link_mass_kg must be positive")
        axis = np.asarray(self.hinge_axis, dtype=float)
        if not np.allclose(axis, [0.0, 1.0, 0.0]):
            raise ValueError("hinge_axis is frozen to [0, 1, 0]")
        if self.hinge_damping < 0 or self.hinge_frictionloss < 0:
            raise ValueError("joint dissipation values must be non-negative")
        if self.joint_range_deg[0] >= self.joint_range_deg[1]:
            raise ValueError("joint_range_deg must be increasing")
        self.airframe.validate()
        self.payload.validate()
        if self.total_link_mass_kg + self.payload.mass_kg > self.airframe.max_payload_kg:
            raise ValueError("external payload exceeds airframe max_payload_kg")
        if self.airframe.mass_kg + self.total_link_mass_kg + self.payload.mass_kg > self.airframe.max_takeoff_mass_kg:
            raise ValueError("total takeoff mass exceeds airframe max_takeoff_mass_kg")


def _load_airframe(path: Path) -> AirframeConfig:
    with path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    config = AirframeConfig(
        manufacturer=str(raw["manufacturer"]),
        model=str(raw["model"]),
        parameter_date=str(raw["parameter_date"]),
        parameter_type=str(raw["parameter_type"]),
        mass_kg=float(raw["mass_kg"]),
        max_takeoff_mass_kg=float(raw["max_takeoff_mass_kg"]),
        max_payload_kg=float(raw["max_payload_kg"]),
        dimensions_m=tuple(float(x) for x in raw["dimensions_m"]),
        diagonal_wheelbase_m=float(raw["diagonal_wheelbase_m"]),
        propeller_diameter_m=float(raw["propeller_diameter_m"]),
        rotor_xy_coordinate_abs_m=float(raw["rotor_xy_coordinate_abs_m"]),
        inertia_diagonal_kg_m2=tuple(float(x) for x in raw["inertia_diagonal_kg_m2"]),
        inertia_method=str(raw["inertia_method"]),
        inertia_source=str(raw["inertia_source"]),
        inertia_is_measured=bool(raw["inertia_is_measured"]),
        suspension_mount_body_m=tuple(float(x) for x in raw["suspension_mount_body_m"]),
        suspension_mount_source=str(raw["suspension_mount_source"]),
        show_dimension_envelope=bool(raw.get("show_dimension_envelope", False)),
        visual_geometry_source=str(raw.get("visual_geometry_source", "visual_only_simulation_assumption")),
        max_wind_resistance_m_s=float(raw["max_wind_resistance_m_s"]),
        max_pitch_angle_deg=float(raw["max_pitch_angle_deg"]),
    )
    config.validate()
    return config


def _load_payload(path: Path) -> PayloadConfig:
    with path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    config = PayloadConfig(
        name=str(raw["name"]),
        mass_kg=float(raw["mass_kg"]),
        mass_source=str(raw["mass_source"]),
        shape=str(raw["shape"]),
        dimensions_xyz_m=tuple(float(x) for x in raw["dimensions_xyz_m"]),
        half_extents_xyz_m=tuple(float(x) for x in raw["half_extents_xyz_m"]),
        long_axis=str(raw["long_axis"]),
        tip_local_position_m=tuple(float(x) for x in raw["tip_local_position_m"]),
        geometry_is_measured=bool(raw["geometry_is_measured"]),
        geometry_basis=str(raw["geometry_basis"]),
        geometry_source=str(raw["geometry_source"]),
        attachment_local_position_m=tuple(float(x) for x in raw["attachment_local_position_m"]),
        geom_center_local_position_m=tuple(float(x) for x in raw["geom_center_local_position_m"]),
    )
    config.validate()
    return config


def load_model_config(path: str | Path) -> ModelConfig:
    path = Path(path)
    with path.open("r", encoding="utf-8") as stream:
        raw = yaml.safe_load(stream)
    airframe = _load_airframe(path.parent / raw["airframe_config"])
    payload = _load_payload(path.parent / raw["payload_config"])
    config = ModelConfig(
        n_links=int(raw["n_links"]),
        total_length=float(raw["total_length"]),
        total_link_mass_kg=float(raw["total_link_mass_kg"]),
        link_mass_source=str(raw["link_mass_source"]),
        hinge_axis=tuple(float(x) for x in raw["hinge_axis"]),
        hinge_damping=float(raw["hinge_damping"]),
        hinge_frictionloss=float(raw["hinge_frictionloss"]),
        joint_range_deg=tuple(float(x) for x in raw["joint_range_deg"]),
        airframe=airframe,
        payload=payload,
    )
    config.validate()
    return config
