"""Generate a MuJoCo model from one planar-chain YAML configuration.

The quadrotor geometry and eight site-based motor actuator organization are
adapted in memory from the read-only Udaan MJCF assets.  The cable body tree
is intentionally replaced by a generated serial chain of y-axis hinges.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

from .model_config import ModelConfig, load_model_config


JOINT_MARKER_COLORS = (
    (0.95, 0.20, 0.20, 1.0),
    (0.95, 0.75, 0.15, 1.0),
    (0.20, 0.80, 0.35, 1.0),
    (0.10, 0.75, 0.90, 1.0),
    (0.25, 0.40, 0.95, 1.0),
    (0.80, 0.25, 0.90, 1.0),
)


def _vec(values: tuple[float, ...] | list[float]) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def _add_quadrotor(worldbody: ET.Element, airframe) -> ET.Element:
    # The Udaan visual/site and actuator organization is adapted in memory;
    # this project-owned quadrotor uses the frozen M400 mass/inertia values.
    quad = ET.SubElement(worldbody, "body", {
        "name": "quadrotor", "pos": "0 0 3.2", "quat": "1 0 0 0",
    })
    ET.SubElement(quad, "joint", {"name": "quadrotor_free", "type": "free"})
    ET.SubElement(quad, "inertial", {
        "pos": "0 0 0", "mass": f"{airframe.mass_kg:.12g}",
        "diaginertia": _vec(airframe.inertia_diagonal_kg_m2),
    })
    if airframe.show_dimension_envelope:
        ET.SubElement(quad, "geom", {
            "name": "m400_dimension_envelope_debug", "type": "box",
            "size": _vec(tuple(x / 2 for x in airframe.dimensions_m)),
            "mass": "0", "contype": "0", "conaffinity": "0", "rgba": "0.15 0.15 0.15 0.08",
        })
    # Simplified visual-only body volumes; these are not official M400 geometry.
    for name, pos, size, rgba in (
        ("m400_fuselage", "0 0 0", "0.20 0.15 0.07", "0.12 0.12 0.14 1"),
        ("m400_lower_body", "0 0 -0.10", "0.14 0.11 0.05", "0.25 0.25 0.28 1"),
    ):
        ET.SubElement(quad, "geom", {
            "name": name, "type": "box", "pos": pos, "size": size,
            "mass": "0", "contype": "0", "conaffinity": "0", "rgba": rgba,
        })
    arms = (
        ("0", "0.92388 0 0 0.382683"),
        ("1", "0.382683 0 0 0.92388"),
        ("2", "-0.382683 0 0 0.92388"),
        ("3", "-0.92388 0 0 0.382683"),
    )
    rotor_xy = airframe.rotor_xy_coordinate_abs_m
    prop_pos = (
        f"{rotor_xy:.12g} {rotor_xy:.12g} 0",
        f"{-rotor_xy:.12g} {rotor_xy:.12g} 0",
        f"{-rotor_xy:.12g} {-rotor_xy:.12g} 0",
        f"{rotor_xy:.12g} {-rotor_xy:.12g} 0",
    )
    for index, (suffix, quat) in enumerate(arms):
        ET.SubElement(quad, "geom", {
            "name": f"quadrotor_rotor_arm_geom_{suffix}", "type": "box",
            "pos": "0 0 0", "quat": quat, "size": f"{rotor_xy:.12g} 0.012 0.012",
            "mass": "0", "contype": "0", "conaffinity": "0", "rgba": "0.4 0.4 0.45 1",
        })
        ET.SubElement(quad, "geom", {
            "name": f"quadrotor_rotor_prop_geom_{suffix}", "type": "cylinder",
            "size": f"{airframe.propeller_diameter_m / 2:.12g} 0.005", "pos": prop_pos[index], "mass": "0",
            "contype": "0", "conaffinity": "0", "rgba": "0.95 0.45 0.1 1",
        })
        ET.SubElement(quad, "site", {
            "name": f"site{index}", "type": "box", "pos": prop_pos[index],
            "size": "0.01 0.01 0.01", "rgba": "0 0 0 0",
        })
    for name, size in (
        ("quadrotor_thrust", "0.035 0.035 0.035"),
        ("quadrotor_Mx", "0.06 0.035 0.025"),
        ("quadrotor_My", "0.06 0.035 0.025"),
        ("quadrotor_Mz", "0.06 0.035 0.025"),
    ):
        ET.SubElement(quad, "site", {
            "name": name, "type": "box", "pos": "0 0 0", "size": size,
            "rgba": "0 0 0 0",
        })
    return quad


def _add_chain(quad: ET.Element, config: ModelConfig) -> None:
    parent = quad
    length = config.link_length
    joint_min, joint_max = config.joint_range_rad
    link_mass = config.total_link_mass_kg / config.n_links
    link_radius = 0.025
    link_i_transverse = link_mass * (length * length / 12.0 + link_radius * link_radius / 4.0)
    link_i_axial = link_mass * link_radius * link_radius / 2.0
    cutter_x, cutter_y, cutter_z = config.payload.dimensions_xyz_m
    cutter_mass = config.payload.mass_kg
    cutter_inertia = (
        cutter_mass * (cutter_y * cutter_y + cutter_z * cutter_z) / 12.0,
        cutter_mass * (cutter_x * cutter_x + cutter_z * cutter_z) / 12.0,
        cutter_mass * (cutter_x * cutter_x + cutter_y * cutter_y) / 12.0,
    )
    for index in range(1, config.n_links + 1):
        link_pos = config.airframe.suspension_mount_body_m if index == 1 else (0.0, 0.0, 0.0)
        link = ET.SubElement(parent, "body", {
            "name": f"link_{index}", "pos": _vec(link_pos), "quat": "1 0 0 0",
        })
        ET.SubElement(link, "joint", {
            "name": f"joint_{index}", "type": "hinge", "axis": _vec(config.hinge_axis),
            "limited": "true", "range": f"{joint_min:.12g} {joint_max:.12g}",
            "damping": f"{config.hinge_damping:.12g}",
            "frictionloss": f"{config.hinge_frictionloss:.12g}",
        })
        ET.SubElement(link, "geom", {
            "name": f"joint_marker_{index}", "type": "sphere", "pos": "0 0 0",
            "size": "0.045", "mass": "0", "contype": "0", "conaffinity": "0",
            "group": "2", "rgba": _vec(JOINT_MARKER_COLORS[index - 1]),
        })
        ET.SubElement(link, "inertial", {
            "pos": f"0 0 {-length / 2:.12g}", "mass": f"{link_mass:.12g}",
            "diaginertia": _vec((link_i_transverse, link_i_transverse, link_i_axial)),
        })
        ET.SubElement(link, "geom", {
            "name": f"link_{index}_geom", "type": "capsule",
            "fromto": f"0 0 0 0 0 {-length:.12g}", "size": "0.025",
            "mass": f"{link_mass:.12g}", "rgba": "0.15 0.45 0.85 1",
        })
        parent = link
        if index < config.n_links:
            parent = ET.SubElement(parent, "body", {
                "name": f"link_{index + 1}_mount", "pos": f"0 0 {-length:.12g}",
            })
    attachment = tuple(
        float(value) + (0.0, 0.0, -length)[axis]
        for axis, value in enumerate(config.payload.attachment_local_position_m)
    )
    cutter = ET.SubElement(parent, "body", {
        "name": "cutter", "pos": _vec(attachment),
    })
    ET.SubElement(cutter, "geom", {
        "name": "cutter_geom", "type": "box", "pos": _vec(config.payload.geom_center_local_position_m),
        "size": _vec(config.payload.half_extents_xyz_m), "mass": f"{config.payload.mass_kg:.12g}",
        "rgba": "0.1 0.75 0.25 1",
    })
    ET.SubElement(cutter, "inertial", {
        "pos": _vec(config.payload.geom_center_local_position_m), "mass": f"{cutter_mass:.12g}",
        "diaginertia": _vec(cutter_inertia),
    })
    ET.SubElement(cutter, "site", {
        "name": "cutter_tip", "type": "sphere", "pos": _vec(config.payload.tip_local_position_m),
        "size": "0.025", "rgba": "0.95 0.85 0.1 1",
    })


def build_planar_chain_model(config_path: str | Path, output_path: str | Path) -> Path:
    """Generate one model XML and return its absolute output path."""
    config = load_model_config(config_path)
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    root = ET.Element("mujoco", {"model": f"uav_planar_chain_{config.n_links}link"})
    root.append(ET.Comment("Quadrotor geometry/actuator organization adapted in memory from Udaan; no upstream file is modified."))
    ET.SubElement(root, "compiler", {"angle": "radian", "coordinate": "local", "inertiafromgeom": "false"})
    option = ET.SubElement(root, "option", {"timestep": "0.001", "gravity": "0 0 -9.81", "integrator": "RK4"})
    ET.SubElement(option, "flag", {"energy": "enable"})
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", {"offwidth": "1280", "offheight": "720"})
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", {"name": "sky", "type": "skybox", "builtin": "gradient", "rgb1": "0.7 0.82 0.95", "rgb2": "0.35 0.45 0.55", "width": "512", "height": "512"})
    ET.SubElement(asset, "texture", {"name": "checker", "type": "2d", "builtin": "checker", "rgb1": "0.42 0.45 0.48", "rgb2": "0.34 0.37 0.4", "width": "512", "height": "512", "mark": "cross", "markrgb": "0.65 0.65 0.65"})
    ET.SubElement(asset, "material", {"name": "ground_mat", "texture": "checker", "texrepeat": "10 10", "texuniform": "true", "reflectance": "0.15"})
    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(worldbody, "light", {"name": "key", "pos": "2 2 6", "dir": "-0.2 -0.2 -1", "directional": "true", "diffuse": "0.9 0.9 0.9"})
    ET.SubElement(worldbody, "geom", {"name": "ground", "type": "plane", "size": "0 0 1", "material": "ground_mat", "condim": "1"})
    ET.SubElement(worldbody, "camera", {
        "name": "main_camera", "mode": "targetbody", "target": "link_3",
        "pos": "4 -6 3.5", "fovy": "50",
    })
    ET.SubElement(worldbody, "camera", {
        "name": "side_camera", "mode": "fixed", "pos": "0 -6 1.7",
        "xyaxes": "1 0 0 0 0 1", "fovy": "45",
    })
    ET.SubElement(worldbody, "camera", {
        "name": "oblique_camera", "mode": "targetbody", "target": "link_3",
        "pos": "5 -3 4.2", "fovy": "50",
    })
    quad = _add_quadrotor(worldbody, config.airframe)
    _add_chain(quad, config)

    equality = ET.SubElement(root, "equality")
    ET.SubElement(equality, "weld", {"name": "passive_anchor", "body1": "quadrotor", "active": "false", "solref": "0.02 1"})
    actuator = ET.SubElement(root, "actuator")
    gears = ("0 0 1 0 0 -0.0249945776", "0 0 1 0 0 0.0249945776", "0 0 1 0 0 -0.0249945776", "0 0 1 0 0 0.0249945776")
    for index, gear in enumerate(gears):
        ET.SubElement(actuator, "motor", {"name": f"rotor_motor_{index}", "site": f"site{index}", "ctrlrange": "0 10", "gear": gear})
    for name, site, gear, limit in (
        ("thrust_motor", "quadrotor_thrust", "0 0 1 0 0 0", "0 40"),
        ("mx_motor", "quadrotor_Mx", "0 0 0 1 0 0", "-3 3"),
        ("my_motor", "quadrotor_My", "0 0 0 0 1 0", "-3 3"),
        ("mz_motor", "quadrotor_Mz", "0 0 0 0 0 1", "-3 3"),
    ):
        ET.SubElement(actuator, "motor", {"name": name, "site": site, "ctrlrange": limit, "gear": gear})

    ET.indent(root, space="  ")
    ET.ElementTree(root).write(output_path, encoding="utf-8", xml_declaration=True)
    return output_path.resolve()
