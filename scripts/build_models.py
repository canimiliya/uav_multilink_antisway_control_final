"""CLI for generating the frozen 4/5/6-link XML variants."""

import argparse
import json
from pathlib import Path

import mujoco

from uav_sway.models.build_planar_chain import build_planar_chain_model
from uav_sway.models.model_config import load_model_config


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--configs", nargs="+", required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    output_dir = Path(args.output_dir)
    summaries = {}
    for config_path in args.configs:
        config = load_model_config(config_path)
        output = output_dir / f"model_{config.n_links}link.xml"
        xml_path = build_planar_chain_model(config_path, output)
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        quad_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor")
        cutter_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cutter")
        link_ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"link_{i}")
            for i in range(1, config.n_links + 1)
        ]
        quad_mass = float(model.body_mass[quad_id])
        total_link_mass = float(sum(model.body_mass[i] for i in link_ids))
        cutter_mass = float(model.body_mass[cutter_id])
        summaries[str(config.n_links)] = {
            "xml": output.as_posix(),
            "nq": int(model.nq),
            "nv": int(model.nv),
            "quadrotor_mass_kg": quad_mass,
            "total_link_mass_kg": total_link_mass,
            "single_link_mass_kg": total_link_mass / config.n_links,
            "cutter_mass_kg": cutter_mass,
            "cutter_inertia_diagonal_kg_m2": [float(x) for x in model.body_inertia[cutter_id]],
            "external_payload_mass_kg": total_link_mass + cutter_mass,
            "total_takeoff_mass_kg": quad_mass + total_link_mass + cutter_mass,
            "link_length_m": config.link_length,
            "mass_sources": {
                "aircraft": "official",
                "links": config.link_mass_source,
                "cutter": config.payload.mass_source,
            },
            "hinge_axis": list(config.hinge_axis),
            "hinge_damping": config.hinge_damping,
            "hinge_frictionloss": config.hinge_frictionloss,
            "joint_range_deg": list(config.joint_range_deg),
            "anchor_constraint": "passive_anchor (inactive in production, active in passive test)",
        }
        print(xml_path)

    if len(summaries) == 3:
        summary = {
            "simulation_assumption": True,
            "source": "DJI official specifications plus explicitly labeled engineering estimates and user-provided payload parameters",
            "aircraft_model": "DJI Matrice 400",
            "aircraft_mass_kg": summaries["5"]["quadrotor_mass_kg"],
            "aircraft_parameter_type": load_model_config(args.configs[1]).airframe.parameter_type,
            "aircraft_mass_source": "official",
            "aircraft_dimensions_m": list(load_model_config(args.configs[1]).airframe.dimensions_m),
            "aircraft_inertia_diagonal_kg_m2": list(load_model_config(args.configs[1]).airframe.inertia_diagonal_kg_m2),
            "cutter_mass_kg": summaries["5"]["cutter_mass_kg"],
            "cutter_mass_source": load_model_config(args.configs[1]).payload.mass_source,
            "cutter_dimensions_m": list(load_model_config(args.configs[1]).payload.dimensions_xyz_m),
            "cutter_geometry_source": load_model_config(args.configs[1]).payload.geometry_source,
            "cutter_inertia_diagonal_kg_m2": summaries["5"]["cutter_inertia_diagonal_kg_m2"],
            "total_link_mass_kg": summaries["5"]["total_link_mass_kg"],
            "link_mass_source": load_model_config(args.configs[1]).link_mass_source,
            "external_payload_mass_kg": summaries["5"]["external_payload_mass_kg"],
            "total_takeoff_mass_kg": summaries["5"]["total_takeoff_mass_kg"],
            "max_payload_kg": load_model_config(args.configs[1]).airframe.max_payload_kg,
            "max_takeoff_mass_kg": load_model_config(args.configs[1]).airframe.max_takeoff_mass_kg,
            "inertia_is_estimated": True,
            "link_mass_is_estimated": True,
            "cutter_geometry_is_estimated": True,
            "payload_margin_kg": load_model_config(args.configs[1]).airframe.max_payload_kg - summaries["5"]["external_payload_mass_kg"],
            "takeoff_mass_margin_kg": load_model_config(args.configs[1]).airframe.max_takeoff_mass_kg - summaries["5"]["total_takeoff_mass_kg"],
            "inertia_method": load_model_config(args.configs[1]).airframe.inertia_method,
            "inertia_source": load_model_config(args.configs[1]).airframe.inertia_source,
            "models": summaries,
        }
        summary_path = output_dir.parent / "model_summary.json"
        summary_path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        m400_path = output_dir.parent / "m400_parameter_summary.json"
        m400_path.write_text(json.dumps({key: summary[key] for key in (
            "aircraft_model", "aircraft_mass_kg", "aircraft_dimensions_m",
            "aircraft_parameter_type", "aircraft_mass_source",
            "aircraft_inertia_diagonal_kg_m2", "cutter_mass_kg",
            "cutter_mass_source", "cutter_dimensions_m", "cutter_geometry_source",
            "cutter_inertia_diagonal_kg_m2", "total_link_mass_kg", "link_mass_source",
            "total_takeoff_mass_kg", "max_payload_kg", "max_takeoff_mass_kg",
            "inertia_is_estimated", "link_mass_is_estimated", "cutter_geometry_is_estimated",
            "payload_margin_kg", "takeoff_mass_margin_kg", "inertia_method", "inertia_source",
        )}, indent=2) + "\n", encoding="utf-8")
        visual_path = output_dir.parent / "visual_geometry_summary.json"
        visual_path.write_text(json.dumps({
            "cutter_orientation": "horizontal_x_axis",
            "cutter_dimensions_xyz_m": list(load_model_config(args.configs[1]).payload.dimensions_xyz_m),
            "cutter_half_extents_xyz_m": list(load_model_config(args.configs[1]).payload.half_extents_xyz_m),
            "cutter_mass_kg": summaries["5"]["cutter_mass_kg"],
            "cutter_attachment": "top_center",
            "cutter_tip_location": "positive_x_end",
            "dimension_envelope_visible_by_default": load_model_config(args.configs[1]).airframe.show_dimension_envelope,
            "fuselage_geometry": "visual_only_simplified_box",
            "fuselage_geometry_source": load_model_config(args.configs[1]).airframe.visual_geometry_source,
            "joint_marker_count_4link": 4,
            "joint_marker_count_5link": 5,
            "joint_marker_count_6link": 6,
            "joint_markers_are_collisionless": True,
            "joint_markers_are_massless": True,
            "joint_marker_colors_rgba": [list(color) for color in __import__(
                "uav_sway.models.build_planar_chain", fromlist=["JOINT_MARKER_COLORS"]
            ).JOINT_MARKER_COLORS],
        }, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
