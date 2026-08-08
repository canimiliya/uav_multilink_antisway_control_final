"""Create and audit the S3 actuator-range-only runtime model."""

from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


ACTUATOR_RANGES = {
    "thrust_motor": (0.0, None),
    "mx_motor": (-25.0, 25.0),
    "my_motor": (-25.0, 25.0),
    "mz_motor": (-12.0, 12.0),
}


def sha256_file(path: str | Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _name(model, object_type, index: int) -> str:
    return str(mujoco.mj_id2name(model, object_type, index))


def _model_fingerprint(model) -> dict:
    def arr(value):
        return np.asarray(value).tolist()

    bodies = []
    for i in range(model.nbody):
        bodies.append({"name": _name(model, mujoco.mjtObj.mjOBJ_BODY, i), "mass": float(model.body_mass[i]), "inertia": arr(model.body_inertia[i])})
    joints = []
    for i in range(model.njnt):
        joints.append({"name": _name(model, mujoco.mjtObj.mjOBJ_JOINT, i), "type": int(model.jnt_type[i]), "axis": arr(model.jnt_axis[i]), "range": arr(model.jnt_range[i]), "damping": float(model.dof_damping[model.jnt_dofadr[i]]) if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE else 0.0, "frictionloss": float(model.dof_frictionloss[model.jnt_dofadr[i]]) if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_FREE else 0.0})
    geoms = []
    for i in range(model.ngeom):
        geoms.append({"name": _name(model, mujoco.mjtObj.mjOBJ_GEOM, i), "type": int(model.geom_type[i]), "size": arr(model.geom_size[i]), "pos": arr(model.geom_pos[i])})
    equality = []
    for i in range(model.neq):
        equality.append({"name": _name(model, mujoco.mjtObj.mjOBJ_EQUALITY, i), "type": int(model.eq_type[i]), "active": int(model.eq_active0[i]), "obj1": int(model.eq_obj1id[i]), "obj2": int(model.eq_obj2id[i])})
    actuators = []
    for i in range(model.nu):
        actuators.append({"name": _name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i), "ctrlrange": arr(model.actuator_ctrlrange[i]), "gear": arr(model.actuator_gear[i])})
    return {"timestep": float(model.opt.timestep), "gravity": arr(model.opt.gravity), "nq": int(model.nq), "nv": int(model.nv), "bodies": bodies, "joints": joints, "geoms": geoms, "equality": equality, "actuators": actuators}


def create_runtime_model(source_xml: str | Path, output_xml: str | Path, diff_json: str | Path) -> dict:
    source_xml, output_xml, diff_json = Path(source_xml), Path(output_xml), Path(diff_json)
    source = mujoco.MjModel.from_xml_path(str(source_xml))
    total_mass = float(np.sum(source.body_mass))
    thrust_max = 2.2 * total_mass * 9.81
    ranges = {name: list(values) for name, values in ACTUATOR_RANGES.items()}
    ranges["thrust_motor"] = [0.0, thrust_max]

    tree = ET.parse(source_xml)
    root = tree.getroot()
    actuator_elements = {element.attrib["name"]: element for element in root.findall("./actuator/*") if "name" in element.attrib}
    for name, values in ranges.items():
        if name not in actuator_elements:
            raise KeyError(name)
        actuator_elements[name].set("ctrlrange", f"{values[0]:.12g} {values[1]:.12g}")
    output_xml.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space="  ")
    tree.write(output_xml, encoding="utf-8", xml_declaration=True)
    runtime = mujoco.MjModel.from_xml_path(str(output_xml))
    source_fp = _model_fingerprint(source)
    runtime_fp = _model_fingerprint(runtime)
    source_by_name = {a["name"]: a["ctrlrange"] for a in source_fp["actuators"]}
    runtime_by_name = {a["name"]: a["ctrlrange"] for a in runtime_fp["actuators"]}
    physics_keys = ["timestep", "gravity", "nq", "nv", "bodies", "joints", "geoms", "equality"]
    physics_equal = all(source_fp[key] == runtime_fp[key] for key in physics_keys)
    actuator_changes_only = all(
        (runtime_by_name[name] != source_by_name[name]) == (name in ranges)
        and (runtime_by_name[name] == ranges[name] if name in ranges else runtime_by_name[name] == source_by_name[name])
        for name in source_by_name
    )
    diff = {
        "source_xml": str(source_xml), "runtime_xml": str(output_xml),
        "source_sha256": sha256_file(source_xml), "runtime_sha256": sha256_file(output_xml),
        "total_mass_kg": total_mass, "max_total_thrust_ratio": 2.2, "computed_max_thrust_N": thrust_max,
        "source_fingerprint": source_fp, "runtime_fingerprint": runtime_fp,
        "physics_fingerprint_equal": physics_equal, "actuator_changes_only": actuator_changes_only,
        "changed_actuator_ranges": ranges,
    }
    diff_json.parent.mkdir(parents=True, exist_ok=True)
    diff_json.write_text(json.dumps(diff, indent=2) + "\n", encoding="utf-8", newline="\n")
    if not physics_equal or not actuator_changes_only:
        raise AssertionError("runtime model changed more than the four direct wrench actuator ranges")
    return diff
