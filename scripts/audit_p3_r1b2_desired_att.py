"""Read-only audit of Udaan's optional desired-attitude argument.

This audit intentionally does not import or modify the vendored package.  It
records whether the optional rotation, angular velocity, and angular
acceleration fields are actually consumed by ``compute``.
"""
from __future__ import annotations

import ast
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "third_party/udaan/udaan/control/quadrotor/geometric_attitude.py"
OUT = ROOT / "artifacts/meeting_demo/desired_att_audit.json"


def main() -> int:
    text = SOURCE.read_text(encoding="utf-8")
    tree = ast.parse(text)
    compute = next(
        node for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "compute"
    )
    signature = ast.unparse(compute.args)
    desired_arg = next(a for a in compute.args.args if a.arg == "desired_att")
    annotations = {
        arg.arg: ast.unparse(arg.annotation) if arg.annotation else None
        for arg in compute.args.args
    }
    non_none = any(
        isinstance(node, ast.Compare)
        and isinstance(node.left, ast.Name)
        and node.left.id == "desired_att"
        and any(isinstance(op, ast.IsNot) for op in node.ops)
        for node in ast.walk(compute)
    )
    # A tuple unpack such as ``_, Omegad, dOmegad = desired_att`` proves that
    # the rotation component is not consumed, even though the optional tuple
    # itself is accepted.
    rotation_consumed = False
    angular_velocity_consumed = False
    angular_acceleration_consumed = False
    for node in ast.walk(compute):
        if isinstance(node, ast.Assign) and isinstance(node.value, ast.Name) and node.value.id == "desired_att":
            for target in node.targets:
                if isinstance(target, (ast.Tuple, ast.List)) and len(target.elts) == 3:
                    rotation_consumed = not (isinstance(target.elts[0], ast.Name) and target.elts[0].id == "_")
                    angular_velocity_consumed = True
                    angular_acceleration_consumed = True
    usages = []
    for path in sorted((ROOT / "third_party/udaan").rglob("*.py")):
        source = path.read_text(encoding="utf-8", errors="replace")
        if "desired_att=" in source or "desired_att =" in source:
            usages.append(str(path.relative_to(ROOT)))
    supported = bool(non_none and rotation_consumed)
    payload = {
        "task": "P3-R1B2-DESIRED-ATTITUDE-HEADING-AUDIT-AND-THREE-TASK-MEETING-DEMO-R1",
        "source_path": str(SOURCE.relative_to(ROOT)),
        "function_signature": f"compute({signature})",
        "desired_att_type": annotations.get("desired_att"),
        "desired_rotation_type": "SO3 (tuple element 0)",
        "desired_angular_velocity_type": "TSO3 (tuple element 1)",
        "desired_angular_acceleration_type": "Vec3 (tuple element 2)",
        "desired_att_non_none_branch_found": non_none,
        "desired_rotation_consumed_by_compute": rotation_consumed,
        "desired_angular_velocity_consumed_by_compute": angular_velocity_consumed,
        "desired_angular_acceleration_consumed_by_compute": angular_acceleration_consumed,
        "example_usage_found": bool(usages),
        "example_usage_paths": usages,
        "supported": supported,
        "conclusion": (
            "desired_att tuple is accepted, but compute ignores tuple element 0 (desired rotation); "
            "heading extension is not functionally supported"
            if not supported else "desired rotation is explicitly consumed"
        ),
        "controller_source_modified": False,
        "model_modified": False,
        "parameters_modified": False,
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
