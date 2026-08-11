"""Audit the vendored Udaan attitude interface before any heading demo.

This is deliberately a read-only gate.  The P3-R1B card permits a demo-only
wrapper only when the first argument of ``GeometricAttitudeController.compute``
is a yaw/heading command.  The vendored API is inspected, never patched.
"""

from __future__ import annotations

import inspect
import json
from pathlib import Path

from udaan.control.quadrotor import GeometricAttitudeController


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "artifacts" / "meeting_demo" / "yaw_interface_audit.json"


def build_audit() -> dict:
    method = GeometricAttitudeController.compute
    signature = inspect.signature(method)
    parameters = list(signature.parameters.values())
    first_argument = parameters[1] if len(parameters) > 1 else None
    source = Path(inspect.getsourcefile(method) or "").resolve()
    source_text = inspect.getsource(method)
    first_argument_name = first_argument.name if first_argument is not None else None
    first_argument_annotation = (
        str(first_argument.annotation)
        if first_argument is not None and first_argument.annotation is not inspect.Signature.empty
        else None
    )
    # ``compute`` is an instance method: self is parameter 0 and the first
    # runtime argument is ``t``.  The implementation also derives attitude
    # from thrust_force and does not consume that argument as a heading.
    supported = bool(
        first_argument_name in {"yaw", "yaw_ref", "heading", "heading_ref"}
        and any(token in source_text.lower() for token in ("yaw", "heading"))
    )
    result = "PASS" if supported else "BLOCKED_YAW_INTERFACE_NOT_SUPPORTED"
    return {
        "task": "P3-R1B-XYZ-PLUS-HEADING-FOUR-TASK-MEETING-DEMO-R1",
        "result": result,
        "source_path": str(source.relative_to(ROOT)) if source.is_relative_to(ROOT) else str(source),
        "function_signature": str(signature),
        "first_argument_name": first_argument_name,
        "first_argument_annotation": first_argument_annotation,
        "first_argument_interpretation": "yaw_or_heading" if supported else "time_parameter",
        "heading_argument_consumed_by_compute": bool("yaw" in source_text.lower() or "heading" in source_text.lower()),
        "attitude_derived_from": "thrust_force" if "_cmd_accel_to_cmd_att" in source_text else "unknown",
        "supported": supported,
        "gate": "P0_UDAAN_INTERFACE",
        "controller_source_modified": False,
        "model_modified": False,
        "parameters_modified": False,
    }


def main() -> int:
    audit = build_audit()
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps(audit, indent=2))
    return 0 if audit["supported"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
