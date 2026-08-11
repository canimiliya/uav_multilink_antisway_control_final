from __future__ import annotations

import json
from pathlib import Path

from scripts.audit_p3_r1b_yaw_interface import build_audit


def test_udaan_first_compute_argument_is_not_heading() -> None:
    audit = build_audit()
    assert audit["first_argument_name"] == "t"
    assert audit["first_argument_interpretation"] == "time_parameter"
    assert audit["result"] == "BLOCKED_YAW_INTERFACE_NOT_SUPPORTED"
    assert audit["supported"] is False


def test_audit_artifact_is_machine_readable_after_gate_run() -> None:
    path = Path(__file__).resolve().parents[2] / "artifacts/meeting_demo/yaw_interface_audit.json"
    if path.exists():
        payload = json.loads(path.read_text(encoding="utf-8"))
        assert payload["result"] == "BLOCKED_YAW_INTERFACE_NOT_SUPPORTED"
