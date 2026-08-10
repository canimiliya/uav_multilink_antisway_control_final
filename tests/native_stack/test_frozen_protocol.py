from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/native_stack/r0"


def read(name): return json.loads((R0 / name).read_text(encoding="utf-8"))


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def test_physical_audit_and_plant_are_frozen():
    audit = read("physical_actuation_audit.json")
    assert audit["audit_complete"] and not audit["plant_changed"]
    assert audit["physics"]["rate_hz"] == 1000
    assert audit["historical_execution_actuators"] == ["thrust_motor", "mx_motor", "my_motor", "mz_motor"]
    assert hashlib.sha256((ROOT / audit["model_path"]).read_bytes()).hexdigest() == audit["model_sha256"]


def test_banks_are_new_disjoint_deterministic_and_holdout_locked():
    dev, hold, integrity = read("native_development_manifest.json"), read("native_holdout_manifest.json"), read("split_integrity.json")
    assert len(dev["cases"]) == 200 and len(hold["cases"]) == 140
    assert canonical_hash(dev["cases"]) == dev["manifest_sha256"]
    assert canonical_hash(hold["cases"]) == hold["manifest_sha256"]
    assert dev["execution_allowed"] and not hold["execution_allowed"] and not hold["executed"]
    assert integrity["all_identity_intersections_empty"] and not integrity["future_information_leakage"]
    assert {case["task_family"] for case in dev["cases"]} == {"setpoint", "smooth_trajectory"}
    assert {case["wind_kind"] for case in dev["cases"]} == {"calm", "moderate", "strong_sustained", "strong_transient", "stochastic", "ramp"}


def test_validation_evidence_passes_without_holdout():
    assert read("scheduler_validation.json")["pass"]
    assert read("physical_wrench_parity.json")["pass"]
    legacy = read("legacy_pipeline_parity.json")
    assert legacy["pass"] and not legacy["old_holdout_accessed"]
    protected = read("protected_evidence_audit.json")
    assert protected["all_unchanged"]
