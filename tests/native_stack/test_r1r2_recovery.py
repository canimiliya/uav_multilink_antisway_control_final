from __future__ import annotations

import json
import hashlib
from pathlib import Path

import numpy as np

from uav_sway.native_stack.r1r2_controllers import R1R2FullLQR, R1R2NativePID, R1R2TaskLQR


ROOT = Path(__file__).resolve().parents[2]
PROTOCOL = ROOT / "reproducibility/native_stack/r1r2/protocol"


def registry(family: str):
    return json.loads((PROTOCOL / f"{family}_candidate_registry.json").read_text(encoding="utf-8"))


def test_frozen_initial_registry_counts_and_rates():
    for family in ("native_pid", "native_full_lqr", "native_task_lqr"):
        values = registry(family)
        assert len(values) == 48
        assert len({row["method_id"] for row in values}) == 48
        assert {row["outer_rate_hz"] for row in values} == {20, 50, 100, 200, 500, 1000}


def test_pid_construction_is_finite():
    spec = registry("native_pid")[0]
    controller = R1R2NativePID(spec["parameters"], spec["method_id"], spec["outer_rate_hz"])
    assert np.isfinite(controller.physical_command().as_array()).all()


def test_lqi_models_are_fifteen_state_and_stabilizable():
    for family, cls in (("native_full_lqr", R1R2FullLQR), ("native_task_lqr", R1R2TaskLQR)):
        spec = registry(family)[0]
        controller = cls(spec["parameters"], spec["method_id"], spec["outer_rate_hz"])
        assert controller.audit["state_dimension"] == 15
        assert controller.audit["controllability_rank"] == 15
        assert controller.audit["stabilizable"] is True


def test_competence_threshold_is_unchanged():
    reference = json.loads((PROTOCOL / "competence_reference.json").read_text(encoding="utf-8"))["contract"]["traditional"]
    source = json.loads((ROOT / "reproducibility/native_stack/r1r1/protocol/competence_contract.json").read_text(encoding="utf-8"))["traditional"]
    assert reference == source


def test_failed_recovery_gate_stops_satc_holdout_and_paper():
    gate = json.loads((ROOT / "reproducibility/native_stack/r1r2/final/final_gate.json").read_text(encoding="utf-8"))
    assert gate["result"] == "P2_NATIVE_TRADITIONAL_RECOVERY_FAILED"
    assert gate["competent_traditional_count"] == 0
    assert gate["satc_search_executed"] is False
    assert gate["native_holdout_executed"] is False
    assert gate["p2_r2_authorized"] is False
    assert gate["paper_search"] is False


def test_final_evidence_manifest_hashes_and_sizes():
    manifest = json.loads((ROOT / "reproducibility/native_stack/r1r2/final/evidence_manifest.json").read_text(encoding="utf-8"))
    assert manifest["entry_count"] == len(manifest["entries"])
    for entry in manifest["entries"]:
        path = ROOT / entry["path"]
        assert path.stat().st_size == entry["size_bytes"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == entry["sha256"]
