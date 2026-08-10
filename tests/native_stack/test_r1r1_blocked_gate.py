from __future__ import annotations

import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "reproducibility/native_stack/r1r1"


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_traditional_competence_block_and_holdout_boundary() -> None:
    gate = read(BASE / "final/final_gate.json")
    assert gate["result"] == "BLOCKED_P2_TRADITIONAL_COMPETENCE"
    assert gate["competent_traditional_count"] == 0
    assert not gate["primary_native_traditional_frozen"]
    assert not gate["satc_stack_frozen"]
    holdout = read(BASE / "final/holdout_status.json")
    assert not holdout["execution_allowed"]
    assert not holdout["executed"]
    assert holdout["authoritative_runs"] == 0
    assert not holdout["compromised"]


def test_r1r1_evidence_manifest_is_exact() -> None:
    manifest = read(BASE / "final/evidence_manifest.json")
    assert manifest["entry_count"] == len(manifest["entries"])
    for entry in manifest["entries"]:
        path = ROOT / entry["path"]
        assert path.stat().st_size == entry["bytes"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == entry["sha256"]
