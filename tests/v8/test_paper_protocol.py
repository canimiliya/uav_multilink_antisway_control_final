from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PAPER = ROOT / "reproducibility/v8/paper"
R0 = ROOT / "reproducibility/v8/r0"


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_registry_is_unique_and_within_frozen_budget() -> None:
    registry = read(PAPER / "candidate_registry.json")
    candidates = registry["candidates"]
    assert registry["unique_candidates"] == 128
    assert len(candidates) == 128
    assert len({row["candidate_id"] for row in candidates}) == 128
    assert all(row["architecture"] == "XU2025-CBS-FTDO-ADAPTED-5LINK" for row in candidates)


def test_protocol_cannot_access_holdout() -> None:
    protocol = read(PAPER / "development_protocol.json")
    assert protocol["holdout_execution_allowed"] is False
    assert protocol["satc_used_for_selection"] is False
    assert protocol["max_unique_configurations"] == 128
    assert len(protocol["stages"]["stage_a"]["sample_ids"]) == 24
    assert len(protocol["stages"]["stage_b"]["candidate_ids"]) == 128
    assert digest(R0 / "holdout_manifest.json") == protocol["holdout_manifest_sha256"]


def test_protocol_binds_implementation_and_equation_mapping() -> None:
    hashes = read(PAPER / "protocol_sha256.json")
    assert hashes["implementation"] == digest(ROOT / "src/uav_sway/v8/xu2025_cbs_ftdo.py")
    assert hashes["equation_mapping"] == digest(PAPER / "equation_mapping.md")
    audit = read(PAPER / "implementation_audit.json")
    assert audit["satc_components_present"] is False
    assert audit["unfaithful_pd_plus_sign_reduction"] is False
