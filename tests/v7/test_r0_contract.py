"""V7-R0 contract integrity tests."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/v7/r0"


def read(name: str) -> dict:
    return json.loads((R0 / name).read_text(encoding="utf-8"))


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def test_v7_manifests_are_new_and_holdout_locked() -> None:
    development = read("development_manifest.json")
    holdout = read("holdout_manifest.json")
    assert development["sample_count"] == 144
    assert holdout["sample_count"] == 112
    assert development["execution_allowed"] is True
    assert holdout["execution_allowed"] is False
    assert {row["sample_id"] for row in development["samples"]}.isdisjoint(row["sample_id"] for row in holdout["samples"])
    assert all(row["execution_allowed"] is False for row in holdout["samples"])


def test_source_and_search_contract() -> None:
    source = read("paper_source_audit.json")
    search = read("search_contract.json")
    assert source["controller_equations_complete"] is True
    assert source["primary_source_sha256"] == "1a2236debef68536d8ba3fb1b3e995f2d831eb7db08a21316d679428d6e45546"
    assert sum(stage["unique"] for stage in search["rounds"]) == 96
    assert search["candidate_97_forbidden"] is True
    assert search["satc_not_used_for_selection"] is True


def test_contract_hashes() -> None:
    recorded = read("contract_sha256.json")
    assert recorded
    for name, expected in recorded.items():
        assert digest(R0 / name) == expected


def test_protected_source_trees_recorded() -> None:
    freeze = read("source_freeze.json")
    assert freeze["source_head"] == "6849c611c1361f5a88527753a5d1cb17dea3b76d"
    assert set(freeze["protected_trees"]) == {"v2", "v3", "v4", "v5", "v6"}
    assert freeze["v1_v6_read_only"] is True
