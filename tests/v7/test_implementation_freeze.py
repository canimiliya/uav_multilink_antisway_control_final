"""V7 Paper implementation and protocol freeze checks."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PAPER = ROOT / "reproducibility/v7/paper"


def read(name: str) -> dict:
    return json.loads((PAPER / name).read_text(encoding="utf-8"))


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def test_exact_unique_budget_and_no_holdout_access() -> None:
    protocol = read("development_protocol.json")
    assert protocol["unique_configuration_count"] == 96
    assert protocol["maximum_unique_configurations"] == 96
    assert len(protocol["stages"]["stage_a"]["candidates"]) == 24
    assert len(protocol["stages"]["stage_b"]["candidates"]) == 48
    assert len(protocol["stages"]["stage_c"]["candidates"]) == 24
    assert protocol["holdout_accessed"] is False


def test_frozen_implementation_hashes() -> None:
    freeze = read("implementation_freeze.json")
    assert freeze["paper_performance_executed"] is False
    assert freeze["holdout_accessed"] is False
    assert freeze["traditional_mutated"] is False
    assert freeze["satc_mutated"] is False
    for name, expected in freeze["sha256"].items():
        assert digest(ROOT / name) == expected
