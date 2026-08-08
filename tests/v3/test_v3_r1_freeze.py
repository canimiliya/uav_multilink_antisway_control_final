"""Evidence checks for the completed V3-R1 development run."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
R1 = ROOT / "reproducibility/v3/r1"


def read(name: str) -> dict:
    return json.loads((R1 / name).read_text(encoding="utf-8"))


def test_r1_gate_records_the_hard_pid_competence_block() -> None:
    gate = read("gate.json")
    assert gate["result"] == "BLOCKED_V3_TRADITIONAL_BASELINE"
    assert gate["holdout_executed"] is False
    assert gate["advanced_self_started"] is False
    assert gate["advanced_paper_selected"] is False
    safety = read("safety_audit.json")["selected"]
    assert safety["full_lqr"]["safe"] is True
    assert safety["task_lqr"]["safe"] is True
    assert safety["pid"]["safe"] is False
    assert safety["pid"]["unsafe_sample_ids"]


def test_selected_lqr_gains_and_task_alignment_are_finite() -> None:
    for name in ("full_lqr_freeze.json", "task_lqr_freeze.json"):
        selected = read(name)["selected"]
        gain = np.asarray(read(name)["parameters"]["K"], dtype=float)
        assert gain.shape == (3, 20)
        assert np.isfinite(gain).all()
        assert selected["sample_count"] == 75
        assert selected["safety_rate"] == 1.0
        assert float(read(name)["parameters"]["spectral_radius"]) < 1.0
    metric = read("task_metric_alignment_audit.json")
    assert metric["pass"] is True
    assert np.asarray(metric["C_dir"], dtype=float).shape == (3, 20)
    assert np.asarray(metric["C_omega_perp"], dtype=float).shape == (3, 20)


def test_stage1_and_stage2_evidence_cardinality() -> None:
    expected_stage1 = {"pid_stage1.csv": 8, "full_lqr_stage1.csv": 64, "task_lqr_stage1.csv": 27}
    for name, count in expected_stage1.items():
        with (R1 / name).open("r", encoding="utf-8", newline="") as stream:
            assert len(list(csv.DictReader(stream))) == count
    with (R1 / "traditional_development_results.csv").open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 675
    assert {row["method"] for row in rows} == {"pid", "full_lqr", "task_lqr"}
    assert {row["stage2_rank"] for row in rows} == {"1", "2", "3"}


def test_advanced_contract_is_frozen_but_no_advanced_was_started() -> None:
    contract = read("advanced_numeric_win_contract.json")
    gate = read("gate.json")
    assert contract["written_before_any_advanced_performance"] is True
    assert contract["holdout_execution_allowed"] is False
    assert gate["advanced_self_started"] is False
    assert gate["advanced_paper_selected"] is False
