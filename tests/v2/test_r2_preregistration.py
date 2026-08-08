"""Structural and numerical-only checks for the V2-R2 preregistration gate."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from uav_sway.control.task_lqr import build_task_lqr


ROOT = Path(__file__).resolve().parents[2]
R2 = ROOT / "reproducibility/v2/r2"
R1R1 = ROOT / "reproducibility/v2/r1r1"
MODEL = ROOT / "reproducibility/frozen/linear_model"


def _json(name: str) -> dict:
    return json.loads((R2 / name).read_text(encoding="utf-8"))


def test_r1r1_is_unchanged_and_holdout_is_forbidden() -> None:
    gate = _json("gate.json")
    assert gate["traditional_baselines_modified"] is False
    assert gate["sample_bank_modified"] is False
    assert gate["holdout_modified"] is False
    holdout = json.loads((R1R1 / "holdout_manifest.json").read_text(encoding="utf-8"))
    assert holdout["execution_allowed"] is False
    assert _json("advanced_preregistration.json")["performance_execution"]["holdout"] is False


def test_self_grid_is_exactly_frozen() -> None:
    grid = _json("self_parameter_grid.json")
    assert grid["candidate_count"] == 36
    assert grid["maximum_candidate_count"] == 36
    assert grid["parameters"]["horizon_steps"] == [20]


def test_self_model_dimensions_reachability_and_backbone_stability() -> None:
    A = np.load(MODEL / "A.npy")
    B = np.load(MODEL / "B.npy")
    C = np.load(MODEL / "C_tip.npy")
    P = np.load(MODEL / "P.npy")
    assert A.shape == (16, 16) and B.shape == (16, 1) and C.shape == (1, 16)
    assert np.isfinite(A).all() and np.isfinite(B).all() and np.isfinite(C).all()
    ctrb = np.hstack([np.linalg.matrix_power(A, i) @ B for i in range(16)])
    assert np.linalg.matrix_rank(ctrb) == _json("self_model_audit.json")["checks"]["full_state_controllability_rank"]
    assert np.linalg.matrix_rank(np.vstack([C @ np.linalg.matrix_power(A, i) @ B for i in range(16)])) == 1
    task = build_task_lqr(A, B, np.load(ROOT / "reproducibility/frozen/task_lqr/C_task.npy"), 20.0, 5.0, 1.0)
    assert task["spectral_radius"] < 1.0
    assert np.min(np.linalg.eigvalsh((task["P"] + task["P"].T) / 2.0)) > 0.0
    assert np.min(np.linalg.eigvalsh((P + P.T) / 2.0)) > 0.0


def test_output_disturbance_contract_forbids_matched_wind() -> None:
    contract = _json("self_method_contract.json")
    assert contract["disturbance_model"]["type"] == "output disturbance"
    assert "matched-wind DOB" in contract["forbidden"]
    assert contract["residual_mpc"]["future_wind_truth"] is False
    assert contract["residual_mpc"]["future_target_truth"] is False


def test_paper_matrix_has_required_candidates_and_selected_adaptation_boundary() -> None:
    required = {"title", "year", "venue/status", "plant_model", "payload_model", "control_method", "required_state", "control_output", "stability_claim", "experiment_type", "code_available", "five_link_adaptation_cost", "core_method_preserved_after_adaptation", "selection_reason"}
    with (R2 / "paper_candidate_matrix.csv").open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) >= 5
    assert required.issubset(rows[0])
    selected = _json("paper_selection.json")["selected_paper"]
    assert selected["adaptation_name"] == "LV2026-CASCADE-ADAPTED"
    assert selected["exact_reproduction_claim"] is False
    assert selected["adaptation_core_preserved"] is True


def test_development_order_is_frozen() -> None:
    order = _json("development_order_contract.json")
    assert [item["stage"] for item in order["sequence"]] == ["V2-R3", "V2-R4", "V2-R5"]
    assert order["cross_method_retuning_forbidden"] is True
