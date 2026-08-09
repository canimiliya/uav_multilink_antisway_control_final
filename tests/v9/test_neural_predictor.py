from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from uav_sway.v9.neural_predictor import (
    CausalLiftedPredictor,
    NeuralPredictorArtifact,
)


ROOT = Path(__file__).resolve().parents[2]
PREDICTOR = ROOT / "reproducibility/v9/predictor"


def artifact() -> NeuralPredictorArtifact:
    return NeuralPredictorArtifact.load(PREDICTOR / "selected_predictor.npz")


def test_selected_artifact_is_finite_stable_and_shape_checked() -> None:
    value = artifact()
    assert value.model_id == "np_v9_14"
    assert value.A.shape[0] == value.A.shape[1]
    assert value.B.shape == (value.lifted_dimension, 6)
    assert value.C.shape == (6, value.lifted_dimension)
    assert np.max(np.abs(np.linalg.eigvals(value.A))) < 1.0
    predicted = value.predict_one(np.zeros(6), np.zeros(6))
    assert predicted.shape == (6,)
    assert np.isfinite(predicted).all()


def test_online_update_waits_for_full_paper_window() -> None:
    value = artifact()
    predictor = CausalLiftedPredictor(value)
    for index in range(40):
        predictor.observe(np.full(6, 0.001 * index), np.zeros(6))
    assert predictor.update_count == 0
    predictor.observe(np.full(6, 0.040), np.zeros(6))
    assert predictor.update_count == 1
    sequence = predictor.predict_sequence(np.zeros(6), np.zeros((4, 6)))
    assert sequence.shape == (4, 6)
    assert np.isfinite(sequence).all()


def test_predictor_gate_blocks_development_and_holdout() -> None:
    freeze = json.loads(
        (PREDICTOR / "predictor_freeze.json").read_text(encoding="utf-8")
    )
    assert freeze["result"] == "BLOCKED_V9_NEURAL_PREDICTOR_NOT_VALIDATED"
    assert freeze["validated"] is False
    assert freeze["development_used_for_selection"] is False
    assert freeze["holdout_accessed"] is False
