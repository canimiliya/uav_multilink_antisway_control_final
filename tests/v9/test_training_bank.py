from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
TRAINING = ROOT / "reproducibility/v9/training"


def test_training_bank_matches_freeze_and_contract() -> None:
    bank_path = TRAINING / "training_bank.npz"
    freeze = json.loads(
        (TRAINING / "training_data_freeze.json").read_text(encoding="utf-8")
    )
    digest = hashlib.sha256(bank_path.read_bytes()).hexdigest()
    assert digest == freeze["bank_sha256"]
    with np.load(bank_path) as bank:
        assert bank["state_error"].shape == (240, 241, 20)
        assert bank["zeta"].shape == (240, 241, 6)
        assert bank["chi"].shape == (240, 241, 6)
        assert bank["command"].shape == (240, 241, 3)
        assert bank["safe_step"].shape == (240, 241)
        assert np.count_nonzero(bank["split"] == "TRAIN") == 192
        assert np.count_nonzero(bank["split"] == "VALIDATION") == 48
        assert np.isfinite(bank["state_error"]).all()
        assert np.isfinite(bank["zeta"]).all()
        assert np.isfinite(bank["chi"]).all()
        assert "wind" not in bank.files
        assert "wind_truth" not in bank.files


def test_training_commands_obey_common_authority() -> None:
    with np.load(TRAINING / "training_bank.npz") as bank:
        commands = bank["command"]
        assert np.max(np.abs(commands)) <= 2.0 + 1.0e-12
        assert np.max(np.abs(np.diff(commands, axis=1))) <= 0.25 + 1.0e-12
        mask = bank["label_valid"] & bank["safe_step"]
        assert int(mask.sum()) == 54697
        assert int(mask[bank["split"] == "TRAIN"].sum()) == 43740
        assert int(mask[bank["split"] == "VALIDATION"].sum()) == 10957
