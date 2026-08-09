from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]


def load_builder():
    path = ROOT / "scripts/build_v3_r4_holdout_protocol.py"
    spec = importlib.util.spec_from_file_location("build_v3_r4_holdout_protocol", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_r4_manifest_expands_r0_to_exact_57_samples():
    builder = load_builder()
    source = builder.read_json(ROOT / "reproducibility/v3/r0/holdout_manifest.json")
    samples = builder.build_samples(source)
    assert len(samples) == len({row["sample_id"] for row in samples}) == 57
    assert sum(row["wind"]["kind"] == "calm" for row in samples) == 12
    assert sum(row["wind"].get("speed_m_s") == 2.0 for row in samples) == 12
    assert sum(row["wind"].get("speed_m_s") == 3.5 for row in samples) == 13
    stochastic = [row for row in samples if row["wind"]["kind"] == "stochastic"]
    assert [row["wind"]["seed"] for row in stochastic] == list(range(3000, 3020))
    assert all(row["target"]["target_id"] == f"spherical_{(row['wind']['seed'] - 3000) % 12:02d}" for row in stochastic)


def test_frozen_runner_uses_manifest_ramp_terminal_speed():
    import sys

    sys.path.insert(0, str(ROOT / "scripts"))
    from run_v3_r1_baselines import _wind_series

    development = _wind_series({"wind": {"kind": "ramp", "speed_m_s": 3.0}})
    holdout = _wind_series({"wind": {"kind": "ramp", "speed_m_s": 3.5}})
    assert np.isclose(development[-1], 3.0)
    assert np.isclose(holdout[-1], 3.5)
