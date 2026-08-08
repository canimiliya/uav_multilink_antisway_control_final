from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from uav_sway.control.acceleration_limiter import AccelerationLimiter
from uav_sway.control.full_state_lqr import FullStateLQR
from uav_sway.control.position_pid import PositionPID
from uav_sway.control.task_lqr import TaskLQR
from uav_sway.disturbances.wind_profiles import generate_wind_profile, load_wind_config
from uav_sway.models.model_config import load_model_config


ROOT = Path(__file__).resolve().parents[2]
FROZEN = ROOT / "reproducibility/frozen"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_runtime_model_and_manifest_hash() -> None:
    runtime = FROZEN / "model/model_5link_controlled.xml"
    model = mujoco.MjModel.from_xml_path(str(runtime))
    assert model.nv == 11
    assert sha256(runtime) == json.loads((ROOT / "reproducibility/release_manifest.json").read_text())["runtime_model"]["sha256"]


def test_model_config_and_wind_are_loadable_and_deterministic() -> None:
    config = load_model_config(ROOT / "configs/model_5link.yaml")
    assert config.n_links == 5
    wind_config = load_wind_config(ROOT / "configs/wind_profiles.yaml")
    first = generate_wind_profile("low_frequency_random", wind_config, seed=7)
    second = generate_wind_profile("low_frequency_random", wind_config, seed=7)
    assert np.array_equal(first.time, second.time)
    assert np.array_equal(first.wind_x, second.wind_x)


def test_limits_and_controller_contracts() -> None:
    limiter = AccelerationLimiter(-2.0, 2.0, 0.25)
    assert limiter.limit(2.0) == 0.25
    assert limiter.limit(-2.0) == 0.0
    pid = PositionPID(0.5, 0.2, 0.0)
    assert np.isfinite(pid.diagnostics.ax_cmd_limited)
    gain = np.load(FROZEN / "linear_model/K.npy")
    lqr = FullStateLQR(gain)
    task = TaskLQR(gain)
    assert lqr.gain.shape == (1, 16)
    assert task.gain.shape == (1, 16)


def test_release_expected_files_and_no_large_frozen_file() -> None:
    expected = ROOT / "reproducibility/expected"
    assert {path.name for path in expected.glob("*.json")} == {"quick_expected.json", "task_lqr_expected.json", "s5b_expected.json"}
    tracked_candidates = [path for path in ROOT.rglob("*") if path.is_file() and ".git" not in path.parts]
    assert max(path.stat().st_size for path in tracked_candidates) < 10 * 1024 * 1024
