"""Non-performance tests for the V3-R0 frozen research contract."""

import hashlib
import json
from pathlib import Path

import numpy as np

from uav_sway.v3.contracts import V3AccelerationCommand, V3AccelerationLimiter


ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility" / "v3" / "r0"


def read_json(name: str) -> dict:
    return json.loads((R0 / name).read_text(encoding="utf-8"))


def test_source_and_plant_are_frozen() -> None:
    source = read_json("source_freeze.json")
    plant = read_json("plant_parity.json")
    assert source["source_tag"] == "v2-research-final-2026-08-09"
    assert source["source_commit"] == "040327551ccc4cb49ed0a4f2c15bffae0bf061f7"
    assert source["main_commit"] == source["v1_commit"] == "62769122b6b75cd124c9cabc48aee2976a159f6b"
    assert plant["plant_model_unchanged"] is True
    assert plant["model_sha256"].lower() == "19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d"


def test_command_shape_amplitude_and_slew() -> None:
    command = V3AccelerationCommand.from_array([0.1, -0.2, 0.3])
    assert command.as_array().shape == (3,)
    limiter = V3AccelerationLimiter()
    limited = limiter.limit([9.0, -9.0, 0.1]).as_array()
    assert np.all(np.abs(limited) <= 0.25 + 1e-12)
    limited = limiter.limit([2.0, -2.0, 2.0]).as_array()
    assert np.all(np.abs(limited) <= 0.5 + 1e-12)


def test_linear_model_and_task_output_contract() -> None:
    linear = read_json("linear_model_audit.json")
    task = read_json("task_output_audit.json")
    assert linear["A_shape"] == [20, 20]
    assert linear["B_shape"] == [20, 3]
    assert linear["B_rank"] == 3
    assert linear["stabilizable"] is True
    assert linear["positive_negative_symmetry"]["finite"] is True
    assert task["task_position_rank"] == 3
    assert task["finite_difference_output_parity"]["pass"] is True


def test_samples_and_holdout_isolation() -> None:
    samples = read_json("sample_bank_contract.json")
    development = read_json("development_manifest.json")
    holdout = read_json("holdout_manifest.json")
    assert samples["development_target_count"] == 18
    assert development["development_seeds"] == list(range(2000, 2020))
    assert holdout["holdout_seeds"] == list(range(3000, 3020))
    assert holdout["execution_allowed"] is False
    assert holdout["V2_HOLDOUT_REUSED"] is False
    assert holdout["V3_HOLDOUT_NEW"] is True
    assert samples["v3_holdout_direction_overlap_with_v2"] is False


def test_no_controller_performance_started() -> None:
    gate = read_json("gate.json")
    assert gate["traditional_started"] is False
    assert gate["advanced_self_started"] is False
    assert gate["advanced_paper_selected"] is False
    assert gate["performance_executed"] is False
