from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/v9/r0"


def read(name: str) -> dict:
    return json.loads((R0 / name).read_text(encoding="utf-8"))


def sha(name: str) -> str:
    return hashlib.sha256((R0 / name).read_bytes()).hexdigest()


def test_v9_source_and_protected_start() -> None:
    contract = read("research_contract.json")
    assert contract["source_head"] == "92188e0eee0e92ed9405af07e80839360e30a19c"
    assert contract["paper"]["upstream_commit"] == "e2ababf519e7b7cca8e23f42dcb8c34fae927037"
    assert contract["paper"]["adaptation_name"] == "JIN2025-NP-MPC-ADAPTED-5LINK"
    assert contract["common_interface"]["paper_torque_actuator_authority"] is False
    assert contract["search_limits"]["neural_predictor_models_registered"] <= 64
    assert contract["search_limits"]["np_mpc_configurations_planned"] <= 128


def test_identification_split_is_whole_trajectory_and_disjoint() -> None:
    manifest = read("identification_manifest.json")
    rows = manifest["trajectories"]
    assert len(rows) == 240
    assert len({row["trajectory_id"] for row in rows}) == 240
    assert sum(row["split"] == "TRAIN" for row in rows) == 192
    assert sum(row["split"] == "VALIDATION" for row in rows) == 48
    assert {row["controller"] for row in rows} == {"full_lqr_048"}
    assert manifest["wind_truth_used_as_training_input"] is False
    assert manifest["holdout_overlap"] is False
    assert manifest["development_overlap"] is False


def test_development_and_holdout_are_byte_identical_to_v7() -> None:
    hashes = read("contract_hashes.json")
    assert sha("development_manifest.json") == "b51d9958b706354867d593d61a93336c702561e1e3b3e9dfcd5488ced0886cc3"
    assert sha("holdout_manifest.json") == "3c903826ec0d6ba3ea0c941a07f7499f0863c38b8474a40c5acc2a1d14b87d65"
    assert hashes["development_manifest"] == sha("development_manifest.json")
    assert hashes["holdout_manifest"] == sha("holdout_manifest.json")
    assert len(read("development_manifest.json")["samples"]) == 144
    holdout = read("holdout_manifest.json")
    assert len(holdout["samples"]) == 112
    assert holdout["execution_allowed"] is False


def test_predictor_registry_is_unique_and_satc_blind() -> None:
    registry = read("predictor_registry.json")
    models = registry["models"]
    assert len(models) == 24
    assert len({model["model_id"] for model in models}) == 24
    assert registry["registered_count"] == len(models)
    assert registry["satc_performance_used_for_selection"] is False
