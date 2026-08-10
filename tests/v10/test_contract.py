from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/v10/r0"


def read(name: str) -> dict:
    return json.loads((R0 / name).read_text(encoding="utf-8"))


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_source_and_final_route_are_frozen() -> None:
    contract = read("research_contract.json")
    assert contract["source_head"] == "46b4c31d45d427f790ef2978702552b5ee8235ab"
    assert contract["FINAL_PREREGISTERED_EXTERNAL_PAPER_ROUTE"] is True
    assert contract["NO_AUTOMATIC_V11"] is True
    assert contract["fidelity"] == "ADAPTED_NOT_EXACT_REPRODUCTION"


def test_inherited_banks_are_byte_identical() -> None:
    split = read("split_integrity.json")
    assert digest(R0 / "development_manifest.json") == split["development_sha256"]
    assert digest(R0 / "holdout_manifest.json") == split["holdout_sha256"]
    assert (R0 / "development_manifest.json").read_bytes() == (ROOT / "reproducibility/v7/r0/development_manifest.json").read_bytes()
    assert (R0 / "holdout_manifest.json").read_bytes() == (ROOT / "reproducibility/v7/r0/holdout_manifest.json").read_bytes()
    assert len(read("development_manifest.json")["samples"]) == 144
    holdout = read("holdout_manifest.json")
    assert len(holdout["samples"]) == 112
    assert holdout["execution_allowed"] is False
    assert split["V8_HOLDOUT_EXECUTED"] is False
    assert split["V9_HOLDOUT_EXECUTED"] is False


def test_search_and_gates_are_preregistered() -> None:
    search = read("search_contract.json")
    assert search["maximum_unique_configurations"] == 128
    assert search["planned_unique_configurations"] == len(search["candidates"]) == 64
    assert len({row["candidate_id"] for row in search["candidates"]}) == 64
    assert len(search["stage_a"]["sample_ids"]) == 12
    assert search["stage_b"] == {"advance_top": 16, "samples_each": 144, "selection_authority": True}
    win = read("win_contract.json")
    assert win["primary_traditional"] == "full_lqr_048"
    assert win["pairs"] == 144 and win["bootstrap_resamples"] == 10000


def test_adaptation_preserves_paper_identity_and_common_authority() -> None:
    adaptation = read("adaptation_contract.json")
    required = {
        "multivariable FxTDO",
        "fixed-time bi-homogeneous phi structure",
        "causal lumped-disturbance estimate",
        "constant disturbance injection over prediction horizon",
        "receding-horizon constrained MPC",
    }
    assert set(adaptation["preserved"]) == required
    assert "torque disturbance observer" in adaptation["omitted"]
    contract = read("research_contract.json")
    assert contract["frozen"]["command"] == "world [ax,ay,az]"
    assert contract["frozen"]["amplitude_per_axis"] == 2.0
    assert contract["frozen"]["slew_per_update"] == 0.25
