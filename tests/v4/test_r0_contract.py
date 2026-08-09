from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/v4/r0"


def read_json(name: str) -> dict:
    return json.loads((R0 / name).read_text(encoding="utf-8"))


def canonical_sha256(path: Path) -> str:
    data = path.read_bytes()
    if b"\x00" not in data:
        data = data.replace(b"\r\n", b"\n")
    return hashlib.sha256(data).hexdigest()


def test_required_v4_r0_evidence_and_gate() -> None:
    required = {
        "source_freeze.json", "v3_failure_postmortem.json", "strong_wind_diagnostic.csv",
        "failure_mechanism_report.json", "v4_method_hypothesis.json", "v4_control_contract.json",
        "development_manifest.json", "holdout_manifest.json", "split_integrity.json",
        "v4_win_contract.json", "v4_search_budget.json", "gate.json", "contract_sha256.json",
    }
    assert required <= {path.name for path in R0.iterdir()}
    gate = read_json("gate.json")
    assert gate["result"] == "V4_STRONG_WIND_RESEARCH_CONTRACT_FROZEN"
    assert gate["pass"] is True
    assert all(value is expected for name, value in gate["checks"].items() for expected in [name not in {
        "V3_HOLDOUT_REUSED_AS_V4_HOLDOUT", "NEW_SELF_PERFORMANCE_EXECUTED", "HOLDOUT_EXECUTED"
    }])
    hashes = read_json("contract_sha256.json")
    for relative, expected in hashes["files"].items():
        assert canonical_sha256(ROOT / relative) == expected
    assert canonical_sha256(ROOT / "docs/V4_RESEARCH_CONTRACT.md") == hashes["docs/V4_RESEARCH_CONTRACT.md"]


def test_v3_source_is_byte_frozen_and_reclassified_only_as_prior() -> None:
    source = read_json("source_freeze.json")
    assert source["source_head"] == "1f6ef12bc657724024f4e281c6b0534ca1d76fad"
    assert source["source_tag_type"] == "tag"
    assert source["v3_final_unchanged"] is True
    assert source["v3_holdout_is_now_v4_prior_evidence"] is True
    assert source["v3_holdout_is_v4_holdout"] is False
    for relative, expected in source["source_file_sha256"].items():
        assert canonical_sha256(ROOT / relative) == expected


def test_postmortem_exactly_reproduces_v3_strong_wind_and_ramp() -> None:
    postmortem = read_json("v3_failure_postmortem.json")
    assert postmortem["v3_failure_reproduced"] is True
    assert postmortem["constant_3p5"]["sample_count"] == 12
    assert postmortem["constant_3p5"]["self_position_rmse_mean_m"] == pytest.approx(0.7113791997210749, abs=1.0e-12)
    assert postmortem["constant_3p5"]["full_lqr_position_rmse_mean_m"] == pytest.approx(0.21230805574990605, abs=1.0e-12)
    assert postmortem["ramp_0_to_3p5"]["self_position_rmse_m"] == pytest.approx(0.028342467997933524, abs=1.0e-12)
    assert postmortem["new_self_performance_executed"] is False
    assert postmortem["v4_holdout_executed"] is False
    with (R0 / "strong_wind_diagnostic.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 36
    assert {row["controller"] for row in rows} == {"self_a_034", "full_lqr_048"}


def test_single_causal_hypothesis_and_fixed_authority() -> None:
    method = read_json("v4_method_hypothesis.json")
    assert method["selection_count"] == 1
    assert method["selected"]["name"] == "CART-OFMPC"
    assert method["selected"]["causal_information_only"] is True
    assert method["selected"]["implementation_started"] is False
    assert method["selected"]["performance_executed"] is False
    control = read_json("v4_control_contract.json")
    assert control["command_interface"] == {"frame": "world", "components": ["ax", "ay", "az"]}
    assert control["acceleration_limit_m_s2_per_axis"] == 2.0
    assert control["slew_limit_m_s2_per_update"] == 0.25
    assert control["outer_rate_hz"] == 20.0
    assert control["physical_contract_change_required"] is False


def test_development_and_holdout_are_disjoint_and_holdout_is_locked() -> None:
    development = read_json("development_manifest.json")
    holdout = read_json("holdout_manifest.json")
    split = read_json("split_integrity.json")
    assert development["sample_count"] == len(development["samples"]) == 94
    assert development["sustained_strong_wind_cases"] == 37
    assert holdout["sample_count"] == len(holdout["samples"]) == 94
    assert holdout["sustained_strong_wind_cases"] == 48
    assert holdout["target_generator"]["seed"] == 20260810
    assert holdout["random_seeds"] == list(range(4000, 4020))
    assert holdout["execution_allowed"] is False
    assert all(sample["execution_allowed"] is False for sample in holdout["samples"])
    assert split["sample_id_overlap"] == []
    assert split["target_direction_exact_overlap"] == 0
    assert split["stochastic_seed_overlap"] == []
    assert split["v3_holdout_reused_as_v4_holdout"] is False


def test_win_contract_was_preregistered_before_performance() -> None:
    win = read_json("v4_win_contract.json")
    assert win["written_before_new_self_performance"] is True
    assert win["primary_traditional"] == "full_lqr_048"
    assert win["overall"]["position_mean_improvement_vs_full_lqr_min_fraction"] == 0.05
    assert win["strong_wind"]["position_mean_improvement_vs_full_lqr_min_fraction"] == 0.05
    assert win["strong_wind"]["position_mean_improvement_vs_legacy_self_min_fraction"] == 0.30
    assert win["strong_wind"]["catastrophic_pair_count_max"] == 0
    assert win["normal_regime_nonregression"]["position_degradation_vs_legacy_self_max_fraction"] == 0.05
    assert win["paired_bootstrap"] == {
        "pairing": "exact sample_id", "resamples": 10000, "seed": 20260812, "confidence": 0.95
    }
