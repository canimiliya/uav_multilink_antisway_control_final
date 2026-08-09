import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/v6/r0"


def read(name: str) -> dict:
    return json.loads((R0 / name).read_text(encoding="utf-8"))


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def test_source_and_read_only_scope():
    contract = read("research_contract.json")
    assert contract["source_head"] == "60a6350d6875d4654ab881fdf8d96e749226133c"
    assert contract["self_search"] == "FORBIDDEN"
    assert contract["frozen"]["self_reference"] == "satc_b_027"
    assert contract["frozen"]["acceleration_limit_m_s2_per_axis"] == 2.0
    assert contract["frozen"]["slew_limit_m_s2_per_update"] == 0.25


def test_new_split_and_holdout_lock():
    dev = read("development_manifest.json")
    holdout = read("holdout_manifest.json")
    integrity = read("split_integrity.json")
    assert len(dev["samples"]) == dev["sample_count"] == 120
    assert len(holdout["samples"]) == holdout["sample_count"] == 96
    assert all(row["execution_allowed"] for row in dev["samples"])
    assert not any(row["execution_allowed"] for row in holdout["samples"])
    assert integrity["development_holdout_ids_disjoint"] is True
    assert integrity["development_holdout_targets_disjoint"] is True
    assert all(not overlap for overlap in integrity["old_holdout_id_overlap"].values())
    assert set(range(6000, 6018)) == set(integrity["development_stochastic_seeds"])
    assert set(range(7000, 7024)) == set(integrity["holdout_stochastic_seeds"])


def test_paper_and_statistics_are_preregistered():
    selection = read("paper_selection_contract.json")
    budget = read("paper_search_budget.json")
    win = read("win_contract.json")
    stats = read("statistical_protocol.json")
    assert selection["suite_size_max"] == 3
    assert selection["replacement_after_suite_freeze"] is False
    assert budget["per_paper_unique_configuration_max"] == 48
    assert win["development_qualification"]["catastrophic_pair_max"] == 0
    assert stats["resamples"] == 10000
    assert stats["bootstrap_seed"] == 20260819


def test_contract_hashes_match():
    for name, expected in read("contract_sha256.json").items():
        assert digest(R0 / name) == expected
