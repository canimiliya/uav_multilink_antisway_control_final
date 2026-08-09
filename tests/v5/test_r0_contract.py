import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/v5/r0"


def read(name: str) -> dict:
    return json.loads((R0 / name).read_text(encoding="utf-8"))


def digest(path: Path) -> str:
    data = path.read_bytes().replace(b"\r\n", b"\n")
    return hashlib.sha256(data).hexdigest()


def test_source_and_physical_contract_are_frozen():
    contract = read("research_contract.json")
    assert contract["source_head"] == "8e3d7ad08747b00d46b9bdfc3cc72451491567db"
    assert contract["frozen"]["acceleration_limit_m_s2_per_axis"] == 2.0
    assert contract["frozen"]["slew_limit_m_s2_per_update"] == 0.25
    assert contract["frozen"]["outer_rate_hz"] == 20.0
    assert contract["new_controller_performance_executed"] is False


def test_split_is_disjoint_and_holdout_locked():
    dev = read("development_manifest.json")
    holdout = read("holdout_manifest.json")
    integrity = read("split_integrity.json")
    assert dev["sample_count"] == len(dev["samples"])
    assert holdout["sample_count"] == len(holdout["samples"])
    assert all(row["execution_allowed"] for row in dev["samples"])
    assert not any(row["execution_allowed"] for row in holdout["samples"])
    assert integrity["development_and_holdout_target_ids_disjoint"] is True
    assert integrity["stochastic_seeds_disjoint"] is True
    assert integrity["holdout_is_v4_unused_holdout"] is False
    assert {"aligned", "opposed", "cross"} <= set(holdout["directional_counts"])


def test_search_statistics_and_directional_gates_are_preregistered():
    budget = read("self_search_budget.json")
    win = read("win_contract.json")
    stats = read("statistical_protocol.json")
    assert budget["maximum_unique_configurations"] == 64
    assert budget["maximum_unique_planned"] <= 64
    assert win["strong_transient"]["catastrophic_pair_max"] == 0
    assert win["directional_hard_gate_each_aligned_opposed_cross"]["catastrophic_pair_max"] == 0
    assert stats["resamples"] == 10000
    assert stats["bootstrap_seed"] == 20260816


def test_contract_hash_manifest_matches():
    hashes = read("contract_sha256.json")
    assert hashes
    for name, expected in hashes.items():
        assert digest(R0 / name) == expected
