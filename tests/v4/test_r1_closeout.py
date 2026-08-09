import csv
import hashlib
import json
from collections import Counter
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/v4/r0"
R1 = ROOT / "reproducibility/v4/r1"


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def read_csv(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def canonical_sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def test_authoritative_run_counts_are_exact() -> None:
    assert len(read_csv(R1 / "baseline_development_results.csv")) == 376
    assert len(read_csv(R1 / "stage_a_results.csv")) == 128
    assert len(read_csv(R1 / "stage_b_results.csv")) == 1692
    assert len(read_csv(R1 / "stage_c_results.csv")) == 564


def test_each_baseline_has_every_development_sample_once() -> None:
    rows = read_csv(R1 / "baseline_development_results.csv")
    counts = Counter(row["candidate_id"] for row in rows)
    assert counts == {"hybrid_x007_y041_z041": 94, "full_lqr_048": 94, "task_lqr_009": 94, "self_a_034": 94}
    assert len({(row["candidate_id"], row["sample_id"]) for row in rows}) == 376


def test_search_budget_and_confirmation_reuse() -> None:
    history = read_json(R1 / "search_history.json")
    protocol = read_json(R1 / "development_protocol.json")
    stage_b_ids = {row["candidate_id"] for row in protocol["search"]["stage_b"]["candidates"]}
    assert history["unique_configuration_count"] == 26 <= history["maximum_unique_configurations"]
    assert len(history["stage_c_selected_candidate_ids"]) == 6
    assert set(history["stage_c_selected_candidate_ids"]) <= stage_b_ids


def test_closeout_is_no_win_without_ablation_or_freeze() -> None:
    gate = read_json(R1 / "gate.json")
    assert gate["result"] == "CLOSED_WITH_NO_V4_CART_OFMPC_DEVELOPMENT_WIN"
    assert gate["overall_gate"] is True
    assert gate["strong_wind_gate"] is False
    assert gate["normal_nonregression_gate"] is True
    assert gate["candidate_frozen"] is False
    assert gate["ablation_executed"] is False
    assert not (R1 / "cart_ofmpc_freeze.json").exists()


def test_near_miss_preserves_failed_strong_wind_gates() -> None:
    near = read_json(R1 / "near_miss.json")
    assert near["candidate_id"] == "cart_b_001"
    assert near["passed_atomic_gates"] == 12
    assert near["atomic_gate_count"] == 15
    failed = {(row["section"], row["gate"]) for row in near["failed_gates"]}
    assert failed == {("strong_wind", "success"), ("strong_wind", "p90_tail"), ("strong_wind", "catastrophic_pairs")}


def test_holdout_is_still_locked_and_unexecuted() -> None:
    audit = read_json(R1 / "holdout_access_audit.json")
    holdout = read_json(R0 / "holdout_manifest.json")
    assert audit["holdout_simulations_executed"] == 0
    assert audit["holdout_debug_samples_executed"] == 0
    assert audit["holdout_results_created"] is False
    assert all(row["execution_allowed"] is False for row in holdout["samples"])


def test_frozen_protocol_input_hashes_still_match() -> None:
    hashes = read_json(R1 / "development_protocol.json")["frozen_inputs"]
    assert hashes["development_manifest_sha256"] == canonical_sha(R0 / "development_manifest.json")
    assert hashes["holdout_manifest_sha256"] == canonical_sha(R0 / "holdout_manifest.json")
    assert hashes["win_contract_sha256"] == canonical_sha(R0 / "v4_win_contract.json")
    assert hashes["search_budget_sha256"] == canonical_sha(R0 / "v4_search_budget.json")


def test_recorded_evidence_hashes_match() -> None:
    hashes = read_json(R1 / "evidence_sha256.json")
    for name, expected in hashes.items():
        assert canonical_sha(R1 / name) == expected
