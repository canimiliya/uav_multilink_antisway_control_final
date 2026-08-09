import csv
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/v4/r0"
R1 = ROOT / "reproducibility/v4/r1"
FINAL = ROOT / "reproducibility/v4/final"


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def canonical_sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def test_required_final_evidence_exists_and_gate_closes_v4() -> None:
    required = {
        "final_gate.json", "final_method_status.json", "final_metric_summary.json",
        "final_claim_matrix.json", "simultaneous_tail_audit.json",
        "directional_failure_analysis.json", "pareto_analysis.json",
        "unused_holdout_manifest.json", "v5_recommendation.json",
        "resume_claims.json", "evidence_manifest.json",
    }
    assert required <= {path.name for path in FINAL.iterdir()}
    gate = read_json(FINAL / "final_gate.json")
    assert gate["result"] == "V4_CLOSED_WITH_MECHANISM_IMPROVEMENT_BUT_NO_QUALIFIED_SELF"
    assert gate["v3_unchanged"] is True
    assert gate["v4_r0_unchanged"] is True
    assert gate["v4_r1_unchanged"] is True
    assert gate["cart_ofmpc_development_winner"] is False
    assert gate["cart_ofmpc_mechanism_improvement"] is True
    assert gate["strong_tail_failure_preserved"] is True


def test_no_performance_holdout_or_v5_execution_is_claimed() -> None:
    gate = read_json(FINAL / "final_gate.json")
    unused = read_json(FINAL / "unused_holdout_manifest.json")
    recommendation = read_json(FINAL / "v5_recommendation.json")
    assert gate["new_controller_performance_executed"] is False
    assert gate["v4_holdout_executed"] is False
    assert gate["v5_started"] is False
    assert unused["v4_holdout_status"] == "UNUSED"
    assert unused["sample_count"] == 94
    assert unused["all_samples_execution_allowed_false"] is True
    assert recommendation["decision"] == "RECOMMEND_NEW_RESEARCH_VERSION"
    assert recommendation["authorization"] == "RECOMMENDATION_ONLY_V5_NOT_STARTED"
    assert all(row["execution_allowed"] is False for row in read_json(R0 / "holdout_manifest.json")["samples"])


def test_failure_and_positive_results_are_both_preserved() -> None:
    method = read_json(FINAL / "final_method_status.json")
    metrics = read_json(FINAL / "final_metric_summary.json")
    assert method["status"] == "CLOSED_WITH_NO_DEVELOPMENT_WIN"
    assert method["best_near_miss"] == "cart_b_001"
    assert method["passed_gates"] == "12/15"
    assert method["development_qualification"] == "FAIL"
    assert metrics["positive_evidence"]["bootstrap_lower_bound_gt_zero"] is True
    assert metrics["positive_evidence"]["normal_nonregression_pass"] is True
    assert metrics["failure_evidence"]["catastrophic_pair_count"] == 7
    assert metrics["failure_evidence"]["strong_success_rate"] == 0.10810810810810811


def test_directional_audit_is_exact_and_cautious() -> None:
    audit = read_json(FINAL / "directional_failure_analysis.json")
    assert audit["sign_partition"]["negative_x"] == {"sample_count": 7, "catastrophic_count": 7}
    assert audit["sign_partition"]["positive_x"] == {"sample_count": 5, "catastrophic_count": 0}
    assert audit["time_to_tail_onset"]["status"] == "NOT_OBSERVABLE_FROM_FROZEN_AGGREGATES"
    assert audit["backbone_qp_vector_cancellation"]["status"] == "VECTOR_ANGLE_NOT_OBSERVABLE_FROM_FROZEN_AGGREGATES"
    with (FINAL / "simultaneous_directional_samples.csv").open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 12
    assert sum(row["catastrophic"] == "True" for row in rows) == 7


def test_pareto_audit_uses_only_comparable_full_bank_candidates() -> None:
    audit = read_json(FINAL / "pareto_analysis.json")
    assert audit["unique_configuration_count"] == 26
    assert audit["stage_a_smoke_only_count"] == 8
    assert audit["stage_b_full_bank_count"] == 18
    assert audit["cross_stage_frontier_prohibited"] is True
    assert audit["strong_gate_pass_counts_across_stage_b"]["p90_tail"] == 0
    assert audit["strong_gate_pass_counts_across_stage_b"]["catastrophic_pairs"] == 0
    assert audit["selection_changed"] is False


def test_final_manifest_hashes_match() -> None:
    manifest = read_json(FINAL / "evidence_manifest.json")
    for relative, expected in manifest["source_sha256"].items():
        assert canonical_sha(ROOT / relative) == expected
    for relative, expected in manifest["output_sha256"].items():
        assert canonical_sha(ROOT / relative) == expected
    assert manifest["analysis_mode"] == "OFFLINE_EXISTING_EVIDENCE_ONLY"


def test_r0_and_r1_evidence_hashes_remain_valid() -> None:
    r1_hashes = read_json(R1 / "evidence_sha256.json")
    for name, expected in r1_hashes.items():
        assert canonical_sha(R1 / name) == expected
