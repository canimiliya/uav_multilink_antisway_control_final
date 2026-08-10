from __future__ import annotations

import hashlib
import json
from pathlib import Path

from uav_sway.native_stack.case_semantics import NativeCaseResolver

ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/native_stack/r0"
R0S = ROOT / "reproducibility/native_stack/r0s"


def read(path: Path): return json.loads(path.read_text(encoding="utf-8"))
def canonical(value): return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def test_identity_manifests_and_all_resolved_cases_are_exact() -> None:
    resolver = NativeCaseResolver()
    for split, expected_count, expected_hash in (
        ("development", 200, "0f03df8fe11310f6357197a9d03b605476831c76db58f64d04e92535e6df9473"),
        ("holdout", 140, "63e6192faf992494f5a78f4c008d844564b0015fc26b43a94a0c98da659b2538"),
    ):
        identity = read(R0 / f"native_{split}_manifest.json")
        resolved = read(R0S / f"resolved_{split}_manifest.json")
        assert len(identity["cases"]) == len(resolved["cases"]) == expected_count
        assert canonical(identity["cases"]) == identity["manifest_sha256"] == expected_hash
        assert resolved["identity_manifest_hash"] == expected_hash
        assert canonical(resolved["cases"]) == resolved["resolved_manifest_hash"]
        for source, frozen in zip(identity["cases"], resolved["cases"]):
            assert frozen == resolver.resolve(source).to_dict()


def test_resolved_split_golden_and_holdout_protection() -> None:
    split = read(R0S / "resolved_split_integrity.json")
    assert split["pass"] and split["resolved_physical_fingerprints_disjoint"]
    golden = read(R0S / "golden_cases.json")
    assert golden["case_count"] == 12 and golden["strict_parity"]
    assert golden["coverage"] == {"direction_classes": ["aligned", "cross", "opposed"], "task_families": ["setpoint", "smooth_trajectory"], "wind_kinds": ["calm", "moderate", "ramp", "stochastic", "strong_sustained", "strong_transient"]}
    holdout = read(R0S / "resolved_holdout_manifest.json")
    assert not holdout["execution_allowed"] and not holdout["executed"] and holdout["authoritative_runs"] == 0


def test_evidence_contracts_and_final_gate() -> None:
    required = {
        "semantic_gap_audit.json", "resolver_contract.json", "target_generation_contract.json",
        "trajectory_contract.json", "wind_contract.json", "wind_application_contract.json",
        "direction_semantics.json", "wind_level_rationale.json", "resolved_development_manifest.json",
        "resolved_holdout_manifest.json", "resolved_split_integrity.json", "semantic_fingerprints.json",
        "authoritative_runner_audit.json", "golden_cases.json", "protected_evidence_audit.json",
        "final_gate.json", "evidence_manifest.json",
    }
    assert required <= {path.name for path in R0S.glob("*.json")}
    final = read(R0S / "final_gate.json")
    if final["project_tests"]["status"] == "PASS":
        assert final["all_required_gates_pass"]
        assert final["result"] == "P2_NATIVE_STACK_BENCHMARK_V1_1_SEMANTICS_READY"
    assert not final["controller_performance"]["executed"]
    assert not final["native_holdout"]["executed"]
