import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "reproducibility/native_stack/governance"


def load(name):
    return json.loads((OUT / name).read_text(encoding="utf-8"))


def test_final_gate_closes_only_governance_audit():
    gate = load("final_gate.json")
    assert gate["result"] == "P2_NATIVE_BENCHMARK_GOVERNANCE_AUDIT_COMPLETE"
    assert gate["OWNER_DECISION"] == "B"
    assert gate["p2_r2_authorized"] is False
    assert all(value is False for value in gate["prohibited"].values())


def test_clean_platform_branch_and_tag_match():
    freeze = load("platform_freeze.json")
    assert freeze["governance_only_scope"] is True
    assert freeze["head"] == freeze["remote_head"] == freeze["tag_target"]
    assert freeze["controller_implementation_included"] is False
    assert freeze["performance_evidence_included"] is False


def test_governance_evidence_manifest_hashes_and_sizes():
    manifest = load("evidence_manifest.json")
    assert manifest["entry_count"] == len(manifest["entries"])
    for entry in manifest["entries"]:
        path = ROOT / entry["path"]
        assert path.stat().st_size == entry["size_bytes"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == entry["sha256"]
