import csv
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
FINAL = ROOT / "reproducibility/v5/final"


def read(name): return json.loads((FINAL / name).read_text(encoding="utf-8"))
def sha(path): return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def test_final_status_is_overall_not_strict_and_v5_is_closed():
    gate = read("final_gate.json")
    assert gate["result"] == "V5_SELF_OVERALL_HOLDOUT_WIN"
    assert gate["strict_holdout_win"] is False
    assert gate["overall_holdout_win"] is True
    assert gate["v5_permanently_closed"] is True
    assert gate["failed_frozen_gate"]["gate"] == "acquisition"
    assert gate["holdout"] == {"executed": True, "sample_count": 96, "participants": 5, "authoritative_runs": 480, "retries": [], "compromised": False}
    assert gate["v1_v4_unchanged"] is True


def test_holdout_and_paper_claim_boundaries_are_exact():
    with (ROOT / "reproducibility/v5/holdout/holdout_results.csv").open(encoding="utf-8", newline="") as stream: rows = list(csv.DictReader(stream))
    claims = read("claim_matrix.json"); methods = read("method_status.json")
    assert len(rows) == 480
    assert len({row["sample_id"] for row in rows}) == 96
    assert methods["paper"]["holdout"] == "NOT_RUN_DEVELOPMENT_INELIGIBLE"
    assert "V5_SELF_STRICT_HOLDOUT_WIN" in claims["forbidden"]
    assert "Self beat Paper on Holdout" in claims["forbidden"]


def test_final_evidence_hashes_match():
    manifest = read("evidence_manifest.json")
    assert len(manifest) >= 20
    for name, expected in manifest.items(): assert sha(ROOT / name) == expected
