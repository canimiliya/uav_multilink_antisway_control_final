import csv
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
FINAL = ROOT / "reproducibility/v6/final"


def read(name: str) -> dict:
    return json.loads((FINAL / name).read_text(encoding="utf-8"))


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def test_final_status_retains_v5_claim_and_closes_v6():
    gate = read("final_gate.json")
    assert gate["result"] == "V6_NO_QUALIFIED_RECENT_PAPER_BASELINE"
    assert gate["v6_complete"] is True
    assert gate["v6_permanently_closed"] is True
    assert gate["highest_project_claim"] == "V5_SELF_OVERALL_HOLDOUT_WIN"
    assert gate["qualified_paper_count"] == 0
    assert gate["v6_holdout"]["executed"] is False
    assert gate["v1_v5_unchanged"] is True


def test_publication_package_is_complete():
    package = read("publication_package.json")
    assert len(package) == 8
    for value in package.values():
        assert (ROOT / value).is_file()
    with (ROOT / package["comparison_table"]).open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == 6
    assert {row["class"] for row in rows} == {"Traditional", "Self", "Recent Paper adaptation"}


def test_claim_matrix_forbids_paper_and_strict_overclaim():
    forbidden = read("claim_matrix.json")["forbidden"]
    assert "V5_SELF_STRICT_HOLDOUT_WIN" in forbidden
    assert "Paper-Advanced > Traditional" in forbidden
    assert "V6 Holdout was executed" in forbidden


def test_final_evidence_manifest_matches():
    manifest = read("evidence_manifest.json")
    assert manifest
    for name, expected in manifest.items():
        assert sha(ROOT / name) == expected
