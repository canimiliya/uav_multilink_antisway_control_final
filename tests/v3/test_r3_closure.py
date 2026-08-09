import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R3 = ROOT / "reproducibility" / "v3" / "r3"


def _read(name: str) -> dict:
    return json.loads((R3 / name).read_text(encoding="utf-8"))


def test_no_win_closure_is_factual() -> None:
    gate = _read("gate.json")
    near = _read("near_miss.json")
    assert gate["result"] == "CLOSED_WITH_NO_V3_PAPER_ADVANCED_WIN"
    assert gate["win_level"] == "NO_WIN"
    assert near["gates"]["position"] is False
    assert near["gates"]["acquisition"] is False
    assert near["eligible"] is False
    assert not (R3 / "paper_freeze.json").exists()


def test_frozen_scopes_and_holdout_remain_closed() -> None:
    gate = _read("gate.json")
    audit = _read("holdout_access_audit.json")
    assert gate["traditional_unchanged"] is True
    assert gate["self_unchanged"] is True
    assert gate["advanced_contract_unchanged"] is True
    assert audit["holdout_executed"] is False
    assert audit["holdout_manifest_loaded"] is False


def test_protocol_freeze_is_ancestor() -> None:
    head = _read("gate.json")["paper_protocol_freeze_head"]
    result = subprocess.run(
        ["git", "merge-base", "--is-ancestor", head, "HEAD"],
        cwd=ROOT,
        check=False,
    )
    assert result.returncode == 0
