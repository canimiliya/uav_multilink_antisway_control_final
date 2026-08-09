"""V3-R2 post-freeze ablation and visual evidence tests."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R2 = ROOT / "reproducibility/v3/r2"


def read(name: str) -> dict:
    return json.loads((R2 / name).read_text(encoding="utf-8"))


def test_ablation_occurs_after_committed_freeze_without_retuning() -> None:
    ablation = read("self_ablation.json")
    assert ablation["executed_after_committed_self_freeze"] is True
    assert ablation["self_freeze_commit"] == "58b6544ab6a123f08d1cf04f3b9a7b38a40647e5"
    assert ablation["no_ablation_retuning"] is True
    assert ablation["development_only"] is True
    assert ablation["holdout_executed"] is False
    assert ablation["predictive_only"]["sample_count"] == 75
    assert ablation["residual_only"]["sample_count"] == 75
    assert ablation["full"]["sample_count"] == 75


def test_full_method_is_the_only_complete_ablation_winner() -> None:
    ablation = read("self_ablation.json")
    assert ablation["full"]["success_rate"] == 1.0
    assert ablation["full"]["success_rate"] > ablation["backbone"]["success_rate"]
    assert ablation["full"]["success_rate"] > ablation["predictive_only"]["success_rate"]
    assert ablation["full"]["success_rate"] > ablation["residual_only"]["success_rate"]
    assert ablation["full"]["position_rmse_3d_m"] < ablation["backbone"]["position_rmse_3d_m"]


def test_representative_manifest_is_complete_and_byte_valid() -> None:
    manifest = read("visual_manifest.json")
    assert manifest["selected_candidate"] == "self_a_034"
    assert manifest["development_only"] is True
    assert len(manifest["traces"]) == 4
    assert len(manifest["plots"]) == 1
    for item in manifest["traces"] + manifest["plots"]:
        path = ROOT / item["path"]
        assert path.is_file()
        assert hashlib.sha256(path.read_bytes()).hexdigest() == item["sha256"]


def test_final_gate_closes_only_self_development() -> None:
    gate = read("gate.json")
    assert gate["result"] == "V3_SELF_ADVANCED_FROZEN"
    assert gate["win_level"] == "STRICT_ALL_METRIC_WIN_DEVELOPMENT"
    assert gate["ablation_executed"] is True
    assert gate["ablation_after_committed_freeze"] is True
    assert gate["advanced_paper_started"] is False
    assert gate["holdout_executed"] is False
