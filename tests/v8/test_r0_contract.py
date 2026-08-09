from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/v8/r0"


def read(name: str) -> dict:
    return json.loads((R0 / name).read_text(encoding="utf-8"))


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_source_selection_precedes_performance() -> None:
    source = read("source_audit.json")
    assert source["priority_a"]["complete_primary_equations_obtained"] is False
    assert source["fallback_b"]["complete_primary_equations_obtained"] is False
    assert source["fallback_c"]["complete_primary_equations_obtained"] is True
    assert source["selected_source"] == "XU2025-CBS-FTDO-ADAPTED-5LINK"
    assert source["performance_accessed"] is False


def test_modal_reduction_is_plant_only_and_has_parity() -> None:
    audit = read("five_link_modal_audit.json")
    selection = read("modal_selection_contract.json")
    reduction = read("double_pendulum_identification.json")
    assert audit["passive_mode_count"] == 5
    assert len(audit["modes"]) == 5
    assert selection["selected_mode_ids"] == [1, 2]
    assert selection["performance_accessed"] is False
    assert reduction["representation"] == "DIRECT_TWO_MODE_MODAL_COORDINATES"
    assert reduction["controller_performance_used"] is False
    assert reduction["parity"]["pass"] is True


def test_v7_splits_are_byte_identical_and_holdout_locked() -> None:
    contract = read("research_contract.json")
    v7_development = ROOT / "reproducibility/v7/r0/development_manifest.json"
    v7_holdout = ROOT / "reproducibility/v7/r0/holdout_manifest.json"
    v8_development = R0 / "development_manifest.json"
    v8_holdout = R0 / "holdout_manifest.json"
    assert v7_development.read_bytes() == v8_development.read_bytes()
    assert v7_holdout.read_bytes() == v8_holdout.read_bytes()
    assert digest(v8_development) == contract["development"]["sha256"]
    assert digest(v8_holdout) == contract["holdout"]["sha256"]
    assert contract["holdout"]["execution_allowed"] is False
    assert all(sample["execution_allowed"] is False for sample in json.loads(v8_holdout.read_text())["samples"])


def test_common_authority_and_search_budget_are_frozen() -> None:
    contract = read("research_contract.json")
    authority = contract["control_authority"]
    assert authority["output"] == "world-frame [ax, ay, az]"
    assert authority["absolute_axis_limit_m_s2"] == 2.0
    assert authority["slew_axis_limit_m_s2_per_update"] == 0.25
    assert authority["outer_rate_hz"] == 20.0
    assert contract["search"]["max_unique_configurations"] == 128
    assert contract["comparators"]["parameters_frozen"] is True
