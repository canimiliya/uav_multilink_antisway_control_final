import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_final_gate_is_ready_without_any_holdout_or_new_research():
    gate = json.loads((ROOT / "reproducibility/native_stack/final/final_gate.json").read_text(encoding="utf-8"))
    assert gate["result"] == "P2_NATIVE_STACK_BENCHMARK_READY"
    assert gate["all_required_gates_pass"]
    assert not gate["holdout_executed"]
    assert not gate["new_controller_performance"]
    assert not gate["new_paper_selected"]
    assert not gate["satc_retuned"] and not gate["traditional_retuned"]
    assert not gate["old_holdout_accessed"]
    assert gate["gates"]["V1_V10_UNCHANGED"]
