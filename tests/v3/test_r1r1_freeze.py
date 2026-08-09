"""Final V3-R1R1 evidence and isolation checks."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
R1 = ROOT / "reproducibility/v3/r1"
R1R1 = ROOT / "reproducibility/v3/r1r1"


def read(name: str) -> dict:
    return json.loads((R1R1 / name).read_text(encoding="utf-8"))


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_pid_passes_every_frozen_competence_threshold() -> None:
    audit = read("pid_competence_audit.json")
    assert audit["pass"] is True
    assert audit["observed"]["safety_rate"] == 1.0
    assert audit["observed"]["calm_axis_acquired_count"] >= 5
    assert audit["observed"]["success_rate"] >= 0.5
    assert audit["observed"]["position_rmse_3d_m"] <= 0.25


def test_r1_lqr_artifacts_are_byte_exact_and_old_pid_evidence_remains() -> None:
    assert sha(R1 / "full_lqr_freeze.json") == "d98d569200dad66c0afb88042a93632c2318c1a62b92b286ba9e0bffdd8b9fa6"
    assert sha(R1 / "task_lqr_freeze.json") == "0399190be129f34a1061dbee0258f160268def76b9fdc032b98b395d3a4589d9"
    assert sha(R1 / "gate.json") == "09783df06d4378f439fceee211a032014ca041a530fdc74d34f4fef4aa151782"
    assert (R1 / "pid_freeze.json").is_file()
    assert read("r1_pid_failure_audit.json")["unsafe_cases_reproduced"] == 7


def test_final_selection_and_metric_envelope_are_deterministic() -> None:
    summary = read("traditional_development_summary.json")["final_selected"]
    def key(kind: str) -> tuple:
        row = summary[kind]
        acquisition = float("inf") if row["acquisition_median_s"] is None else row["acquisition_median_s"]
        return (-row["safe_sample_count"], -row["task_success_count"], row["position_rmse_3d_m"], acquisition, row["ramp_steady_state_position_error_m"], row["ramp_peak_position_error_m"], row["orientation_rmse_deg"], row["total_acceleration_effort"], row["candidate_id"])
    primary = min(summary, key=key)
    assert primary == read("primary_traditional_baseline.json")["primary_method"] == "full_lqr"
    envelope = read("traditional_metric_envelope.json")
    assert envelope["three_traditional_valid"] is True
    assert set(envelope["metrics"]) == {"safety", "success", "position", "acquisition", "orientation", "ramp_peak", "ramp_steady", "effort"}


def test_external_metrics_still_use_actual_cutter_tip() -> None:
    source = (ROOT / "scripts/run_v3_r1_baselines.py").read_text(encoding="utf-8")
    assert "position_error = task.tip_position_world - target" in source
    assert "state_reader.task_reader.read(model, data)" in source


def test_holdout_and_advanced_remain_unexecuted() -> None:
    gate = read("gate.json")
    audit = read("holdout_access_audit.json")
    assert gate["result"] == "V3_TRADITIONAL_BASELINES_FROZEN"
    assert gate["advanced_self_started"] is False
    assert gate["advanced_paper_selected"] is False
    assert gate["holdout_executed"] is False
    assert audit["holdout_controller_results_read"] is False
    assert audit["holdout_data_loaded"] is False


def test_advanced_contract_is_refrozen_against_primary_and_envelope() -> None:
    contract = read("advanced_numeric_win_contract.json")
    supersession = read("advanced_contract_supersession.json")
    assert contract["primary_traditional"] == "full_lqr:full_lqr_048"
    assert contract["safety_threshold_rate"] == 1.0
    assert contract["success_threshold_rate"] == 0.72
    assert contract["paired_bootstrap"] == {"variable": "RMSE_primary_i - RMSE_advanced_i", "pairing": "exact sample_id", "resamples": 10000, "seed": 20260809, "required_ci_lower_bound": "> 0"}
    assert supersession["old_contract_status"] == "SUPERSEDED_BY_R1R1_AFTER_TRADITIONAL_RECOVERY"
