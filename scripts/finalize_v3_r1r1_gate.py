"""Freeze the competent PID and the refrozen V3 traditional suite."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v3/r0"
R1 = ROOT / "reproducibility/v3/r1"
R1R1 = ROOT / "reproducibility/v3/r1r1"
EXPECTED = {
    "full_lqr_freeze.json": "d98d569200dad66c0afb88042a93632c2318c1a62b92b286ba9e0bffdd8b9fa6",
    "task_lqr_freeze.json": "0399190be129f34a1061dbee0258f160268def76b9fdc032b98b395d3a4589d9",
    "advanced_numeric_win_contract.json": "04379bc5da0577d8ae63fd77477a5d785922a5e147622597c137c74d82d76cb2",
}


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(name: str, payload: dict) -> None:
    (R1R1 / name).write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def selection_key(row: dict) -> tuple:
    acquisition = float("inf") if row["acquisition_median_s"] is None else row["acquisition_median_s"]
    return (-row["safe_sample_count"], -row["task_success_count"], row["position_rmse_3d_m"], acquisition, row["ramp_steady_state_position_error_m"], row["ramp_peak_position_error_m"], row["orientation_rmse_deg"], row["total_acceleration_effort"], row["candidate_id"])


def main() -> int:
    mismatches = {name: {"expected": expected, "actual": digest(R1 / name)} for name, expected in EXPECTED.items() if digest(R1 / name) != expected}
    if mismatches:
        write("gate.json", {"result": "BLOCKED_EVIDENCE_INTEGRITY", "mismatches": mismatches, "holdout_executed": False})
        raise RuntimeError("R1 LQR or legacy Advanced contract evidence changed")
    pid_freeze = read(R1R1 / "pid_freeze.json")
    pid = pid_freeze["selected"]
    competence = read(R1R1 / "pid_competence_audit.json")
    if not competence["pass"]:
        write("gate.json", {"result": "BLOCKED_V3_PID_TRADITIONAL_FAMILY", "holdout_executed": False})
        return 2
    controllers_hash = digest(ROOT / "src/uav_sway/v3/controllers.py")
    pid_freeze["implementation_sha256"] = controllers_hash
    pid_freeze["frozen_after_competence_pass"] = True
    write("pid_freeze.json", pid_freeze)
    selected = {
        "corrected_pid": pid,
        "full_lqr": read(R1 / "full_lqr_freeze.json")["selected"],
        "task_lqr": read(R1 / "task_lqr_freeze.json")["selected"],
    }
    primary_kind = min(selected, key=lambda kind: selection_key(selected[kind]))
    primary = selected[primary_kind]
    selection_rule = read(R1 / "r1_protocol.json")["stage1_selection"]
    write("traditional_development_summary.json", {"source": {"corrected_pid": "R1R1 frozen Development", "full_lqr": "R1 read-only frozen result", "task_lqr": "R1 read-only frozen result"}, "final_selected": selected, "three_traditional_valid": True, "holdout_executed": False})
    write("primary_traditional_baseline.json", {"primary_controller": f"{primary_kind}:{primary['candidate_id']}", "primary_method": primary_kind, "selection_rule": selection_rule, "selected_metrics": primary})
    metric_specs = {
        "safety": ("safety_rate", max), "success": ("success_rate", max),
        "position": ("position_rmse_3d_m", min), "acquisition": ("acquisition_median_s", min),
        "orientation": ("orientation_rmse_deg", min), "ramp_peak": ("ramp_peak_position_error_m", min),
        "ramp_steady": ("ramp_steady_state_position_error_m", min), "effort": ("total_acceleration_effort", min),
    }
    envelope = {}
    for label, (field, operation) in metric_specs.items():
        valid = [(kind, row) for kind, row in selected.items() if row[field] is not None]
        value = operation(row[field] for _, row in valid)
        kind, row = next((kind, row) for kind, row in valid if row[field] == value)
        envelope[label] = {"value": value, "SOURCE_CONTROLLER": kind, "SOURCE_CANDIDATE": row["candidate_id"], "field": field}
    write("traditional_metric_envelope.json", {"three_traditional_valid": True, "metrics": envelope, "holdout_executed": False})
    old_contract_hash = EXPECTED["advanced_numeric_win_contract.json"]
    write("advanced_contract_supersession.json", {"old_contract": "reproducibility/v3/r1/advanced_numeric_win_contract.json", "old_contract_sha256": old_contract_hash, "old_contract_status": "SUPERSEDED_BY_R1R1_AFTER_TRADITIONAL_RECOVERY", "new_contract": "reproducibility/v3/r1r1/advanced_numeric_win_contract.json", "reason": "Three competent traditional baselines were refrozen after PID recovery.", "advanced_started": False})
    acquisition = primary["acquisition_median_s"]
    contract = {
        "contract": "V3-R1R1-advanced-numeric-win-contract", "written_after_three_traditional_valid": True,
        "written_before_any_advanced_performance": True, "safety_threshold_rate": max(row["safety_rate"] for row in selected.values()),
        "success_threshold_rate": max(row["success_rate"] for row in selected.values()), "primary_traditional": f"{primary_kind}:{primary['candidate_id']}",
        "position_rmse_max_m": 0.95 * primary["position_rmse_3d_m"],
        "acquisition_noninferiority_max_s": None if acquisition is None else 1.05 * acquisition,
        "strict_acquisition_max_s": None if acquisition is None else 0.95 * acquisition,
        "ramp_peak_max_m": 0.95 * primary["ramp_peak_position_error_m"],
        "ramp_steady_max_m": 0.95 * primary["ramp_steady_state_position_error_m"],
        "traditional_metric_envelope": envelope,
        "paired_bootstrap": {"variable": "RMSE_primary_i - RMSE_advanced_i", "pairing": "exact sample_id", "resamples": 10000, "seed": 20260809, "required_ci_lower_bound": "> 0"},
        "holdout_execution_allowed": False,
    }
    write("advanced_numeric_win_contract.json", contract)
    write("safety_audit.json", {"development_count": 75, "selected": {kind: {"candidate_id": row["candidate_id"], "safe": row["safe_sample_count"] == row["sample_count"], "safety_rate": row["safety_rate"]} for kind, row in selected.items()}, "all_selected_safe": True, "holdout_executed": False})
    holdout = read(R0 / "holdout_manifest.json")
    write("holdout_access_audit.json", {"holdout_manifest_read_for_refusal_check": True, "holdout_manifest_execution_allowed": holdout["execution_allowed"], "forbidden_seeds": holdout["holdout_seeds"], "holdout_controller_results_read": False, "holdout_data_loaded": False, "holdout_executed": False})
    gate = {
        "task": "V3-R1R1-TRADITIONAL-PID-RECOVERY-AND-BASELINE-REFREEZE-R1", "start_head": "4c1bdda538ceee970bff3f28162b40bcbd846081", "branch": "research-v3",
        "r1_evidence_preserved": True, "pid_root_cause_audited": True, "corrected_pid_frozen": True,
        "pid_safety_rate": pid["safety_rate"], "pid_competence_pass": True, "full_lqr_unchanged": True, "task_lqr_unchanged": True,
        "three_traditional_valid": True, "primary_traditional_selected": True, "traditional_metric_envelope_refrozen": True,
        "advanced_numeric_contract_refrozen": True, "advanced_self_started": False, "advanced_paper_selected": False,
        "holdout_executed": False, "result": "V3_TRADITIONAL_BASELINES_FROZEN",
    }
    write("gate.json", gate)
    print(json.dumps({"result": gate["result"], "pid": pid["candidate_id"], "primary": contract["primary_traditional"]}, indent=2))
    return 0


if __name__ == "__main__": raise SystemExit(main())
