"""Freeze the P2-R1R1 Native baseline protocol before performance execution."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/native_stack/r1r1/protocol"
SOURCE_HEAD = "0498ad5336033b0c14550fe5324ed46826826c56"
DEV_IDENTITY = "0f03df8fe11310f6357197a9d03b605476831c76db58f64d04e92535e6df9473"
DEV_RESOLVED = "0034d481a1abe04a46505410a557dc24815932c8a07cf96b08156c374d1aefe9"
HOLDOUT_IDENTITY = "63e6192faf992494f5a78f4c008d844564b0015fc26b43a94a0c98da659b2538"
HOLDOUT_RESOLVED = "4a4b5d92027760e0d37176b7f768746c690a4ca53236ec60224cd423d6582df0"
RATES = [20, 50, 100, 200, 500, 1000]


def write(name: str, value: object) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def main() -> None:
    manifest = json.loads((ROOT / "reproducibility/native_stack/r0s/resolved_development_manifest.json").read_text(encoding="utf-8"))
    stage_a_ids = [case["sample_id"] for case in manifest["cases"][:32]]
    common = {
        "task": "P2-R1R1-NATIVE-TRADITIONAL-AND-SATC-BASELINE-DEVELOPMENT-FREEZE-R1",
        "source_tag": "native-stack-benchmark-v1.1", "source_head": SOURCE_HEAD,
        "case_semantics": "native-case-semantics-v1",
        "authoritative_runner": "AuthoritativeNativeCaseRunner",
        "development_identity_hash": DEV_IDENTITY, "development_resolved_hash": DEV_RESOLVED,
        "holdout_identity_hash": HOLDOUT_IDENTITY, "holdout_resolved_hash": HOLDOUT_RESOLVED,
    }
    write("research_contract.json", {**common,
        "primary_mode": "NATIVE_RATE", "equal_rate_hz": 100,
        "physical_command": "WrenchCommand[T,tau_x,tau_y,tau_z]",
        "development_cases": 200, "holdout_cases": 140,
        "forbidden": ["paper_search", "paper_download", "paper_implementation", "paper_performance", "holdout_execution", "benchmark_semantic_changes", "manifest_changes", "V1-V10_changes"],
        "paper_flags": {"new_paper_selected": False, "search": False, "download": False, "implementation": False, "performance": False},
        "holdout": {"execution_allowed": False, "executed": False, "authoritative_runs": 0, "compromised": False, "permitted_audits": ["hash", "semantic_fingerprint"]},
        "supersedes_for_performance": "f7893659567911696f7219aa06300129d763845b",
    })
    write("metric_contract.json", {"version": "p2-r1r1-metrics-v1", "aggregation": "exact 200-case Development",
        "setpoint": ["safety_rate", "success_rate", "tip_position_rmse_m", "tip_position_p90_m", "tip_position_p95_m", "orientation_rmse_rad", "acquisition_time_s", "strong_mean_m", "strong_p90_m", "catastrophic_count"],
        "smooth_trajectory": ["safety_rate", "tip_tracking_rmse_m", "tracking_p90_m", "tracking_p95_m", "orientation_tracking_rmse_rad", "tip_velocity_tracking_rmse_m_s", "endpoint_position_error_m", "endpoint_velocity_error_m_s", "strong_wind_tracking_rmse_m", "catastrophic_count"],
        "system": ["physical_effort", "thrust_saturation_rate", "torque_saturation_rate", "wrench_rate_rms", "runtime_mean_ms", "runtime_p95_ms", "runtime_p99_ms", "runtime_max_ms", "deadline_miss_rate"],
        "definitions": {"success": "safe and final tip position error <=0.50 m and final tip speed <=0.50 m/s", "acquisition": "first time after issue at which position error <=0.50 m and speed <=0.50 m/s for 0.50 continuous seconds", "catastrophic": "SafetyV2 failure or non-finite state/command"},
    })
    write("competence_contract.json", {"version": "p2-r1r1-competence-v1", "frozen_before_performance": True,
        "traditional": {"safety_rate_min": 0.98, "catastrophic_count_max": 4, "success_rate_min": 0.70, "setpoint_rmse_max_m": 1.25, "trajectory_rmse_max_m": 1.50, "deadline_miss_rate_max": 0.01},
        "satc_advanced": {"competence_required": True, "safety_not_below_primary": True, "success_not_below_primary": True, "meaningful_improvement_min_fraction": 0.05, "eligible_metrics": ["setpoint_position", "trajectory_tracking", "strong_mean", "strong_p90", "acquisition"], "no_task_family_regression_max_fraction": 0.10, "no_catastrophic_regression": True, "deadline_miss_rate_max": 0.01},
        "rationale": "Thresholds follow the frozen 2 m mission geometry, 0.5 m task tolerance, SafetyV2 envelope, and real-time component periods; no performance result was inspected."
    })
    write("rate_contract.json", {"version": "p2-r1r1-rate-v1", "supported_hz": RATES, "physics_hz": 1000,
        "native_rate": {"pid": {"outer": 100, "inner": 500}, "full_lqr": {"outer": 100, "inner": 500}, "task_lqr": {"outer": 100, "inner": 500}, "satc_native": {"outer": 100, "inner": 500, "optimizer": 100}, "legacy_adapter": {"outer": 20, "inner": 200}},
        "equal_rate_diagnostic": {"rate_hz": 100, "case_ids": stage_a_ids, "selection_authority": False, "retuning_allowed": False},
        "deadline": "component wall time strictly greater than its preregistered period", "runtime_clock": "time.perf_counter_ns"
    })
    write("search_contract.json", {"version": "p2-r1r1-search-v1", "stage_a_case_ids": stage_a_ids,
        "stage_a_selection_authority": True, "stage_b_case_ids": [case["sample_id"] for case in manifest["cases"]], "stage_b_complete_required": True,
        "stage_c": {"allowed": True, "local_refinement_only": True, "early_stop_allowed": True},
        "maximum_unique_configs": {"native_pid": 96, "native_full_lqr": 96, "native_task_lqr": 96, "satc_native": 128},
        "planned_unique_configs": {"native_pid": 12, "native_full_lqr": 8, "native_task_lqr": 8, "satc_native": 16},
        "stage_b_finalists_per_family": 2, "determinism": {"candidate_order": "registry order", "case_order": "manifest order", "parallel_reduction": "case-id sorted"}
    })
    write("selection_contract.json", {"version": "p2-r1r1-selection-v1", "incumbent_challenger_required": True,
        "family_rule": {"competence_first": True, "ordering": ["safety_rate_desc", "competence_pass_desc", "success_rate_desc", "setpoint_position_rmse_asc", "trajectory_position_rmse_asc", "strong_p90_asc", "acquisition_time_asc", "physical_effort_asc", "runtime_p95_asc", "method_id_asc"], "challenger_not_forced": True},
        "primary_traditional_rule": ["competence_pass_desc", "safety_rate_desc", "success_rate_desc", "setpoint_position_rmse_asc", "trajectory_position_rmse_asc", "strong_p90_asc", "acquisition_time_asc", "physical_effort_asc", "runtime_p95_asc", "method_id_asc"],
        "paper_may_not_change_primary": True, "envelope_metrics": ["safety", "success", "setpoint", "trajectory", "acquisition", "strong_mean", "strong_p90", "orientation", "effort", "runtime"]
    })
    write("statistical_protocol.json", {"version": "p2-r1r1-statistics-v1", "comparison": "SATC frozen stack minus Primary Native Traditional on exact paired Development cases", "resamples": 10000, "seed": 20260810, "confidence_interval": 0.95, "method": "percentile paired bootstrap", "subsets": ["overall", "setpoint", "smooth_trajectory", "strong"], "holdout_claim": False})
    write("stage_a_manifest.json", {"version": "p2-r1r1-stage-a-v1", "case_count": len(stage_a_ids), "case_ids": stage_a_ids,
        "coverage": {"task_family": ["setpoint", "smooth_trajectory"], "trajectory_type": ["step", "minimum_jerk", "approach_stop", "waypoint_3d"], "wind_kind": ["calm", "moderate", "strong_sustained", "strong_transient", "stochastic", "ramp"], "wind_direction": ["aligned", "opposed", "cross"]}, "selection_authority": True})
    hashes = {}
    for path in sorted(OUT.glob("*.json")):
        hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    write("protocol_manifest.json", {"files": hashes, "performance_executed_at_freeze": False})


if __name__ == "__main__":
    main()
