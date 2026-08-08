"""Write the R3 closed-without-development-win evidence package."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
R3 = ROOT / "reproducibility/v2/r3"
R1R1 = ROOT / "reproducibility/v2/r1r1"
START_HEAD = "2aa1d72a58fdb2bac450d1106b1e044bc258e0ee"
FREEZE_HEAD = "3fbc718477383f48a9d8e99f63050b76471c7e58"


def sha(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write(name: str, value: dict) -> None:
    (R3 / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def main() -> int:
    summary = json.loads((R3 / "development_summary.json").read_text(encoding="utf-8"))
    candidates = summary["candidates"]
    common = {
        "task": "V2-R3-OF-TSRMPC-DEVELOPMENT-AND-FREEZE-R1",
        "start_head": START_HEAD,
        "implementation_freeze_head": FREEZE_HEAD,
        "stage1_candidates": 36,
        "stage1_samples_per_candidate": 9,
        "stage1_runs": 324,
        "stage2_candidates": 6,
        "stage2_samples_per_candidate": 57,
        "stage2_runs": 342,
        "solver_tolerance_unchanged": True,
        "holdout_executed": False,
        "advanced_paper_executed": False,
    }
    write("development_protocol.json", {
        **common,
        "split": "development",
        "sample_bank": "reproducibility/v2/r1r1/development_manifest.json",
        "sample_bank_sha256": sha(R1R1 / "development_manifest.json"),
        "development_seeds": list(range(20)),
        "stage1_case_definition": ["+x/-x 0.15/0.25 calm", "+x/-x 0.15/0.25 wind3.0", "ramp 0->3.0 equilibrium hold"],
        "stage2_case_definition": "all 57 frozen development samples",
        "holdout_runner_refusal": True,
        "paper_runner_refusal": True,
        "raw_trace_policy": "none retained because no candidate became eligible; stage summaries only",
    })
    write("solver_audit.json", {
        **common,
        "solver": {"name": "OSQP", "eps_abs": 1e-5, "eps_rel": 1e-5, "max_iter": 4000, "warm_start": True, "accepted_status": "solved", "fallback": False},
        "top6": [{"candidate_id": c["candidate_id"], "outer_update_count": 57 * 241, "solver_success_count": c["solver_success_count"], "solver_success_rate": c["solver_success_rate"], "total_solve_time_p95_ms": c["total_solve_time_p95_ms"]} for c in candidates],
        "all_top6_solver_success_rate_100_percent": all(c["solver_success_rate"] == 1.0 for c in candidates),
    })
    write("safety_audit.json", {
        **common,
        "top6": [{"candidate_id": c["candidate_id"], "sample_count": c["sample_count"], "safe_sample_count": c["safe_sample_count"], "safety_rate": c["safe_sample_count"] / c["sample_count"], "limiter_mismatch_max": c["limiter_mismatch_max"]} for c in candidates],
        "all_top6_safety_rate_100_percent": all(c["safe_sample_count"] == c["sample_count"] for c in candidates),
        "unsafe_samples_retained": True,
        "raw_trace_file_count": 0,
    })
    write("self_metric_comparison.json", {
        **common,
        "selected_candidate": None,
        "development_eligibility": False,
        "final_scientific_pass": False,
        "traditional_baseline_rerun": False,
        "holdout_final_pass": False,
        "thresholds": {"safety_rate": 1.0, "solver_success_rate": 1.0, "task_success_rate_min": 0.719298, "position_rmse_3d_max": 0.108004, "acquisition_median_max_s": 2.185, "ramp_peak_max": 0.123784, "ramp_steady_max": 0.174534, "solve_p95_max_ms_exclusive": 50.0, "limiter_mismatch_max": 1e-6},
        "near_miss": "reproducibility/v2/r3/near_miss.json",
    })
    write("self_ablation.json", {**common, "completed": False, "reason": "no candidate crossed the development eligibility gate; ablation is permitted only after Self freeze", "backbone": "not_run", "rmpc_only": "not_run", "offset_only": "not_run", "full": "not_run"})
    write("gate.json", {
        **common,
        "branch": "research-v2",
        "traditional_unchanged": True,
        "r1r1_unchanged": True,
        "r2_contract_unchanged": True,
        "sample_bank_unchanged": True,
        "holdout_unchanged": True,
        "implementation_frozen_before_performance": True,
        "c_task_semantics_pass": True,
        "output_bias_model_correct": True,
        "matched_wind_dob_used": False,
        "steady_state_control_parity_pass": True,
        "backbone_parity_pass": True,
        "qp_constraint_pass": True,
        "grid_size": 36,
        "selected_candidate_exists": False,
        "development_safety_pass": True,
        "development_success_gate_pass": False,
        "development_position_gate_pass": False,
        "development_acquisition_gate_pass": False,
        "development_ramp_gate_pass": False,
        "realtime_gate_pass": True,
        "self_ablation_completed": False,
        "result": "CLOSED_WITH_NO_DEVELOPMENT_WIN_OF_TSRMPC",
    })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
