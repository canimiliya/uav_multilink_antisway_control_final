"""Finalize the unique V3 Holdout without changing frozen evaluation rules."""

from __future__ import annotations

import csv
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
R4 = ROOT / "reproducibility/v3/r4"
DOCS = ROOT / "docs"
CONTROLLERS = {
    "hybrid_x007_y041_z041": "corrected_pid",
    "full_lqr_048": "full_lqr",
    "task_lqr_009": "task_lqr",
    "self_a_034": "self_dr_tsrmpc",
}
PRIMARY = "full_lqr_048"
SELF = "self_a_034"


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict | list) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def load_rows(path: Path) -> list[dict]:
    bool_fields = {"safe", "task_success", "solver_valid", "limiter_parity_valid"}
    text_fields = {"sample_id", "scenario", "wind_kind", "controller", "candidate_id", "safety_failure_reasons", "qp_statuses"}
    rows = []
    with path.open(encoding="utf-8", newline="") as stream:
        for raw in csv.DictReader(stream):
            row: dict = {}
            for key, value in raw.items():
                if key in text_fields:
                    row[key] = value
                elif key in bool_fields:
                    row[key] = value.lower() == "true" if value else False
                elif value == "":
                    row[key] = None
                else:
                    row[key] = float(value)
            rows.append(row)
    return rows


def summarize(candidate_id: str, rows: list[dict]) -> dict:
    acquisition = [row["acquisition_time_s"] for row in rows if row["task_success"] and row["acquisition_time_s"] is not None]
    return {
        "candidate_id": candidate_id,
        "controller": CONTROLLERS[candidate_id],
        "sample_count": len(rows),
        "safe_sample_count": sum(row["safe"] for row in rows),
        "task_success_count": sum(row["task_success"] for row in rows),
        "safety_rate": float(np.mean([row["safe"] for row in rows])),
        "success_rate": float(np.mean([row["task_success"] for row in rows])),
        "acquisition_median_s": float(np.median(acquisition)) if acquisition else None,
        "position_rmse_3d_m": float(np.mean([row["position_rmse_3d_m"] for row in rows])),
        "orientation_rmse_deg": float(np.mean([row["orientation_rmse_deg"] for row in rows])),
        "ramp_peak_position_error_m": float(max(row["ramp_peak_position_error_m"] for row in rows)),
        "ramp_steady_state_position_error_m": float(max(row["ramp_steady_state_position_error_m"] for row in rows)),
        "total_acceleration_effort": float(np.mean([row["total_acceleration_effort"] for row in rows])),
        "solve_time_p95_ms": float(max(row["solve_time_p95_ms"] for row in rows)),
        "aggregation_semantics": {
            "position_orientation_effort": "mean over the exact 57 samples",
            "acquisition": "median over successful samples under the frozen terminal gate",
            "ramp_peak_and_steady": "maximum of the inherited per-sample t=2..8 peak and t>=8 steady metrics",
            "solve_p95": "maximum per-sample p95",
        },
    }


def fraction_better(baseline: float | None, challenger: float | None) -> float | None:
    if baseline is None or challenger is None or baseline == 0.0:
        return None
    return float((baseline - challenger) / baseline)


def comparison(baseline: dict, challenger: dict) -> dict:
    return {
        "baseline": baseline["candidate_id"],
        "challenger": challenger["candidate_id"],
        "safety_rate": {"baseline": baseline["safety_rate"], "self": challenger["safety_rate"], "delta": challenger["safety_rate"] - baseline["safety_rate"]},
        "success_rate": {"baseline": baseline["success_rate"], "self": challenger["success_rate"], "delta": challenger["success_rate"] - baseline["success_rate"]},
        "position_rmse_3d_m": {"baseline": baseline["position_rmse_3d_m"], "self": challenger["position_rmse_3d_m"], "improvement_fraction": fraction_better(baseline["position_rmse_3d_m"], challenger["position_rmse_3d_m"])},
        "acquisition_median_s": {"baseline": baseline["acquisition_median_s"], "self": challenger["acquisition_median_s"], "improvement_fraction": fraction_better(baseline["acquisition_median_s"], challenger["acquisition_median_s"])},
        "orientation_rmse_deg": {"baseline": baseline["orientation_rmse_deg"], "self": challenger["orientation_rmse_deg"], "improvement_fraction": fraction_better(baseline["orientation_rmse_deg"], challenger["orientation_rmse_deg"])},
        "ramp_peak_position_error_m": {"baseline": baseline["ramp_peak_position_error_m"], "self": challenger["ramp_peak_position_error_m"], "improvement_fraction": fraction_better(baseline["ramp_peak_position_error_m"], challenger["ramp_peak_position_error_m"])},
        "ramp_steady_state_position_error_m": {"baseline": baseline["ramp_steady_state_position_error_m"], "self": challenger["ramp_steady_state_position_error_m"], "improvement_fraction": fraction_better(baseline["ramp_steady_state_position_error_m"], challenger["ramp_steady_state_position_error_m"])},
        "total_acceleration_effort": {"baseline": baseline["total_acceleration_effort"], "self": challenger["total_acceleration_effort"], "improvement_fraction": fraction_better(baseline["total_acceleration_effort"], challenger["total_acceleration_effort"])},
    }


def main() -> int:
    state = read_json(R4 / "holdout_execution_state.json")
    if state["status"] != "COMPLETED" or state["compromised"]:
        raise RuntimeError("Holdout execution is not valid and complete")
    if sha256(R4 / "holdout_results.csv") != state["result_sha256"]:
        raise RuntimeError("Holdout result hash drift")
    manifest = read_json(R4 / "holdout_execution_manifest.json")
    expected_ids = [sample["sample_id"] for sample in manifest["samples"]]
    rows = load_rows(R4 / "holdout_results.csv")
    by_candidate = {candidate_id: [row for row in rows if row["candidate_id"] == candidate_id] for candidate_id in CONTROLLERS}
    if len(rows) != 228:
        raise RuntimeError("authoritative result count is not 228")
    for candidate_id, values in by_candidate.items():
        if [row["sample_id"] for row in values] != expected_ids:
            raise RuntimeError(f"sample pairing/order drift for {candidate_id}")
        if any(row["controller"] != CONTROLLERS[candidate_id] for row in values):
            raise RuntimeError(f"controller identity drift for {candidate_id}")

    summaries = {candidate_id: summarize(candidate_id, values) for candidate_id, values in by_candidate.items()}
    write_json(R4 / "holdout_controller_summary.json", {
        "split": "holdout",
        "sample_count_per_controller": 57,
        "authoritative_runs": 228,
        "primary_traditional": PRIMARY,
        "controllers": summaries,
        "paper": "NOT_RUN_DEVELOPMENT_INELIGIBLE",
    })

    primary = summaries[PRIMARY]
    self_summary = summaries[SELF]
    primary_cmp = comparison(primary, self_summary)
    primary_rows = by_candidate[PRIMARY]
    self_rows = by_candidate[SELF]
    common_success = [
        (p["acquisition_time_s"], s["acquisition_time_s"])
        for p, s in zip(primary_rows, self_rows, strict=True)
        if p["task_success"] and s["task_success"] and p["acquisition_time_s"] is not None and s["acquisition_time_s"] is not None
    ]
    primary_cmp["common_success_acquisition_diagnostic"] = {
        "pair_count": len(common_success),
        "median_primary_minus_self_s": float(np.median([p - s for p, s in common_success])) if common_success else None,
        "formal_gate_replaced": False,
    }
    write_json(R4 / "self_vs_primary.json", primary_cmp)
    write_json(R4 / "self_vs_all_traditionals.json", {
        candidate_id: comparison(summaries[candidate_id], self_summary)
        for candidate_id in ("hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009")
    })

    deltas = np.asarray([p["position_rmse_3d_m"] - s["position_rmse_3d_m"] for p, s in zip(primary_rows, self_rows, strict=True)], dtype=float)
    rng = np.random.Generator(np.random.PCG64(20260809))
    bootstrap_means = np.mean(rng.choice(deltas, size=(10000, len(deltas)), replace=True), axis=1)
    ci = np.percentile(bootstrap_means, [2.5, 97.5])
    bootstrap = {
        "pairing": "exact sample_id",
        "pairs": len(deltas),
        "variable": "RMSE_full_lqr_i - RMSE_self_i",
        "resamples": 10000,
        "seed": 20260809,
        "mean_delta_m": float(np.mean(deltas)),
        "median_delta_m": float(np.median(deltas)),
        "positive_pair_fraction": float(np.mean(deltas > 0.0)),
        "ci_95_m": [float(ci[0]), float(ci[1])],
        "lower_bound_gt_zero": bool(ci[0] > 0.0),
        "seed_hunting": False,
    }
    write_json(R4 / "paired_bootstrap.json", bootstrap)

    best_safety = max(summaries[candidate_id]["safety_rate"] for candidate_id in CONTROLLERS if candidate_id != SELF)
    best_success = max(summaries[candidate_id]["success_rate"] for candidate_id in CONTROLLERS if candidate_id != SELF)
    position = primary_cmp["position_rmse_3d_m"]["improvement_fraction"]
    acquisition = primary_cmp["acquisition_median_s"]["improvement_fraction"]
    ramp_peak = primary_cmp["ramp_peak_position_error_m"]["improvement_fraction"]
    ramp_steady = primary_cmp["ramp_steady_state_position_error_m"]["improvement_fraction"]
    gates = {
        "safety_no_worse_than_best_traditional": self_summary["safety_rate"] >= best_safety,
        "success_no_worse_than_best_traditional": self_summary["success_rate"] >= best_success,
        "position_improvement_ge_5pct_vs_primary": position is not None and position >= 0.05,
        "acquisition_degradation_le_5pct_vs_primary": acquisition is not None and acquisition >= -0.05,
        "ramp_peak_or_steady_improvement_ge_5pct": (ramp_peak is not None and ramp_peak >= 0.05) or (ramp_steady is not None and ramp_steady >= 0.05),
        "acquisition_or_ramp_additional_improvement_ge_5pct": (acquisition is not None and acquisition >= 0.05) or (ramp_peak is not None and ramp_peak >= 0.05) or (ramp_steady is not None and ramp_steady >= 0.05),
        "paired_bootstrap_ci_lower_gt_zero": bootstrap["lower_bound_gt_zero"],
    }
    overall = all(gates.values())
    strict_gates = {
        "safety_no_worse_than_best_traditional": gates["safety_no_worse_than_best_traditional"],
        "success_no_worse_than_best_traditional": gates["success_no_worse_than_best_traditional"],
        "position_improvement_ge_5pct": gates["position_improvement_ge_5pct_vs_primary"],
        "acquisition_improvement_ge_5pct": acquisition is not None and acquisition >= 0.05,
        "ramp_peak_or_steady_improvement_ge_5pct": gates["ramp_peak_or_steady_improvement_ge_5pct"],
        "paired_bootstrap_ci_lower_gt_zero": bootstrap["lower_bound_gt_zero"],
    }
    strict = all(strict_gates.values())
    final_status = "V3_SELF_STRICT_HOLDOUT_WIN" if strict else "V3_SELF_OVERALL_HOLDOUT_WIN" if overall else "V3_SELF_DEVELOPMENT_WIN_NOT_CONFIRMED_ON_HOLDOUT"

    safety = {
        "all_controllers": {candidate_id: {"safe_samples": summary["safe_sample_count"], "sample_count": 57, "safety_rate": summary["safety_rate"]} for candidate_id, summary in summaries.items()},
        "best_traditional_safety_rate": best_safety,
        "self_no_worse": gates["safety_no_worse_than_best_traditional"],
        "failure_reasons": {candidate_id: sorted({row["safety_failure_reasons"] for row in values if row["safety_failure_reasons"] not in {"", "[]"}}) for candidate_id, values in by_candidate.items()},
    }
    write_json(R4 / "holdout_safety_audit.json", safety)
    write_json(R4 / "paper_final_status.json", {
        "method": "LV2026-SPACC-ADAPTED-3D",
        "development_status": "NO_WIN",
        "holdout_status": "NOT_RUN_DEVELOPMENT_INELIGIBLE",
        "holdout_runs": 0,
        "allowed_claim": "The V3 adaptation of Lv et al. did not satisfy Development qualification and therefore was not evaluated on Holdout.",
        "forbidden_claims": ["Self > original Lv et al.", "Self > Paper on Holdout"],
    })
    write_json(R4 / "claim_matrix.json", {
        "final_self_status": final_status,
        "overall_win": overall,
        "strict_all_metric_win_holdout": strict,
        "allowed": [
            "All four frozen V3 controller participants were evaluated once on the same 57-sample unseen Holdout.",
            "Self comparisons may be reported against corrected PID, Full-LQR, and Task-LQR using the frozen metrics.",
            "The Paper adaptation was Development-ineligible and was not run on Holdout.",
        ],
        "not_allowed": ["second V3 Holdout", "post-Holdout Self tuning", "Self > original Lv et al.", "Self > Paper on Holdout"],
    })
    gate = {
        "task": "V3-R4-ONE-SHOT-HOLDOUT-VALIDATION-AND-FINAL-EVIDENCE-FREEZE-R1",
        "start_head": "a668e1ebdd9de1341e6c1cd278ed0b21f730ed83",
        "holdout_protocol_freeze_head": state["protocol_freeze_head"],
        "final_evidence_commit_definition": "the Git commit containing this gate and all final R4 evidence",
        "original_r0_contract_modified": False,
        "sample_count": 57,
        "controllers_executed": 4,
        "authoritative_runs": 228,
        "retries": state["retries"],
        "compromised": False,
        "paper_holdout_status": "NOT_RUN_DEVELOPMENT_INELIGIBLE",
        "gates": gates,
        "strict_gates": strict_gates,
        "overall_win": overall,
        "strict_all_metric_win_holdout": strict,
        "final_status": final_status,
        "second_holdout_authorized": False,
        "v3_closed_after_this_task": True,
    }
    write_json(R4 / "gate.json", gate)

    def pct(value: float | None) -> str:
        return "N/A" if value is None else f"{100.0 * value:.3f}%"

    report = f"""# V3 Final Technical Report

## Outcome

The unique V3 Holdout completed without compromise. The final Self claim is `{final_status}`. No controller, parameter, metric, safety rule, plant, or win contract was changed after protocol freeze.

## Holdout protocol

- 57 unseen samples: 12 calm spherical targets, 12 at 2.0 m/s, 12 at 3.5 m/s, 20 stochastic samples with seeds 3000--3019, and one 0--3.5 m/s ramp equilibrium hold.
- Four frozen participants used the same manifest: corrected PID, Full-State LQR, Task-Weighted LQR, and `self_a_034` / 3D-DR-TSRMPC.
- Primary Traditional remained `full_lqr_048`.
- Paper status remained `NOT_RUN_DEVELOPMENT_INELIGIBLE`.

## Controller summary

| Controller | Safety | Success | Position RMSE (m) | Acquisition median (s) | Orientation RMSE (deg) | Ramp peak (m) | Ramp steady (m) | Effort | Solve p95 (ms) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
"""
    labels = {
        "hybrid_x007_y041_z041": "Corrected PID",
        "full_lqr_048": "Full-LQR",
        "task_lqr_009": "Task-LQR",
        "self_a_034": "Self",
    }
    for candidate_id in labels:
        s = summaries[candidate_id]
        acquisition_text = "N/A" if s["acquisition_median_s"] is None else f"{s['acquisition_median_s']:.6f}"
        report += f"| {labels[candidate_id]} | {s['safety_rate']:.6f} | {s['success_rate']:.6f} | {s['position_rmse_3d_m']:.9f} | {acquisition_text} | {s['orientation_rmse_deg']:.9f} | {s['ramp_peak_position_error_m']:.9f} | {s['ramp_steady_state_position_error_m']:.9f} | {s['total_acceleration_effort']:.9f} | {s['solve_time_p95_ms']:.6f} |\n"
    report += f"""

## Self versus frozen Primary

- Position improvement: {pct(position)}
- Acquisition improvement: {pct(acquisition)}
- Ramp peak improvement: {pct(ramp_peak)}
- Ramp steady improvement: {pct(ramp_steady)}
- Exact-pair bootstrap: mean delta {bootstrap['mean_delta_m']:.9f} m, median {bootstrap['median_delta_m']:.9f} m, positive fraction {bootstrap['positive_pair_fraction']:.6f}, 95% CI [{bootstrap['ci_95_m'][0]:.9f}, {bootstrap['ci_95_m'][1]:.9f}] m.

## Frozen gates

- Overall Holdout win: `{overall}`
- Strict all-metric Holdout win: `{strict}`
- Final status: `{final_status}`

The formal acquisition gate uses the frozen aggregate definition. Common-success paired acquisition is retained only as a diagnostic. Ramp summary values preserve the inherited R1 aggregation semantics: the maximum per-sample peak/steady metric across the exact bank.

## Paper route and final boundary

The V3 adaptation of Lv et al. did not satisfy Development qualification and therefore was not evaluated on Holdout. This report does not claim that Self outperformed the original paper or a Paper controller on Holdout. V3 is permanently closed after this evidence freeze; no Self retuning or second V3 Holdout is authorized.
"""
    DOCS.mkdir(parents=True, exist_ok=True)
    (DOCS / "V3_FINAL_TECHNICAL_REPORT.md").write_text(report, encoding="utf-8", newline="\n")
    print(json.dumps({"result": final_status, "overall_win": overall, "strict_all_metric_win": strict, "bootstrap": bootstrap}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
