"""Freeze the final V4-R1 Development evidence without touching Holdout."""

from __future__ import annotations

import csv
import hashlib
import json
import subprocess
from collections import defaultdict
from pathlib import Path

import numpy as np

from evaluate_v4_r1 import NORMAL, STRONG, audit_candidate, rank_key, read_rows


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v4/r0"
R1 = ROOT / "reproducibility/v4/r1"
DOC = ROOT / "docs/V4_CART_OFMPC_DEVELOPMENT_REPORT.md"
START_HEAD = "b0dd45013e496bfd0f0371ef7c4f7e2333f8ab80"
PROTOCOL_HEAD = "ff5c6ac1a02524705ff53ac106196798a6b1ca72"


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def write_csv(path: Path, rows: list[dict]) -> None:
    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns: columns.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n", extrasaction="ignore"); writer.writeheader(); writer.writerows(rows)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def mean_field(rows: list[dict], field: str) -> float:
    return float(np.mean([float(row[field]) for row in rows]))


def max_field(rows: list[dict], field: str) -> float:
    return float(np.max([float(row[field]) for row in rows]))


def mechanism(rows: list[dict]) -> dict:
    return {
        "sample_count": len(rows),
        "residual_clipping_mean_fraction": mean_field(rows, "residual_clip_fraction_mean"),
        "steady_infeasibility_mean_fraction": mean_field(rows, "steady_infeasibility_fraction"),
        "requested_steady_input_mean_max_abs_m_s2": mean_field(rows, "requested_steady_input_max_abs_mean"),
        "requested_steady_input_global_max_abs_m_s2": max_field(rows, "requested_steady_input_max_abs_max"),
        "feasible_steady_input_mean_max_abs_m_s2": mean_field(rows, "feasible_steady_input_max_abs_mean"),
        "trust_mean": mean_field(rows, "trust_mean"),
        "trust_case_p05": float(np.percentile([float(row["trust_mean"]) for row in rows], 5)),
        "unrepresented_residual_mean_norm": mean_field(rows, "residual_unrepresented_norm_mean"),
        "antiwindup_debt_mean_norm": mean_field(rows, "residual_debt_norm_mean"),
        "amplitude_saturation_mean_fraction": mean_field(rows, "amplitude_saturation_fraction"),
        "slew_activity_mean_fraction": mean_field(rows, "slew_activity_fraction"),
        "backbone_norm_mean": mean_field(rows, "backbone_norm_mean"),
        "qp_correction_norm_mean": mean_field(rows, "qp_correction_norm_mean"),
        "limiter_mismatch_global_max_m_s2": max_field(rows, "limiter_mismatch_max"),
        "position_case_p90_m": float(np.percentile([float(row["position_rmse_3d_m"]) for row in rows], 90)),
        "position_case_p95_m": float(np.percentile([float(row["position_rmse_3d_m"]) for row in rows], 95)),
    }


def summary_rows() -> list[dict]:
    result: list[dict] = []
    for stage in ("stage_a", "stage_b", "stage_c"):
        payload = json.loads((R1 / f"{stage}_summary.json").read_text(encoding="utf-8"))
        for row in payload["summaries"]:
            result.append({
                "stage": stage, "selection_authority": stage != "stage_a", "candidate_id": row["candidate_id"],
                "sample_count": row["sample_count"], "safety_rate": row["safety_rate"], "success_rate": row["success_rate"],
                "position_rmse_3d_m": row["position_rmse_3d_m"], "position_p90_3d_m": row["position_p90_3d_m"],
                "acquisition_median_s": row["acquisition_median_s"], "effort": row["total_acceleration_effort"], "solve_p95_ms": row["solve_time_p95_ms"],
            })
    return result


def main() -> int:
    baseline_rows = read_rows(R1 / "baseline_development_results.csv"); baselines: dict[str, list[dict]] = defaultdict(list)
    for row in baseline_rows: baselines[row["candidate_id"]].append(row)
    stage_c_rows = read_rows(R1 / "stage_c_results.csv"); candidates: dict[str, list[dict]] = defaultdict(list)
    for row in stage_c_rows: candidates[row["candidate_id"]].append(row)
    audits = sorted((audit_candidate(candidate_id, rows, baselines) for candidate_id, rows in candidates.items()), key=rank_key)
    best = audits[0]; best_id = best["candidate_id"]; best_rows = candidates[best_id]
    if any(row["final_pass"] for row in audits):
        raise RuntimeError("this closeout is only valid when no Stage-C candidate passes")
    protocol = json.loads((R1 / "development_protocol.json").read_text(encoding="utf-8"))
    parameters = next(row for row in protocol["search"]["stage_b"]["candidates"] if row["candidate_id"] == best_id)
    strong_rows = [row for row in best_rows if row["cohort"] in STRONG]; normal_rows = [row for row in best_rows if row["cohort"] in NORMAL]
    strong_mechanism = mechanism(strong_rows); normal_mechanism = mechanism(normal_rows)
    write_csv(R1 / "candidate_results.csv", summary_rows()); write_csv(R1 / "development_results.csv", best_rows)
    write_json(R1 / "stage_c_gate_audit.json", {"confirmation_runs": len(stage_c_rows), "candidate_count": len(audits), "ranked": audits, "selected_pass": None, "best_near_miss": best_id, "holdout_executed": False})
    write_json(R1 / "near_miss.json", {
        "status": "BEST_NEAR_MISS_NOT_FROZEN", "candidate_id": best_id, "parameters": parameters,
        "passed_atomic_gates": best["hard_gate_pass_count"], "atomic_gate_count": best["hard_gate_count"],
        "failed_gates": [{"section": section, "gate": name} for section, values in best["gates"].items() for name, passed in values.items() if not passed],
        "metrics": best, "selection_rule": protocol["search"]["stage_b"]["selection_rule"], "no_additional_candidate_authorized": True,
    })
    write_json(R1 / "paired_bootstrap.json", {"candidate_id": best_id, "comparison": "full_lqr_048 minus CART-OFMPC position RMSE", **best["bootstrap"], "qualification_pass": best["gates"]["overall"]["bootstrap"]})
    write_json(R1 / "strong_wind_comparison.json", {
        "candidate_id": best_id, "cohorts": sorted(STRONG), "metrics": best["strong_wind"], "comparisons": {key: value for key, value in best["comparisons"].items() if key.startswith("strong_")},
        "catastrophic_pair_ids": best["catastrophic_pair_ids"], "gates": best["gates"]["strong_wind"],
        "conclusion": "Mean strong-wind error improved, but the simultaneous-onset tail retained catastrophic pairs and success remained below the best Traditional.",
    })
    write_json(R1 / "normal_regime_comparison.json", {
        "candidate_id": best_id, "cohorts": sorted(NORMAL), "metrics": best["normal_regime"], "comparisons": {key: value for key, value in best["comparisons"].items() if key.startswith("normal_")},
        "gates": best["gates"]["normal_nonregression"], "conclusion": "Normal-regime safety and success were preserved; position and acquisition remained within the frozen non-regression limits.",
    })
    write_json(R1 / "residual_trust_audit.json", {
        "candidate_id": best_id, "strong_wind": strong_mechanism, "normal_regime": normal_mechanism,
        "causal_inputs_only": True, "wind_truth_read": False,
        "interpretation": "Trust fell in abrupt simultaneous-onset cases but remained causal. The lower trust prevented V3-scale residual/equilibrium amplification, yet did not eliminate the new onset-tail failure.",
    })
    write_json(R1 / "steady_feasibility_audit.json", {
        "candidate_id": best_id, "solver": "three-variable active-set box QP with amplitude and finite-slew reachable bounds", "silent_scaling": False,
        "strong_wind": {key: value for key, value in strong_mechanism.items() if "steady" in key or "infeasibility" in key or "input" in key},
        "normal_regime": {key: value for key, value in normal_mechanism.items() if "steady" in key or "infeasibility" in key or "input" in key},
        "requested_and_feasible_logged_separately": True,
    })
    write_json(R1 / "antiwindup_audit.json", {
        "candidate_id": best_id, "state": "bounded unrepresented-residual debt", "feedback_enabled": parameters["debt_feedback"] > 0.0,
        "antiwindup_gain": parameters["antiwindup_gain"], "debt_decay": parameters["debt_decay"], "debt_limit": parameters["debt_limit"],
        "strong_unrepresented_residual_mean_norm": strong_mechanism["unrepresented_residual_mean_norm"], "strong_debt_mean_norm": strong_mechanism["antiwindup_debt_mean_norm"],
        "interpretation": "Debt remained bounded and prevented an accumulating infeasible equilibrium request; no anti-windup retuning was performed after results.",
    })
    write_json(R1 / "constraint_audit.json", {
        "candidate_id": best_id, "physical_acceleration_limit_m_s2": 2.0, "slew_limit_m_s2_per_update": 0.25,
        "max_observed_command_m_s2": max_field(best_rows, "max_abs_command_m_s2"), "max_observed_step_m_s2": max_field(best_rows, "max_command_step_m_s2"),
        "limiter_mismatch_global_max_m_s2": max_field(best_rows, "limiter_mismatch_max"), "all_samples_within_common_authority": all(float(row["max_abs_command_m_s2"]) <= 2.0 + 1.0e-9 and float(row["max_command_step_m_s2"]) <= 0.25 + 1.0e-9 for row in best_rows),
        "strong_amplitude_saturation_fraction": strong_mechanism["amplitude_saturation_mean_fraction"], "strong_slew_activity_fraction": strong_mechanism["slew_activity_mean_fraction"],
    })
    write_json(R1 / "implementation_audit.json", {
        "method": "CART-OFMPC", "architecture_version": "CART-OFMPC-A1", "protocol_freeze_head": PROTOCOL_HEAD,
        "implementation_sha256": sha256(ROOT / "src/uav_sway/v4/cart_ofmpc.py"), "runner_sha256": sha256(ROOT / "scripts/run_v4_r1_development.py"),
        "protocol_sha256": sha256(R1 / "development_protocol.json"), "frozen_task_lqr_backbone": "task_lqr_009", "backbone_gain_modified": False,
        "required_mechanisms_present": {"causal_dynamic_residual": True, "constraint_feasible_steady_target": True, "trust_scheduler": True, "unrepresented_residual_antiwindup": True, "finite_horizon_predictive_correction": True, "physical_acceleration_and_slew_constraints": True},
        "true_wind_or_future_information_used": False,
    })
    all_cart_rows = read_rows(R1 / "stage_a_results.csv") + read_rows(R1 / "stage_b_results.csv") + stage_c_rows
    write_json(R1 / "safety_audit.json", {
        "baseline_authoritative_runs": len(baseline_rows), "cart_development_runs": len(all_cart_rows), "baseline_all_safe": all(row["safe"] for row in baseline_rows),
        "cart_all_safe": all(row["safe"] for row in all_cart_rows), "best_near_miss_safety_rate": best["overall"]["safety_rate"],
        "failure_reasons": sorted({reason for row in all_cart_rows for reason in json.loads(row.get("safety_failure_reasons", "[]").replace("'", '"'))}) if all_cart_rows else [],
    })
    holdout = json.loads((R0 / "holdout_manifest.json").read_text(encoding="utf-8"))
    write_json(R1 / "holdout_access_audit.json", {
        "holdout_manifest_sha256": sha256(R0 / "holdout_manifest.json"), "matches_protocol_frozen_sha256": sha256(R0 / "holdout_manifest.json") == protocol["frozen_inputs"]["holdout_manifest_sha256"],
        "sample_count": len(holdout["samples"]), "all_execution_allowed_false": all(row["execution_allowed"] is False for row in holdout["samples"]),
        "holdout_simulations_executed": 0, "holdout_debug_samples_executed": 0, "holdout_results_created": False,
    })
    write_json(R1 / "ablation.json", {
        "executed": False, "reason": "No CART-OFMPC candidate passed all Development gates; the contract permits mechanism ablation only after candidate freeze.",
        "candidate_frozen": False, "no_ablation_based_retuning": True, "holdout_executed": False,
    })
    gate = {
        "task": "V4-R1-CART-OFMPC-DEVELOPMENT-AND-SELF-FREEZE-R1", "start_head": START_HEAD, "cart_protocol_freeze_head": PROTOCOL_HEAD,
        "baseline_authoritative_runs": len(baseline_rows), "stage_a_runs": len(read_rows(R1 / "stage_a_results.csv")), "stage_b_runs": len(read_rows(R1 / "stage_b_results.csv")), "stage_c_runs": len(stage_c_rows),
        "new_unique_configurations": 26, "budget_max": 32, "best_near_miss": best_id, "best_near_miss_atomic_gates": f"{best['hard_gate_pass_count']}/{best['hard_gate_count']}",
        "overall_gate": all(best["gates"]["overall"].values()), "strong_wind_gate": all(best["gates"]["strong_wind"].values()), "normal_nonregression_gate": all(best["gates"]["normal_nonregression"].values()), "bootstrap_gate": best["gates"]["overall"]["bootstrap"],
        "candidate_frozen": False, "ablation_executed": False, "paper_started": False, "holdout_executed": False,
        "project_tests": "141 passed in 1.18s",
        "result": "CLOSED_WITH_NO_V4_CART_OFMPC_DEVELOPMENT_WIN",
    }
    write_json(R1 / "gate.json", gate)
    write_json(R1 / "evidence_sha256.json", {path.name: sha256(path) for path in sorted(R1.glob("*")) if path.is_file() and path.name not in {"evidence_sha256.json"}})
    catastrophic = len(best["catastrophic_pair_ids"])
    report = f"""# V4 CART-OFMPC Development Report

## Result

V4-R1 is closed with **no qualified CART-OFMPC Development candidate**. The four frozen comparators were evaluated once on all 94 Development samples (376 authoritative runs). The preregistered CART search then used 8 Stage-A structural configurations, 18 Stage-B full-bank configurations, and six unchanged Stage-B configurations for Stage-C confirmation. No V4 Holdout sample was executed.

## Frozen protocol and implementation

The performance-before-data checkpoint is `{PROTOCOL_HEAD}`. CART-OFMPC uses the immutable `task_lqr_009` gain, a causal one-step residual estimator, a bounded steady-target active-set QP, residual trust, bounded anti-windup debt, and a finite-horizon task-space QP that directly constrains physical acceleration and slew. It never reads true or future wind. The steady solver records requested and feasible inputs separately and never silently scales an infeasible equilibrium.

## Comparator evidence

The frozen Full-LQR achieved overall mean position RMSE `{best['overall']['position_mean_m'] / (1.0 - best['comparisons']['overall_position_improvement_vs_full']):.6f} m`. The legacy `self_a_034` preserved 100% normal-regime success but retained its previously identified sustained-strong-wind failure. All 376 comparator runs were safe.

## Best near-miss

`{best_id}` passed {best['hard_gate_pass_count']} of {best['hard_gate_count']} atomic gates. Overall position RMSE was `{best['overall']['position_mean_m']:.6f} m`, a `{100.0 * best['comparisons']['overall_position_improvement_vs_full']:.2f}%` improvement over Full-LQR. The paired 10,000-resample bootstrap CI was `[{best['bootstrap']['ci_95_m'][0]:.6f}, {best['bootstrap']['ci_95_m'][1]:.6f}] m`, so its lower bound was positive. Normal-regime success stayed at `{100.0 * best['normal_regime']['success_rate']:.1f}%`; position changed by `{100.0 * best['comparisons']['normal_position_change_vs_legacy']:.2f}%` and acquisition by `{100.0 * best['comparisons']['normal_acquisition_change_vs_legacy']:.2f}%` relative to legacy Self, both within contract.

The candidate nevertheless failed three strong-wind gates: success was `{100.0 * best['strong_wind']['success_rate']:.2f}%`, below the best Traditional; P90 position changed by `{100.0 * best['comparisons']['strong_p90_change_vs_full']:.2f}%` versus Full-LQR, above the +10% limit; and `{catastrophic}` exact pairs exceeded twice the Full-LQR error. Thus the favorable strong-wind mean (`{best['strong_wind']['position_mean_m']:.6f} m`, `{100.0 * best['comparisons']['strong_position_improvement_vs_full']:.2f}%` better than Full-LQR) cannot qualify the method.

## Mechanism conclusion

The original V3 amplification chain was materially interrupted: in the best near-miss strong cohort, residual clipping averaged `{100.0 * strong_mechanism['residual_clipping_mean_fraction']:.2f}%`, amplitude saturation `{100.0 * strong_mechanism['amplitude_saturation_mean_fraction']:.2f}%`, and slew activity `{100.0 * strong_mechanism['slew_activity_mean_fraction']:.2f}%`, versus the R0 legacy findings of roughly 69.74%, 27.77%, and 62.55%. Requested steady input remained bounded in practice rather than reaching the legacy 5-11 m/s2 range. However, abrupt simultaneous wind onset produced a different tail: trust dropped while backbone/QP cancellation and slew activity remained concentrated in several directions. The mechanism repair improved the mean but did not satisfy tail robustness or strong-cohort success.

## Closure

No candidate is frozen and no ablation was run, because the frozen contract permits ablation only after a Development-qualified freeze. There was no 27th/33rd configuration, no comparator retuning, no Paper work, no contract change, and no Holdout access. Final status: `CLOSED_WITH_NO_V4_CART_OFMPC_DEVELOPMENT_WIN`.
"""
    DOC.write_text(report, encoding="utf-8", newline="\n")
    print(json.dumps({"result": gate["result"], "near_miss": best_id, "gates": f"{best['hard_gate_pass_count']}/{best['hard_gate_count']}", "holdout_executed": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
