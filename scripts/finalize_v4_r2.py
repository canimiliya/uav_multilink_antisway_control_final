"""Build the final V4 evidence package from frozen V3/V4 evidence only.

This script performs no simulation and imports no controller runner.  It reads
the already committed R0/R1 JSON and CSV evidence, computes offline aggregates,
and writes the immutable V4 final closeout package.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import statistics
import subprocess
from pathlib import Path
from typing import Iterable


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v4/r0"
R1 = ROOT / "reproducibility/v4/r1"
FINAL = ROOT / "reproducibility/v4/final"
REPORT = ROOT / "docs/V4_FINAL_TECHNICAL_REPORT.md"
START_HEAD = "720e65ecc1ae0104fff00c7498ca19d7c8878038"
V3_TREE = "956e77658e01efabb110dcf1934215af344a620d"
V4_R0_TREE = "0eadf8a8b9e173bd9d73e06521e94a42a0938b70"
V4_R1_TREE = "0406181f317c60ca16687099841593d48f89efe3"
TASK = "V4-R2-FINAL-EVIDENCE-FREEZE-AND-V5-DECISION-EVIDENCE-R1"
RESULT = "V4_CLOSED_WITH_MECHANISM_IMPROVEMENT_BUT_NO_QUALIFIED_SELF"


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
        newline="\n",
    )


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = list(rows[0])
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def f(row: dict, key: str) -> float:
    return float(row[key])


def mean(rows: Iterable[dict], key: str) -> float:
    return statistics.fmean(f(row, key) for row in rows)


def average_ranks(values: list[float]) -> list[float]:
    indexed = sorted(enumerate(values), key=lambda item: item[1])
    result = [0.0] * len(values)
    start = 0
    while start < len(indexed):
        end = start + 1
        while end < len(indexed) and indexed[end][1] == indexed[start][1]:
            end += 1
        rank = (start + end - 1) / 2.0
        for index, _ in indexed[start:end]:
            result[index] = rank
        start = end
    return result


def pearson(left: list[float], right: list[float]) -> float:
    left_mean = statistics.fmean(left)
    right_mean = statistics.fmean(right)
    numerator = sum((a - left_mean) * (b - right_mean) for a, b in zip(left, right))
    left_energy = sum((a - left_mean) ** 2 for a in left)
    right_energy = sum((b - right_mean) ** 2 for b in right)
    if left_energy == 0.0 or right_energy == 0.0:
        return 0.0
    return numerator / math.sqrt(left_energy * right_energy)


def spearman(left: list[float], right: list[float]) -> float:
    return pearson(average_ranks(left), average_ranks(right))


def group_summary(rows: list[dict], manifest: dict[str, dict]) -> dict:
    directions = [manifest[row["sample_id"]]["target"]["direction_unit"] for row in rows]
    qp_to_backbone = [
        f(row, "qp_correction_norm_mean") / max(f(row, "backbone_norm_mean"), 1.0e-12)
        for row in rows
    ]
    return {
        "sample_count": len(rows),
        "target_direction_mean": [statistics.fmean(direction[i] for direction in directions) for i in range(3)],
        "target_direction_abs_mean": [statistics.fmean(abs(direction[i]) for direction in directions) for i in range(3)],
        "target_positive_counts_xyz": [sum(direction[i] > 0.0 for direction in directions) for i in range(3)],
        "position_rmse_mean_m": mean(rows, "position_rmse_3d_m"),
        "position_peak_proxy_mean_m": mean(rows, "ramp_peak_position_error_m"),
        "orientation_rmse_mean_deg": mean(rows, "orientation_rmse_deg"),
        "trust_mean": mean(rows, "trust_mean"),
        "residual_clip_fraction_mean": mean(rows, "residual_clip_fraction_mean"),
        "residual_debt_norm_mean": mean(rows, "residual_debt_norm_mean"),
        "requested_steady_input_max_abs_mean_m_s2": mean(rows, "requested_steady_input_max_abs_mean"),
        "backbone_norm_mean": mean(rows, "backbone_norm_mean"),
        "qp_correction_norm_mean": mean(rows, "qp_correction_norm_mean"),
        "qp_to_backbone_norm_ratio_mean": statistics.fmean(qp_to_backbone),
        "slew_activity_fraction_mean": mean(rows, "slew_activity_fraction"),
        "amplitude_activity_fraction_mean": mean(rows, "amplitude_saturation_fraction"),
    }


def dominates(left: dict, right: dict) -> bool:
    # Success is maximized; error, tail, acquisition, and effort are minimized.
    left_values = (
        -float(left["success_rate"]),
        float(left["position_rmse_3d_m"]),
        float(left["position_p90_3d_m"]),
        float(left["acquisition_median_s"]),
        float(left["effort"]),
    )
    right_values = (
        -float(right["success_rate"]),
        float(right["position_rmse_3d_m"]),
        float(right["position_p90_3d_m"]),
        float(right["acquisition_median_s"]),
        float(right["effort"]),
    )
    return all(a <= b for a, b in zip(left_values, right_values)) and any(
        a < b for a, b in zip(left_values, right_values)
    )


def main() -> int:
    if git("rev-parse", "HEAD") != START_HEAD:
        raise RuntimeError("finalizer must start at the frozen V4-R1 HEAD")
    protected = {
        "reproducibility/v3": git("rev-parse", "HEAD:reproducibility/v3"),
        "reproducibility/v4/r0": git("rev-parse", "HEAD:reproducibility/v4/r0"),
        "reproducibility/v4/r1": git("rev-parse", "HEAD:reproducibility/v4/r1"),
    }
    expected = {
        "reproducibility/v3": V3_TREE,
        "reproducibility/v4/r0": V4_R0_TREE,
        "reproducibility/v4/r1": V4_R1_TREE,
    }
    if protected != expected:
        raise RuntimeError(f"protected tree drift: {protected}")

    near = read_json(R1 / "near_miss.json")
    strong = read_json(R1 / "strong_wind_comparison.json")
    normal = read_json(R1 / "normal_regime_comparison.json")
    bootstrap = read_json(R1 / "paired_bootstrap.json")
    legacy = read_json(R0 / "v3_failure_postmortem.json")
    holdout = read_json(R0 / "holdout_manifest.json")
    stage_b_gate = read_json(R1 / "stage_b_gate_audit.json")
    candidate_rows = read_csv(R1 / "candidate_results.csv")
    development_rows = read_csv(R1 / "development_results.csv")
    manifest_rows = read_json(R0 / "development_manifest.json")["samples"]
    manifest = {row["sample_id"]: row for row in manifest_rows}

    catastrophic_ids = set(strong["catastrophic_pair_ids"])
    simultaneous = [row for row in development_rows if row["cohort"] == "STRONG_SIMULTANEOUS"]
    catastrophic = [row for row in simultaneous if row["sample_id"] in catastrophic_ids]
    noncatastrophic = [row for row in simultaneous if row["sample_id"] not in catastrophic_ids]
    if len(simultaneous) != 12 or len(catastrophic) != 7:
        raise RuntimeError("unexpected simultaneous cohort composition")

    directional_rows = []
    for row in simultaneous:
        direction = manifest[row["sample_id"]]["target"]["direction_unit"]
        backbone = f(row, "backbone_norm_mean")
        directional_rows.append(
            {
                "sample_id": row["sample_id"],
                "catastrophic": row["sample_id"] in catastrophic_ids,
                "target_x": direction[0],
                "target_y": direction[1],
                "target_z": direction[2],
                "position_rmse_3d_m": f(row, "position_rmse_3d_m"),
                "position_peak_proxy_m": f(row, "ramp_peak_position_error_m"),
                "orientation_rmse_deg": f(row, "orientation_rmse_deg"),
                "trust_mean": f(row, "trust_mean"),
                "residual_clip_fraction_mean": f(row, "residual_clip_fraction_mean"),
                "residual_debt_norm_mean": f(row, "residual_debt_norm_mean"),
                "requested_steady_input_max_abs_mean_m_s2": f(row, "requested_steady_input_max_abs_mean"),
                "backbone_norm_mean": backbone,
                "qp_correction_norm_mean": f(row, "qp_correction_norm_mean"),
                "qp_to_backbone_norm_ratio": f(row, "qp_correction_norm_mean") / max(backbone, 1.0e-12),
                "slew_activity_fraction": f(row, "slew_activity_fraction"),
                "amplitude_activity_fraction": f(row, "amplitude_saturation_fraction"),
            }
        )
    write_csv(FINAL / "simultaneous_directional_samples.csv", directional_rows)

    position = [f(row, "position_rmse_3d_m") for row in simultaneous]
    correlations = {}
    for key in (
        "orientation_rmse_deg",
        "trust_mean",
        "residual_clip_fraction_mean",
        "residual_debt_norm_mean",
        "requested_steady_input_max_abs_mean",
        "backbone_norm_mean",
        "qp_correction_norm_mean",
        "slew_activity_fraction",
        "amplitude_saturation_fraction",
    ):
        correlations[key] = spearman(position, [f(row, key) for row in simultaneous])
    for index, axis in enumerate("xyz"):
        signed = [manifest[row["sample_id"]]["target"]["direction_unit"][index] for row in simultaneous]
        correlations[f"target_{axis}_signed"] = spearman(position, signed)
        correlations[f"target_{axis}_absolute"] = spearman(position, [abs(value) for value in signed])

    directional_analysis = {
        "scope": "offline aggregation of the 12 frozen cart_b_001 STRONG_SIMULTANEOUS rows",
        "catastrophic_definition_source": "reproducibility/v4/r1/strong_wind_comparison.json",
        "catastrophic": group_summary(catastrophic, manifest),
        "noncatastrophic": group_summary(noncatastrophic, manifest),
        "sign_partition": {
            "negative_x": {
                "sample_count": sum(manifest[row["sample_id"]]["target"]["direction_unit"][0] < 0 for row in simultaneous),
                "catastrophic_count": sum(row["sample_id"] in catastrophic_ids and manifest[row["sample_id"]]["target"]["direction_unit"][0] < 0 for row in simultaneous),
            },
            "positive_x": {
                "sample_count": sum(manifest[row["sample_id"]]["target"]["direction_unit"][0] > 0 for row in simultaneous),
                "catastrophic_count": sum(row["sample_id"] in catastrophic_ids and manifest[row["sample_id"]]["target"]["direction_unit"][0] > 0 for row in simultaneous),
            },
            "interpretation": "x sign perfectly partitions this frozen 12-direction cohort; y sign, z sign, and vertical magnitude do not.",
            "generalization_limit": "exploratory n=12 directional association; it is not a causal or unseen-direction validation",
        },
        "spearman_correlations_with_position_rmse": correlations,
        "time_to_tail_onset": {
            "status": "NOT_OBSERVABLE_FROM_FROZEN_AGGREGATES",
            "reason": "R1 preserved one row per sample, not time-series traces; no new simulation is authorized.",
        },
        "backbone_qp_vector_cancellation": {
            "status": "VECTOR_ANGLE_NOT_OBSERVABLE_FROM_FROZEN_AGGREGATES",
            "available_proxy": "catastrophic cases show large co-activation and QP/backbone norm ratio, but vector opposition cannot be recomputed",
        },
    }
    write_json(FINAL / "directional_failure_analysis.json", directional_analysis)

    simultaneous_audit = {
        "candidate_id": "cart_b_001",
        "cohort": "STRONG_SIMULTANEOUS",
        "sample_count": 12,
        "catastrophic_pair_count": 7,
        "catastrophic_pair_ids": sorted(catastrophic_ids),
        "position_tail": {
            "strong_all_p90_m": strong["metrics"]["position_p90_m"],
            "p90_change_vs_full_lqr": strong["comparisons"]["strong_p90_change_vs_full"],
            "catastrophic_position_mean_m": directional_analysis["catastrophic"]["position_rmse_mean_m"],
            "noncatastrophic_position_mean_m": directional_analysis["noncatastrophic"]["position_rmse_mean_m"],
        },
        "orientation": {
            "catastrophic_rmse_mean_deg": directional_analysis["catastrophic"]["orientation_rmse_mean_deg"],
            "noncatastrophic_rmse_mean_deg": directional_analysis["noncatastrophic"]["orientation_rmse_mean_deg"],
        },
        "mechanism_hierarchy": {
            "confirmed": [
                "The catastrophic partition is exactly the seven negative-x target directions in the frozen 12-direction simultaneous cohort.",
                "Catastrophic rows have much lower trust, greater residual clipping/debt, larger backbone and QP norms, and much higher slew activity than non-catastrophic rows.",
                "All 18 full-bank Stage-B candidates failed both the strong P90-tail and catastrophic-pair gates.",
            ],
            "strongly_supported": [
                "The wind-onset innovation and simultaneous target transient trigger a direction-dependent low-trust, high-correction regime.",
                "Large backbone/QP co-activation, correction larger than backbone by norm, and active slew limits support a coordination-limited recovery mechanism.",
                "The remaining failure is an abrupt-onset transient-robustness limitation rather than recurrence of V3 persistent-equilibrium infeasibility.",
            ],
            "hypothesized": [
                "The fixed +x wind and negative-x target demand may align the onset innovation against the task transient and produce the observed sign partition.",
                "Innovation-rate hysteresis, bumpless steady-target engagement, an explicit cancellation penalty, or a transient tube layer may interrupt the tail without normal-regime regression.",
            ],
        },
        "limitations": [
            "n=12 and Development-only: the exact x-sign split is exploratory, not a general law.",
            "No time series were frozen, so tail-onset time and backbone/QP vector opposition are not directly identifiable.",
            "Correlations are descriptive and do not establish causality.",
        ],
    }
    write_json(FINAL / "simultaneous_tail_audit.json", simultaneous_audit)

    stage_a = [row for row in candidate_rows if row["stage"] == "stage_a"]
    stage_b = [row for row in candidate_rows if row["stage"] == "stage_b"]
    frontier = [
        row["candidate_id"]
        for row in stage_b
        if not any(other is not row and dominates(other, row) for other in stage_b)
    ]
    ranked = {row["candidate_id"]: row for row in stage_b_gate["ranked"]}
    representatives = {}
    for candidate_id in ("cart_b_001", "cart_b_014"):
        gate_row = ranked[candidate_id]
        representatives[candidate_id] = {
            "overall": gate_row["overall"],
            "strong_wind": gate_row["strong_wind"],
            "normal_regime": gate_row["normal_regime"],
            "hard_gate_pass_count": gate_row["hard_gate_pass_count"],
            "failed_gates": [
                f"{section}.{name}"
                for section, values in gate_row["gates"].items()
                for name, passed in values.items()
                if not passed
            ],
            "catastrophic_pair_count": len(gate_row["catastrophic_pair_ids"]),
        }
    pareto = {
        "unique_configuration_count": len(stage_a) + len(stage_b),
        "stage_a_smoke_only_count": len(stage_a),
        "stage_b_full_bank_count": len(stage_b),
        "cross_stage_frontier_prohibited": True,
        "reason": "Stage A used 16 smoke samples and has no selection authority; only the 18 Stage-B full-bank candidates are mutually comparable.",
        "stage_b_objectives": {
            "maximize": ["success_rate"],
            "minimize": ["position_rmse_3d_m", "position_p90_3d_m", "acquisition_median_s", "effort"],
        },
        "stage_b_pareto_frontier": frontier,
        "strong_gate_pass_counts_across_stage_b": {
            "success": sum(row["gates"]["strong_wind"]["success"] for row in stage_b_gate["ranked"]),
            "p90_tail": sum(row["gates"]["strong_wind"]["p90_tail"] for row in stage_b_gate["ranked"]),
            "catastrophic_pairs": sum(row["gates"]["strong_wind"]["catastrophic_pairs"] for row in stage_b_gate["ranked"]),
        },
        "representative_comparison": representatives,
        "conclusion": "ARCHITECTURE_LEVEL_TRANSIENT_ROBUSTNESS_LIMITATION_WITH_PARAMETER_TRADEOFF",
        "interpretation": "cart_b_014 trades slower acquisition and normal-regime regression for higher strong success, yet still fails tail/catastrophic gates. No full-bank candidate clears either tail gate, so additional CART retuning is not evidence-supported under the exhausted frozen search.",
        "selection_changed": False,
    }
    write_json(FINAL / "pareto_analysis.json", pareto)

    legacy_strong = legacy["constant_3p5"]
    near_metrics = near["metrics"]
    final_metric_summary = {
        "best_near_miss": "cart_b_001",
        "passed_atomic_gates": 12,
        "atomic_gate_count": 15,
        "positive_evidence": {
            "legacy_residual_clip_fraction": legacy_strong["self_residual_clip_update_fraction"],
            "cart_strong_residual_clip_fraction": read_json(R1 / "residual_trust_audit.json")["strong_wind"]["residual_clipping_mean_fraction"],
            "legacy_slew_fraction": legacy_strong["self_slew_active_fraction"],
            "cart_strong_slew_fraction": read_json(R1 / "constraint_audit.json")["strong_slew_activity_fraction"],
            "legacy_requested_steady_input_mean_max_abs_m_s2": legacy_strong["self_unscaled_steady_input_mean_max_abs_m_s2"],
            "legacy_requested_steady_input_peak_max_abs_m_s2": legacy_strong["self_unscaled_steady_input_peak_max_abs_m_s2"],
            "cart_requested_steady_input_mean_max_abs_m_s2": read_json(R1 / "residual_trust_audit.json")["strong_wind"]["requested_steady_input_mean_max_abs_m_s2"],
            "cart_requested_steady_input_peak_max_abs_m_s2": read_json(R1 / "residual_trust_audit.json")["strong_wind"]["requested_steady_input_global_max_abs_m_s2"],
            "overall_position_improvement_vs_full_lqr": near_metrics["comparisons"]["overall_position_improvement_vs_full"],
            "strong_mean_improvement_vs_full_lqr": near_metrics["comparisons"]["strong_position_improvement_vs_full"],
            "strong_mean_improvement_vs_legacy_self": near_metrics["comparisons"]["strong_position_improvement_vs_legacy"],
            "bootstrap_lower_bound_gt_zero": bootstrap["lower_bound_gt_zero"],
            "normal_nonregression_pass": all(normal["gates"].values()),
        },
        "failure_evidence": {
            "strong_success_rate": strong["metrics"]["success_rate"],
            "strong_position_p90_m": strong["metrics"]["position_p90_m"],
            "strong_p90_change_vs_full_lqr": strong["comparisons"]["strong_p90_change_vs_full"],
            "catastrophic_pair_count": len(catastrophic_ids),
            "catastrophic_pair_ids": sorted(catastrophic_ids),
        },
        "paired_bootstrap": bootstrap,
    }
    write_json(FINAL / "final_metric_summary.json", final_metric_summary)

    write_json(
        FINAL / "final_method_status.json",
        {
            "method": "CART-OFMPC",
            "status": "CLOSED_WITH_NO_DEVELOPMENT_WIN",
            "best_near_miss": "cart_b_001",
            "best_near_miss_frozen_as_winner": False,
            "passed_gates": "12/15",
            "failed_gates": ["strong_success", "strong_p90_tail", "catastrophic_pair_count"],
            "mechanism_repair": "DEMONSTRATED_FOR_DOMINANT_V3_PERSISTENT_STRONG_WIND_INFEASIBILITY_CHAIN",
            "development_qualification": "FAIL",
            "holdout_eligibility": False,
            "claim": "Mechanism improvement with favorable mean performance, but no qualified V4 Self method.",
        },
    )

    write_json(
        FINAL / "final_claim_matrix.json",
        {
            "v3_self": {"development": "WIN", "holdout": "NOT_CONFIRMED"},
            "v4_cart_ofmpc": {
                "development_overall_mean_improvement": True,
                "mechanism_repair_demonstrated": True,
                "development_qualification": "FAIL",
                "holdout": "NOT_RUN",
            },
            "v4_holdout": "UNUSED",
            "paper": "NOT_PART_OF_V4",
            "prohibited_claims": [
                "CART-OFMPC WIN",
                "CART-OFMPC completely failed",
                "final strong-wind generalization succeeded",
                "V4 outperformed all baselines",
            ],
        },
    )

    write_json(
        FINAL / "unused_holdout_manifest.json",
        {
            "source": "reproducibility/v4/r0/holdout_manifest.json",
            "source_sha256": sha256(R0 / "holdout_manifest.json"),
            "sample_count": len(holdout["samples"]),
            "all_samples_execution_allowed_false": all(row["execution_allowed"] is False for row in holdout["samples"]),
            "sample_ids": [row["sample_id"] for row in holdout["samples"]],
            "v4_holdout_executed": False,
            "v4_holdout_status": "UNUSED",
            "retrospective_rescue_of_cart_ofmpc_prohibited": True,
            "future_inheritance_requires_new_version_decision": True,
        },
    )

    write_json(
        FINAL / "v5_recommendation.json",
        {
            "decision": "RECOMMEND_NEW_RESEARCH_VERSION",
            "authorization": "RECOMMENDATION_ONLY_V5_NOT_STARTED",
            "new_scientific_question": "Can a causal controller remain tail-robust when a 3D target step and strong wind onset occur simultaneously, without sacrificing normal-regime response?",
            "new_mechanism_hypothesis": "An onset innovation impulse and target transient enter a direction-dependent low-trust regime; backbone/predictive co-activation then meets slew-limited authority and creates a long directional tail.",
            "why_not_just_retune_cart": [
                "The frozen Stage-A/B/C budget is exhausted.",
                "All 18 full-bank Stage-B candidates failed both tail gates.",
                "cart_b_014 demonstrates the trade: better strong success still leaves catastrophic pairs and regresses normal position/acquisition.",
                "The remaining question concerns transient coordination, not the persistent-equilibrium infeasibility already repaired by CART.",
            ],
            "candidate_research_directions_not_implemented": [
                "transient shock-aware confidence governor with innovation-rate/consistency hysteresis",
                "slew-aware bumpless steady-target engagement",
                "explicit backbone-QP cancellation penalty with logged vector diagnostics",
                "transient robust or tube constraint layer",
            ],
            "must_be_frozen_before_performance": [
                "new version and independent Development/Holdout data policy",
                "causal observation and controller interface",
                "simultaneous-onset cohort and directional coverage",
                "P90 and catastrophic-pair gates",
                "time-series diagnostics needed to test onset and vector-cancellation hypotheses",
                "search budget, metrics, safety, and stopping rules",
            ],
        },
    )

    write_json(
        FINAL / "resume_claims.json",
        {
            "allowed": [
                "Identified a constraint-infeasible disturbance-compensation failure mode in a UAV suspended-load controller.",
                "Developed constraint-aware residual-trust MPC that reduced strong-wind mean position error while preserving normal-regime performance in Development.",
                "Reduced residual clipping from about 69.74% to 3.53% and slew activity from about 62.55% to 9.71% in the frozen strong-wind Development cohort.",
                "Detected an abrupt-onset directional tail using cohort, P90, and catastrophic-pair gates, preventing a mean-only false success claim.",
            ],
            "prohibited": [
                "Final strong-wind generalization succeeded.",
                "V4 outperformed all baselines.",
                "CART-OFMPC was Holdout validated.",
            ],
        },
    )

    final_gate = {
        "task": TASK,
        "start_head": START_HEAD,
        "protected_tree_ids": protected,
        "v3_unchanged": True,
        "v4_r0_unchanged": True,
        "v4_r1_unchanged": True,
        "cart_ofmpc_development_winner": False,
        "cart_ofmpc_mechanism_improvement": True,
        "strong_tail_failure_preserved": True,
        "new_controller_performance_executed": False,
        "v4_holdout_executed": False,
        "v5_started": False,
        "result": RESULT,
    }
    write_json(FINAL / "final_gate.json", final_gate)

    report = f"""# V4 Final Technical Report

## Final status

V4 is permanently closed as **mechanism improvement without a qualified Self method**. CART-OFMPC corrected the dominant V3 persistent-strong-wind infeasibility chain, but `cart_b_001` passed only 12/15 frozen Development gates. No candidate was frozen, no V4 Holdout sample was run, and no V5 work was started.

## What CART fixed

The V3 strong-wind diagnostic recorded residual clipping at {100 * legacy_strong['self_residual_clip_update_fraction']:.2f}%, slew activity at {100 * legacy_strong['self_slew_active_fraction']:.2f}%, and requested steady input averaging {legacy_strong['self_unscaled_steady_input_mean_max_abs_m_s2']:.2f} m/s² with an {legacy_strong['self_unscaled_steady_input_peak_max_abs_m_s2']:.2f} m/s² peak. For `cart_b_001`, the corresponding frozen strong-cohort values were about {100 * final_metric_summary['positive_evidence']['cart_strong_residual_clip_fraction']:.2f}%, {100 * final_metric_summary['positive_evidence']['cart_strong_slew_fraction']:.2f}%, and {final_metric_summary['positive_evidence']['cart_requested_steady_input_mean_max_abs_m_s2']:.4f} m/s² mean. Overall position improved {100 * near_metrics['comparisons']['overall_position_improvement_vs_full']:.2f}% versus Full-LQR; strong mean position improved {100 * near_metrics['comparisons']['strong_position_improvement_vs_full']:.2f}% versus Full-LQR and {100 * near_metrics['comparisons']['strong_position_improvement_vs_legacy']:.2f}% versus legacy Self. The paired bootstrap and normal non-regression gates passed.

This supports a narrow positive result: CART materially interrupted the old persistent-equilibrium amplification mechanism. It does not establish a V4 controller win.

## Why qualification still failed

Strong success was {100 * strong['metrics']['success_rate']:.2f}%. Strong position P90 was {strong['metrics']['position_p90_m']:.6f} m, {100 * strong['comparisons']['strong_p90_change_vs_full']:.2f}% worse than Full-LQR, and seven simultaneous-onset pairs were catastrophic: `{', '.join(sorted(catastrophic_ids))}`. The favorable mean therefore cannot override the frozen tail gates.

## Simultaneous-onset mechanism audit

Within the 12 frozen simultaneous directions, all seven negative-x targets were catastrophic and all five positive-x targets were non-catastrophic. Catastrophic mean position RMSE was {directional_analysis['catastrophic']['position_rmse_mean_m']:.6f} m versus {directional_analysis['noncatastrophic']['position_rmse_mean_m']:.6f} m; orientation RMSE was {directional_analysis['catastrophic']['orientation_rmse_mean_deg']:.3f}° versus {directional_analysis['noncatastrophic']['orientation_rmse_mean_deg']:.3f}°. Trust fell to {directional_analysis['catastrophic']['trust_mean']:.3f} versus {directional_analysis['noncatastrophic']['trust_mean']:.3f}; residual clipping rose to {100 * directional_analysis['catastrophic']['residual_clip_fraction_mean']:.2f}% versus {100 * directional_analysis['noncatastrophic']['residual_clip_fraction_mean']:.2f}%; slew activity rose to {100 * directional_analysis['catastrophic']['slew_activity_fraction_mean']:.2f}% versus {100 * directional_analysis['noncatastrophic']['slew_activity_fraction_mean']:.2f}%. QP-correction/backbone norm ratio averaged {directional_analysis['catastrophic']['qp_to_backbone_norm_ratio_mean']:.3f} versus {directional_analysis['noncatastrophic']['qp_to_backbone_norm_ratio_mean']:.3f}.

**Confirmed:** the frozen cohort has an exact x-sign partition and a low-trust/high-correction/high-slew catastrophic regime; all 18 full-bank Stage-B candidates failed both tail gates.

**Strongly supported:** simultaneous onset creates a new direction-dependent transient-coordination problem under slew-limited recovery, distinct from the repaired V3 steady infeasibility chain.

**Hypothesized:** fixed +x wind combined with negative-x target demand aligns the innovation against the target transient. This needs a new preregistered experiment. R1 did not preserve time-series traces, so tail-onset time and the backbone/QP vector angle are not observable and are not claimed as confirmed.

## Pareto and V5 decision evidence

Stage A's eight candidates used a 16-sample smoke bank and cannot share a formal frontier with Stage B. Across the 18 comparable full-bank candidates, none passed P90 or catastrophic-pair gates. `cart_b_014` increased strong success to {100 * representatives['cart_b_014']['strong_wind']['success_rate']:.2f}% and reduced the catastrophic count to {representatives['cart_b_014']['catastrophic_pair_count']}, but failed normal position/acquisition and still failed both tail gates. This is an architecture-level transient-robustness limitation expressed through a parameter trade-off, not evidence for a 27th CART configuration.

The evidence supports `RECOMMEND_NEW_RESEARCH_VERSION`, centered on abrupt simultaneous onset and frozen before any performance run. This recommendation does not authorize or start V5.

## Claims and closure

V3 Self remains a Development win not confirmed on V3 Holdout. V4 CART shows mean improvement and mechanism repair but fails Development qualification. V4 Holdout remains `UNUSED`; it must not retrospectively rescue CART. Paper was not part of V4. Final result: `{RESULT}`.
"""
    REPORT.write_text(report, encoding="utf-8", newline="\n")

    source_files = [
        R0 / "development_manifest.json",
        R0 / "holdout_manifest.json",
        R0 / "v3_failure_postmortem.json",
        R1 / "candidate_results.csv",
        R1 / "development_results.csv",
        R1 / "near_miss.json",
        R1 / "stage_b_gate_audit.json",
        R1 / "strong_wind_comparison.json",
        R1 / "normal_regime_comparison.json",
        R1 / "paired_bootstrap.json",
    ]
    output_files = sorted(path for path in FINAL.iterdir() if path.name != "evidence_manifest.json") + [REPORT]
    write_json(
        FINAL / "evidence_manifest.json",
        {
            "task": TASK,
            "analysis_mode": "OFFLINE_EXISTING_EVIDENCE_ONLY",
            "hash_normalization": "text files canonicalized to LF before SHA-256",
            "source_sha256": {str(path.relative_to(ROOT)).replace("\\", "/"): sha256(path) for path in source_files},
            "output_sha256": {str(path.relative_to(ROOT)).replace("\\", "/"): sha256(path) for path in output_files},
            "new_controller_performance_executed": False,
            "holdout_executed": False,
            "v5_started": False,
        },
    )
    print(json.dumps({"task": TASK, "result": RESULT, "v5_recommendation": "RECOMMEND_NEW_RESEARCH_VERSION"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
