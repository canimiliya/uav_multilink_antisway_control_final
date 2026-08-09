"""Apply the frozen V4-R1 gates and select Stage-C confirmations."""

from __future__ import annotations

import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
R1 = ROOT / "reproducibility/v4/r1"
STRONG = {"STRONG_PREEXISTING", "STRONG_SIMULTANEOUS", "STRONG_POST_TARGET"}
NORMAL = {"NORMAL_CALM", "NORMAL_CONSTANT", "NORMAL_STOCHASTIC"}


def write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def read_rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        for key in ("safe", "task_success"):
            row[key] = row[key].lower() == "true"
        for key in ("position_rmse_3d_m", "acquisition_time_s", "total_acceleration_effort", "solve_time_p95_ms"):
            row[key] = None if row.get(key, "") == "" else float(row[key])
    return rows


def metrics(rows: list[dict]) -> dict:
    acquired = [row["acquisition_time_s"] for row in rows if row["task_success"] and row["acquisition_time_s"] is not None]
    position = np.asarray([row["position_rmse_3d_m"] for row in rows])
    return {
        "sample_count": len(rows),
        "safety_rate": float(np.mean([row["safe"] for row in rows])),
        "success_rate": float(np.mean([row["task_success"] for row in rows])),
        "position_mean_m": float(np.mean(position)),
        "position_p90_m": float(np.percentile(position, 90)),
        "position_p95_m": float(np.percentile(position, 95)),
        "acquisition_median_s": float(np.median(acquired)) if acquired else None,
        "effort_mean": float(np.mean([row["total_acceleration_effort"] for row in rows])),
        "solve_p95_ms": float(max(row["solve_time_p95_ms"] for row in rows)),
    }


def fraction_improvement(reference: float, candidate: float) -> float:
    return float((reference - candidate) / reference)


def fraction_change(candidate: float, reference: float) -> float:
    return float((candidate - reference) / reference)


def paired_bootstrap(reference: list[dict], candidate: list[dict]) -> dict:
    ref = {row["sample_id"]: row["position_rmse_3d_m"] for row in reference}
    cand = {row["sample_id"]: row["position_rmse_3d_m"] for row in candidate}
    ids = sorted(ref)
    if ids != sorted(cand):
        raise RuntimeError("exact sample-id pairing failed")
    delta = np.asarray([ref[value] - cand[value] for value in ids])
    rng = np.random.Generator(np.random.PCG64(20260812))
    means = np.mean(delta[rng.integers(0, len(delta), size=(10000, len(delta)))], axis=1)
    ci = np.percentile(means, [2.5, 97.5])
    return {
        "pairs": len(delta), "resamples": 10000, "seed": 20260812,
        "mean_delta_m": float(np.mean(delta)), "median_delta_m": float(np.median(delta)),
        "positive_pair_fraction": float(np.mean(delta > 0.0)), "ci_95_m": [float(ci[0]), float(ci[1])],
        "lower_bound_gt_zero": bool(ci[0] > 0.0),
    }


def audit_candidate(candidate_id: str, rows: list[dict], baselines: dict[str, list[dict]]) -> dict:
    traditional_ids = ("hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009")
    overall = metrics(rows); strong_rows = [row for row in rows if row["cohort"] in STRONG]; normal_rows = [row for row in rows if row["cohort"] in NORMAL]
    strong = metrics(strong_rows); normal = metrics(normal_rows)
    full = metrics(baselines["full_lqr_048"]); full_strong = metrics([row for row in baselines["full_lqr_048"] if row["cohort"] in STRONG])
    legacy_strong = metrics([row for row in baselines["self_a_034"] if row["cohort"] in STRONG]); legacy_normal = metrics([row for row in baselines["self_a_034"] if row["cohort"] in NORMAL])
    best_safety = max(metrics(baselines[value])["safety_rate"] for value in traditional_ids); best_success = max(metrics(baselines[value])["success_rate"] for value in traditional_ids)
    best_strong_safety = max(metrics([row for row in baselines[value] if row["cohort"] in STRONG])["safety_rate"] for value in traditional_ids)
    best_strong_success = max(metrics([row for row in baselines[value] if row["cohort"] in STRONG])["success_rate"] for value in traditional_ids)
    full_by_id = {row["sample_id"]: row for row in baselines["full_lqr_048"] if row["cohort"] in STRONG}
    catastrophic = [row["sample_id"] for row in strong_rows if row["position_rmse_3d_m"] > 2.0 * full_by_id[row["sample_id"]]["position_rmse_3d_m"]]
    bootstrap = paired_bootstrap(baselines["full_lqr_048"], rows)
    overall_gates = {
        "safety": overall["safety_rate"] >= best_safety - 1.0e-12,
        "success": overall["success_rate"] >= best_success - 1.0e-12,
        "position": fraction_improvement(full["position_mean_m"], overall["position_mean_m"]) >= 0.05 - 1.0e-12,
        "acquisition": overall["acquisition_median_s"] is not None and overall["acquisition_median_s"] <= 1.05 * full["acquisition_median_s"] + 1.0e-12,
        "bootstrap": bootstrap["lower_bound_gt_zero"],
    }
    strong_gates = {
        "safety": strong["safety_rate"] >= best_strong_safety - 1.0e-12,
        "success": strong["success_rate"] >= best_strong_success - 1.0e-12,
        "position_vs_full": fraction_improvement(full_strong["position_mean_m"], strong["position_mean_m"]) >= 0.05 - 1.0e-12,
        "position_vs_legacy": fraction_improvement(legacy_strong["position_mean_m"], strong["position_mean_m"]) >= 0.30 - 1.0e-12,
        "p90_tail": fraction_change(strong["position_p90_m"], full_strong["position_p90_m"]) <= 0.10 + 1.0e-12,
        "catastrophic_pairs": len(catastrophic) == 0,
    }
    normal_gates = {
        "safety": normal["safety_rate"] >= legacy_normal["safety_rate"] - 1.0e-12,
        "success": normal["success_rate"] >= legacy_normal["success_rate"] - 1.0e-12,
        "position": fraction_change(normal["position_mean_m"], legacy_normal["position_mean_m"]) <= 0.05 + 1.0e-12,
        "acquisition": normal["acquisition_median_s"] is not None and normal["acquisition_median_s"] <= 1.10 * legacy_normal["acquisition_median_s"] + 1.0e-12,
    }
    gates = {"overall": overall_gates, "strong_wind": strong_gates, "normal_nonregression": normal_gates}
    all_atomic = [value for section in gates.values() for value in section.values()]
    return {
        "candidate_id": candidate_id, "overall": overall, "strong_wind": strong, "normal_regime": normal,
        "comparisons": {
            "overall_position_improvement_vs_full": fraction_improvement(full["position_mean_m"], overall["position_mean_m"]),
            "overall_acquisition_change_vs_full": fraction_change(overall["acquisition_median_s"], full["acquisition_median_s"]),
            "strong_position_improvement_vs_full": fraction_improvement(full_strong["position_mean_m"], strong["position_mean_m"]),
            "strong_position_improvement_vs_legacy": fraction_improvement(legacy_strong["position_mean_m"], strong["position_mean_m"]),
            "strong_p90_change_vs_full": fraction_change(strong["position_p90_m"], full_strong["position_p90_m"]),
            "normal_position_change_vs_legacy": fraction_change(normal["position_mean_m"], legacy_normal["position_mean_m"]),
            "normal_acquisition_change_vs_legacy": fraction_change(normal["acquisition_median_s"], legacy_normal["acquisition_median_s"]),
        },
        "catastrophic_pair_ids": catastrophic, "bootstrap": bootstrap, "gates": gates,
        "hard_gate_pass_count": int(sum(all_atomic)), "hard_gate_count": len(all_atomic), "final_pass": bool(all(all_atomic)),
    }


def rank_key(audit: dict) -> tuple:
    # Frozen rule: pass status, gate count, strong position, overall position,
    # normal position, acquisition, effort, runtime, stable candidate id.
    return (
        -int(audit["final_pass"]), -audit["hard_gate_pass_count"], audit["strong_wind"]["position_mean_m"],
        audit["overall"]["position_mean_m"], audit["normal_regime"]["position_mean_m"],
        float("inf") if audit["overall"]["acquisition_median_s"] is None else audit["overall"]["acquisition_median_s"],
        audit["overall"]["effort_mean"], audit["overall"]["solve_p95_ms"], audit["candidate_id"],
    )


def main() -> int:
    baseline_rows = read_rows(R1 / "baseline_development_results.csv")
    baselines: dict[str, list[dict]] = defaultdict(list)
    for row in baseline_rows: baselines[row["candidate_id"]].append(row)
    stage_rows = read_rows(R1 / "stage_b_results.csv")
    candidates: dict[str, list[dict]] = defaultdict(list)
    for row in stage_rows: candidates[row["candidate_id"]].append(row)
    audits = sorted((audit_candidate(candidate_id, rows, baselines) for candidate_id, rows in candidates.items()), key=rank_key)
    selected = [row["candidate_id"] for row in audits[:6]]
    write_json(R1 / "stage_b_gate_audit.json", {"source": "frozen Stage-B 94-sample Development", "candidate_count": len(audits), "ranked": audits, "holdout_executed": False})
    write_json(R1 / "search_history.json", {
        "protocol_frozen_before_performance": True, "stage_a": {"candidate_count": 8, "sample_count_each": 16, "selection_authority": False},
        "stage_b": {"candidate_count": 18, "sample_count_each": 94, "ranking_rule": "development_protocol.json"},
        "stage_c_selected_candidate_ids": selected, "stage_c_selection_source": "top six Stage-B configurations; parameters unchanged",
        "unique_configuration_count": 26, "maximum_unique_configurations": 32, "holdout_executed": False,
    })
    print(json.dumps({"stage_c": selected, "stage_b_passes": [row["candidate_id"] for row in audits if row["final_pass"]], "top": [{"id": row["candidate_id"], "gates": f"{row['hard_gate_pass_count']}/{row['hard_gate_count']}", "pass": row["final_pass"]} for row in audits[:6]]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
