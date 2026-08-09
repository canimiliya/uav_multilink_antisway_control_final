"""Apply the preregistered V5 Development gates and select confirmations."""

from __future__ import annotations

import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SELF = ROOT / "reproducibility/v5/self"
STRONG = {"STRONG_PREEXISTING", "STRONG_SIMULTANEOUS", "STRONG_POST_TARGET", "STRONG_NEAR_SIMULTANEOUS"}
NORMAL = {"NORMAL_CALM", "NORMAL_CONSTANT", "NORMAL_STOCHASTIC"}
TRADITIONAL = ("hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009")
BOOTSTRAP_SEED = 20260816


def write(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def rows(path: Path) -> list[dict]:
    with path.open(encoding="utf-8", newline="") as stream: result = list(csv.DictReader(stream))
    for row in result:
        row["safe"] = row["safe"].lower() == "true"; row["task_success"] = row["task_success"].lower() == "true"
        for key in ("position_rmse_3d_m", "orientation_rmse_deg", "acquisition_time_s", "total_acceleration_effort", "solve_time_p95_ms"):
            row[key] = None if row.get(key, "") == "" else float(row[key])
    return result


def metrics(values: list[dict]) -> dict:
    acquired = [r["acquisition_time_s"] for r in values if r["task_success"] and r["acquisition_time_s"] is not None]
    position = np.asarray([r["position_rmse_3d_m"] for r in values])
    return {
        "sample_count": len(values), "safety_rate": float(np.mean([r["safe"] for r in values])), "success_rate": float(np.mean([r["task_success"] for r in values])),
        "position_mean_m": float(np.mean(position)), "position_p90_m": float(np.percentile(position, 90)), "position_p95_m": float(np.percentile(position, 95)),
        "orientation_mean_deg": float(np.mean([r["orientation_rmse_deg"] for r in values])),
        "acquisition_median_s": float(np.median(acquired)) if acquired else None,
        "effort_mean": float(np.mean([r["total_acceleration_effort"] for r in values])), "solve_p95_ms": float(max(r["solve_time_p95_ms"] for r in values)),
    }


def improve(reference: float, candidate: float) -> float: return float((reference - candidate) / reference)
def degrade(candidate: float, reference: float) -> float: return float((candidate - reference) / reference)


def bootstrap(reference: list[dict], candidate: list[dict]) -> dict:
    ref = {r["sample_id"]: r["position_rmse_3d_m"] for r in reference}; cand = {r["sample_id"]: r["position_rmse_3d_m"] for r in candidate}
    ids = sorted(ref)
    if ids != sorted(cand): raise RuntimeError("exact pairing failed")
    delta = np.asarray([ref[value] - cand[value] for value in ids])
    rng = np.random.Generator(np.random.PCG64(BOOTSTRAP_SEED))
    means = np.mean(delta[rng.integers(0, len(delta), size=(10000, len(delta)))], axis=1); ci = np.percentile(means, [2.5, 97.5])
    return {"pairs": len(delta), "resamples": 10000, "seed": BOOTSTRAP_SEED, "mean_delta_m": float(np.mean(delta)), "median_delta_m": float(np.median(delta)), "positive_pair_fraction": float(np.mean(delta > 0)), "ci_95_m": [float(ci[0]), float(ci[1])], "lower_bound_gt_zero": bool(ci[0] > 0)}


def audit(candidate_id: str, values: list[dict], baseline: dict[str, list[dict]]) -> dict:
    overall = metrics(values); strong_rows = [r for r in values if r["cohort"] in STRONG]; normal_rows = [r for r in values if r["cohort"] in NORMAL]
    strong = metrics(strong_rows); normal = metrics(normal_rows)
    full = metrics(baseline["full_lqr_048"]); full_strong_rows = [r for r in baseline["full_lqr_048"] if r["cohort"] in STRONG]; full_strong = metrics(full_strong_rows)
    legacy_strong = metrics([r for r in baseline["self_a_034"] if r["cohort"] in STRONG]); legacy_normal = metrics([r for r in baseline["self_a_034"] if r["cohort"] in NORMAL])
    best_safety = max(metrics(baseline[x])["safety_rate"] for x in TRADITIONAL); best_success = max(metrics(baseline[x])["success_rate"] for x in TRADITIONAL)
    best_strong_success = max(metrics([r for r in baseline[x] if r["cohort"] in STRONG])["success_rate"] for x in TRADITIONAL)
    full_by_id = {r["sample_id"]: r for r in full_strong_rows}
    catastrophic = [r["sample_id"] for r in strong_rows if r["position_rmse_3d_m"] > 2.0 * full_by_id[r["sample_id"]]["position_rmse_3d_m"]]
    boot = bootstrap(baseline["full_lqr_048"], values)
    overall_gates = {
        "safety": overall["safety_rate"] >= best_safety - 1e-12, "success": overall["success_rate"] >= best_success - 1e-12,
        "position": improve(full["position_mean_m"], overall["position_mean_m"]) >= .05 - 1e-12,
        "acquisition": overall["acquisition_median_s"] is not None and overall["acquisition_median_s"] <= 1.05 * full["acquisition_median_s"] + 1e-12,
        "bootstrap": boot["lower_bound_gt_zero"],
    }
    strong_gates = {
        "success": strong["success_rate"] >= best_strong_success - 1e-12,
        "position_vs_full": improve(full_strong["position_mean_m"], strong["position_mean_m"]) >= .05 - 1e-12,
        "position_vs_legacy": improve(legacy_strong["position_mean_m"], strong["position_mean_m"]) >= .30 - 1e-12,
        "p90_tail": degrade(strong["position_p90_m"], full_strong["position_p90_m"]) <= .10 + 1e-12,
        "catastrophic_pairs": len(catastrophic) == 0,
    }
    normal_gates = {
        "safety": normal["safety_rate"] >= legacy_normal["safety_rate"] - 1e-12, "success": normal["success_rate"] >= legacy_normal["success_rate"] - 1e-12,
        "position": degrade(normal["position_mean_m"], legacy_normal["position_mean_m"]) <= .05 + 1e-12,
        "acquisition": normal["acquisition_median_s"] is not None and normal["acquisition_median_s"] <= 1.10 * legacy_normal["acquisition_median_s"] + 1e-12,
    }
    directional: dict[str, dict] = {}
    for stratum in ("aligned", "opposed", "cross"):
        cand = [r for r in strong_rows if r["directional_stratum"] == stratum]; ref = [r for r in full_strong_rows if r["directional_stratum"] == stratum]
        cand_m = metrics(cand); ref_m = metrics(ref); ref_by_id = {r["sample_id"]: r for r in ref}
        cats = [r["sample_id"] for r in cand if r["position_rmse_3d_m"] > 2.0 * ref_by_id[r["sample_id"]]["position_rmse_3d_m"]]
        directional[stratum] = {"candidate": cand_m, "full_lqr": ref_m, "p90_degradation": degrade(cand_m["position_p90_m"], ref_m["position_p90_m"]), "catastrophic_pair_ids": cats, "gates": {"p90_tail": degrade(cand_m["position_p90_m"], ref_m["position_p90_m"]) <= .10 + 1e-12, "catastrophic_pairs": len(cats) == 0}}
    gates = {"overall": overall_gates, "strong_transient": strong_gates, "normal_nonregression": normal_gates, "directional": {name: data["gates"] for name, data in directional.items()}}
    atomic = [*overall_gates.values(), *strong_gates.values(), *normal_gates.values(), *[value for data in directional.values() for value in data["gates"].values()]]
    return {"candidate_id": candidate_id, "overall": overall, "strong": strong, "normal": normal, "directional": directional, "catastrophic_pair_ids": catastrophic, "bootstrap": boot, "gates": gates, "hard_gate_pass_count": int(sum(atomic)), "hard_gate_count": len(atomic), "final_pass": bool(all(atomic)), "comparisons": {"overall_position_improvement_vs_full": improve(full["position_mean_m"], overall["position_mean_m"]), "strong_position_improvement_vs_full": improve(full_strong["position_mean_m"], strong["position_mean_m"]), "strong_position_improvement_vs_legacy": improve(legacy_strong["position_mean_m"], strong["position_mean_m"]), "normal_position_degradation_vs_legacy": degrade(normal["position_mean_m"], legacy_normal["position_mean_m"]), "normal_acquisition_degradation_vs_legacy": degrade(normal["acquisition_median_s"], legacy_normal["acquisition_median_s"]), "worst_directional_p90_degradation": max(data["p90_degradation"] for data in directional.values())}}


def rank_key(value: dict) -> tuple:
    return (-int(value["final_pass"]), -value["hard_gate_pass_count"], value["comparisons"]["worst_directional_p90_degradation"], value["strong"]["position_mean_m"], value["overall"]["position_mean_m"], value["normal"]["position_mean_m"], float("inf") if value["overall"]["acquisition_median_s"] is None else value["overall"]["acquisition_median_s"], value["overall"]["effort_mean"], value["overall"]["solve_p95_ms"], value["candidate_id"])


def main() -> int:
    baseline: dict[str, list[dict]] = defaultdict(list)
    for row in rows(SELF / "baseline_development_results.csv"): baseline[row["candidate_id"]].append(row)
    candidates: dict[str, list[dict]] = defaultdict(list)
    for row in rows(SELF / "stage_b_results.csv"): candidates[row["candidate_id"]].append(row)
    audits = sorted((audit(name, value, baseline) for name, value in candidates.items()), key=rank_key)
    selected = [value["candidate_id"] for value in audits[:8]]
    write(SELF / "stage_b_gate_audit.json", {"candidate_count": len(audits), "ranked": audits, "holdout_executed": False})
    write(SELF / "search_history.json", {"protocol_frozen_before_performance": True, "stage_a": {"candidate_count": 12, "sample_count_each": 24, "selection_authority": False}, "stage_b": {"candidate_count": 36, "sample_count_each": 120}, "stage_c_selected_candidate_ids": selected, "stage_c_reuses_stage_b_parameters": True, "unique_configuration_count": 48, "maximum_unique_configurations": 64, "holdout_executed": False})
    print(json.dumps({"stage_b_passes": [v["candidate_id"] for v in audits if v["final_pass"]], "stage_c": selected, "top": [{"id": v["candidate_id"], "gates": f"{v['hard_gate_pass_count']}/{v['hard_gate_count']}", "pass": v["final_pass"]} for v in audits[:8]]}, indent=2))
    return 0


if __name__ == "__main__": raise SystemExit(main())
