"""Apply the preregistered V7 Paper Development strong-baseline gates."""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

try:
    from scripts import evaluate_v5_self as ev
except ImportError:
    import evaluate_v5_self as ev


ROOT = Path(__file__).resolve().parents[1]
V7 = ROOT / "reproducibility/v7"
DEV = V7 / "development"
PAPER = V7 / "paper"
TRADITIONAL = ("hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009")
STRONG = {"STRONG_PREEXISTING", "STRONG_NEAR_SIMULTANEOUS"}
ev.BOOTSTRAP_SEED = 20260823


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def group(path: Path) -> dict[str, list[dict]]:
    result: dict[str, list[dict]] = defaultdict(list)
    for row in ev.rows(path):
        result[row["candidate_id"]].append(row)
    return result


def strong(values: list[dict]) -> list[dict]:
    return [row for row in values if row["cohort"] in STRONG]


def audit(candidate_id: str, values: list[dict], baseline: dict[str, list[dict]]) -> dict:
    overall = ev.metrics(values)
    strong_rows = strong(values)
    strong_metrics = ev.metrics(strong_rows)
    full_rows = baseline["full_lqr_048"]
    full = ev.metrics(full_rows)
    full_strong_rows = strong(full_rows)
    full_strong = ev.metrics(full_strong_rows)
    best_safety = max(ev.metrics(baseline[name])["safety_rate"] for name in TRADITIONAL)
    best_success = max(ev.metrics(baseline[name])["success_rate"] for name in TRADITIONAL)
    full_by_id = {row["sample_id"]: row for row in full_strong_rows}
    catastrophic = [row["sample_id"] for row in strong_rows if row["position_rmse_3d_m"] > 2.0 * full_by_id[row["sample_id"]]["position_rmse_3d_m"]]
    bootstrap = ev.bootstrap(full_rows, values)
    position_improvement = ev.improve(full["position_mean_m"], overall["position_mean_m"])
    strong_improvement = ev.improve(full_strong["position_mean_m"], strong_metrics["position_mean_m"])
    strong_p90_degradation = ev.degrade(strong_metrics["position_p90_m"], full_strong["position_p90_m"])
    orientation_improvement = ev.improve(full["orientation_mean_deg"], overall["orientation_mean_deg"])
    effort_improvement = ev.improve(full["effort_mean"], overall["effort_mean"])
    acquisition_improvement = None
    if overall["acquisition_median_s"] is not None and full["acquisition_median_s"] is not None:
        acquisition_improvement = ev.improve(full["acquisition_median_s"], overall["acquisition_median_s"])
    extras = {
        "acquisition_improvement_ge_5pct": acquisition_improvement is not None and acquisition_improvement >= 0.05 - 1.0e-12,
        "strong_position_improvement_ge_10pct": strong_improvement >= 0.10 - 1.0e-12,
        "orientation_improvement_ge_10pct": orientation_improvement >= 0.10 - 1.0e-12,
        "effort_improvement_ge_10pct": effort_improvement >= 0.10 - 1.0e-12,
    }
    gates = {
        "safety_no_worse_best_traditional": overall["safety_rate"] >= best_safety - 1.0e-12,
        "success_no_worse_best_traditional": overall["success_rate"] >= best_success - 1.0e-12,
        "position_improvement_vs_full_ge_5pct": position_improvement >= 0.05 - 1.0e-12,
        "paired_bootstrap_lower_gt_zero": bootstrap["lower_bound_gt_zero"],
        "strong_p90_nonregression": strong_p90_degradation <= 0.10 + 1.0e-12,
        "zero_catastrophic_pairs": len(catastrophic) == 0,
        "meaningful_extra": any(extras.values()),
    }
    return {
        "candidate_id": candidate_id, "overall": overall, "strong": strong_metrics,
        "comparisons": {
            "position_improvement_vs_full": position_improvement,
            "acquisition_improvement_vs_full": acquisition_improvement,
            "strong_position_improvement_vs_full": strong_improvement,
            "strong_p90_degradation_vs_full": strong_p90_degradation,
            "orientation_improvement_vs_full": orientation_improvement,
            "effort_improvement_vs_full": effort_improvement,
        },
        "meaningful_extra_options": extras, "catastrophic_pair_ids": catastrophic,
        "bootstrap": bootstrap, "gates": gates,
        "gate_pass_count": int(sum(gates.values())), "gate_count": len(gates), "final_pass": bool(all(gates.values())),
    }


def rank(value: dict) -> tuple:
    overall = value["overall"]
    return (
        -int(value["final_pass"]), -value["gate_pass_count"], value["strong"]["position_p90_m"],
        overall["position_mean_m"], float("inf") if overall["acquisition_median_s"] is None else overall["acquisition_median_s"],
        overall["orientation_mean_deg"], overall["effort_mean"], overall["solve_p95_ms"], value["candidate_id"],
    )


def main() -> int:
    baseline = group(DEV / "baseline_results.csv")
    expected = {"hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009", "satc_b_027"}
    if set(baseline) != expected or any(len(rows) != 144 for rows in baseline.values()):
        raise RuntimeError("V7 comparator evidence drift")
    candidates = group(PAPER / "stage_b_results.csv")
    for name, rows in group(PAPER / "stage_c_results.csv").items():
        candidates[name].extend(rows)
    if len(candidates) != 72 or any(len(rows) != 144 for rows in candidates.values()):
        raise RuntimeError("V7 Paper search evidence drift")
    audits = sorted((audit(name, rows, baseline) for name, rows in candidates.items()), key=rank)
    passes = [value for value in audits if value["final_pass"]]
    selected = passes[0] if passes else audits[0]
    protocol = read(PAPER / "development_protocol.json")
    parameters = {row["candidate_id"]: row for stage in ("stage_b", "stage_c") for row in protocol["stages"][stage]["candidates"]}
    write(DEV / "comparator_summary.json", {
        "primary": "full_lqr_048", "participants": {name: ev.metrics(rows) for name, rows in baseline.items()},
        "satc_role": "FROZEN_REFERENCE_NOT_QUALIFICATION_BASELINE", "holdout_accessed": False,
    })
    write(PAPER / "development_gate_audit.json", {
        "candidate_count": len(audits), "sample_count_each": 144, "ranked": audits,
        "qualified_ids": [value["candidate_id"] for value in passes], "selected": selected["candidate_id"],
        "selection_rule": protocol["selection_rule"], "search_budget_extended": False,
        "paper_replaced": False, "satc_used_for_selection": False, "holdout_accessed": False,
    })
    write(PAPER / "development_final.json", {
        "result": "V7_KANG2026_DEVELOPMENT_QUALIFIED" if passes else "KANG2026_ADAPTATION_NOT_STRONGER_THAN_TRADITIONAL",
        "qualified": bool(passes), "qualified_count": len(passes), "selected_candidate": selected["candidate_id"],
        "selected_parameters": parameters[selected["candidate_id"]], "selected_metrics": selected,
        "authoritative_runs": {"comparators": 576, "stage_a": 576, "stage_b": 6912, "stage_c": 3456, "total": 11520},
        "holdout_execution_allowed": bool(passes), "holdout_executed": False,
    })
    print(json.dumps({
        "result": "QUALIFIED" if passes else "NO_WIN", "qualified": len(passes),
        "selected": selected["candidate_id"], "gates": f"{selected['gate_pass_count']}/{selected['gate_count']}",
        "position_improvement": selected["comparisons"]["position_improvement_vs_full"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

