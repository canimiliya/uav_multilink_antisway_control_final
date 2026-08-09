"""Apply every preregistered V8 Development hard gate and advanced-value gate."""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

try:
    from scripts import evaluate_v5_self as ev
except ImportError:
    import evaluate_v5_self as ev


ROOT = Path(__file__).resolve().parents[1]
V8 = ROOT / "reproducibility/v8"
DEV = V8 / "development"
PAPER = V8 / "paper"
TRADITIONAL = ("hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009")
STRONG = {"STRONG_PREEXISTING", "STRONG_NEAR_SIMULTANEOUS"}
ev.BOOTSTRAP_SEED = 20260825


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
    satc = ev.metrics(baseline["satc_b_027"])
    best_safety = max(ev.metrics(baseline[name])["safety_rate"] for name in TRADITIONAL)
    best_success = max(ev.metrics(baseline[name])["success_rate"] for name in TRADITIONAL)
    full_by_id = {row["sample_id"]: row for row in full_strong_rows}
    catastrophic = [
        row["sample_id"]
        for row in strong_rows
        if row["position_rmse_3d_m"] > 2.0 * full_by_id[row["sample_id"]]["position_rmse_3d_m"] + 1.0e-12
    ]
    bootstrap = ev.bootstrap(full_rows, values)
    position_improvement = ev.improve(full["position_mean_m"], overall["position_mean_m"])
    strong_improvement = ev.improve(full_strong["position_mean_m"], strong_metrics["position_mean_m"])
    strong_p90_degradation = ev.degrade(strong_metrics["position_p90_m"], full_strong["position_p90_m"])
    orientation_improvement = ev.improve(full["orientation_mean_deg"], overall["orientation_mean_deg"])
    effort_improvement = ev.improve(full["effort_mean"], overall["effort_mean"])
    runtime_improvement_vs_satc = ev.improve(satc["solve_p95_ms"], overall["solve_p95_ms"])
    acquisition_improvement = None
    if overall["acquisition_median_s"] is not None and full["acquisition_median_s"] is not None:
        acquisition_improvement = ev.improve(full["acquisition_median_s"], overall["acquisition_median_s"])
    hard = {
        "safety_no_worse_best_traditional": overall["safety_rate"] >= best_safety - 1.0e-12,
        "success_no_worse_best_traditional": overall["success_rate"] >= best_success - 1.0e-12,
        "position_improvement_vs_full_ge_5pct": position_improvement >= 0.05 - 1.0e-12,
        "acquisition_degradation_vs_full_le_5pct": (
            overall["acquisition_median_s"] is not None
            and full["acquisition_median_s"] is not None
            and overall["acquisition_median_s"] <= 1.05 * full["acquisition_median_s"] + 1.0e-12
        ),
        "paired_bootstrap_lower_gt_zero": bootstrap["lower_bound_gt_zero"],
        "strong_mean_improvement_vs_full_ge_5pct": strong_improvement >= 0.05 - 1.0e-12,
        "strong_p90_degradation_vs_full_le_10pct": strong_p90_degradation <= 0.10 + 1.0e-12,
        "zero_catastrophic_pairs": len(catastrophic) == 0,
    }
    extras = {
        "acquisition_improvement_ge_5pct": acquisition_improvement is not None and acquisition_improvement >= 0.05 - 1.0e-12,
        "orientation_improvement_ge_10pct": orientation_improvement >= 0.10 - 1.0e-12,
        "strong_position_improvement_ge_10pct": strong_improvement >= 0.10 - 1.0e-12,
        "effort_improvement_ge_10pct": effort_improvement >= 0.10 - 1.0e-12,
        "runtime_improvement_vs_satc_ge_50pct": runtime_improvement_vs_satc >= 0.50 - 1.0e-12,
    }
    advanced_value = any(extras.values())
    return {
        "candidate_id": candidate_id,
        "overall": overall,
        "strong": strong_metrics,
        "comparisons": {
            "position_improvement_vs_full": position_improvement,
            "acquisition_improvement_vs_full": acquisition_improvement,
            "strong_position_improvement_vs_full": strong_improvement,
            "strong_p90_degradation_vs_full": strong_p90_degradation,
            "orientation_improvement_vs_full": orientation_improvement,
            "effort_improvement_vs_full": effort_improvement,
            "runtime_improvement_vs_satc": runtime_improvement_vs_satc,
        },
        "hard_gates": hard,
        "hard_gate_pass_count": int(sum(hard.values())),
        "hard_gate_count": len(hard),
        "advanced_value_options": extras,
        "advanced_value_pass": advanced_value,
        "catastrophic_pair_ids": catastrophic,
        "bootstrap": bootstrap,
        "final_pass": bool(all(hard.values()) and advanced_value),
    }


def rank(value: dict) -> tuple:
    overall = value["overall"]
    return (
        -int(value["final_pass"]),
        -value["hard_gate_pass_count"],
        -overall["success_rate"],
        value["strong"]["position_p90_m"],
        overall["position_mean_m"],
        float("inf") if overall["acquisition_median_s"] is None else overall["acquisition_median_s"],
        overall["orientation_mean_deg"],
        overall["effort_mean"],
        overall["solve_p95_ms"],
        value["candidate_id"],
    )


def confirmation_matches(selected: dict, confirmation_rows: list[dict] | None, baseline: dict[str, list[dict]]) -> tuple[bool, dict | None]:
    if confirmation_rows is None:
        return False, None
    confirmation = audit(selected["candidate_id"], confirmation_rows, baseline)
    fields = ("safety_rate", "success_rate", "position_mean_m", "position_p90_m", "orientation_mean_deg", "effort_mean")
    exact = all(abs(float(selected["overall"][name]) - float(confirmation["overall"][name])) <= 1.0e-12 for name in fields)
    exact = exact and selected["overall"]["acquisition_median_s"] == confirmation["overall"]["acquisition_median_s"]
    return bool(exact and confirmation["final_pass"]), confirmation


def main() -> int:
    baseline = group(DEV / "baseline_results.csv")
    expected = {"hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009", "satc_b_027"}
    if set(baseline) != expected or any(len(rows) != 144 for rows in baseline.values()):
        raise RuntimeError("V8 carried-forward comparator evidence drift")
    candidates = group(PAPER / "stage_b_results.csv")
    if len(candidates) != 128 or any(len(rows) != 144 for rows in candidates.values()):
        raise RuntimeError("V8 Stage-B search evidence incomplete")
    audits = sorted((audit(name, rows, baseline) for name, rows in candidates.items()), key=rank)
    passes = [value for value in audits if value["final_pass"]]
    selected = passes[0] if passes else audits[0]
    stage_c_path = PAPER / "stage_c_results.csv"
    stage_c = group(stage_c_path).get(selected["candidate_id"]) if stage_c_path.exists() else None
    confirmed, confirmation = confirmation_matches(selected, stage_c, baseline)
    registry = {row["candidate_id"]: row for row in read(PAPER / "candidate_registry.json")["candidates"]}
    if passes and not confirmed:
        result = "V8_PAPER_QUALIFIED_PENDING_CONFIRMATION"
    elif passes and confirmed:
        result = "V8_XU2025_DEVELOPMENT_QUALIFIED"
    else:
        result = "V8_NO_QUALIFIED_DOUBLE_PENDULUM_PAPER_BASELINE"
    write(
        DEV / "comparator_summary.json",
        {
            "primary": "full_lqr_048",
            "participants": {name: ev.metrics(rows) for name, rows in baseline.items()},
            "evidence_carried_from_v7": True,
            "traditional_rerun": False,
            "satc_rerun": False,
            "holdout_accessed": False,
        },
    )
    write(
        PAPER / "development_gate_audit.json",
        {
            "candidate_count": len(audits),
            "sample_count_each": 144,
            "ranked": audits,
            "qualified_ids": [value["candidate_id"] for value in passes],
            "selected": selected["candidate_id"],
            "confirmation": confirmation,
            "confirmation_pass": confirmed,
            "paper_replaced": False,
            "holdout_accessed": False,
        },
    )
    write(
        PAPER / "development_final.json",
        {
            "result": result,
            "qualified": bool(passes and confirmed),
            "provisionally_qualified": bool(passes),
            "qualified_count": len(passes),
            "selected_candidate": selected["candidate_id"],
            "selected_parameters": registry[selected["candidate_id"]],
            "selected_metrics": selected,
            "confirmation_pass": confirmed,
            "authoritative_runs": {
                "comparators_carried_forward": 576,
                "stage_a_nonselection": 384,
                "stage_b": 18432,
                "stage_c_confirmation": 144 if stage_c is not None else 0,
            },
            "holdout_execution_allowed": bool(passes and confirmed),
            "holdout_executed": False,
        },
    )
    print(
        json.dumps(
            {
                "result": result,
                "qualified": len(passes),
                "confirmed": confirmed,
                "selected": selected["candidate_id"],
                "hard_gates": f"{selected['hard_gate_pass_count']}/{selected['hard_gate_count']}",
                "advanced_value": selected["advanced_value_pass"],
                "position_improvement": selected["comparisons"]["position_improvement_vs_full"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
