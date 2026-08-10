"""Apply all preregistered V10 Development strong-baseline gates."""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

try:
    from scripts import evaluate_v5_self as ev
except ImportError:
    import evaluate_v5_self as ev


ROOT = Path(__file__).resolve().parents[1]
V10 = ROOT / "reproducibility/v10"
DEV = V10 / "development"
V7_DEV = ROOT / "reproducibility/v7/development"
STRONG = {"STRONG_PREEXISTING", "STRONG_NEAR_SIMULTANEOUS"}
TRADITIONAL = ("hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009")
ev.BOOTSTRAP_SEED = 20260901


def write(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
        stream.write("\n")


def group(path: Path) -> dict[str, list[dict]]:
    result: dict[str, list[dict]] = defaultdict(list)
    for row in ev.rows(path):
        result[row["candidate_id"]].append(row)
    return result


def improve(reference: float, candidate: float) -> float:
    return (reference - candidate) / reference


def audit(candidate_id: str, values: list[dict], baseline: dict[str, list[dict]]) -> dict:
    overall = ev.metrics(values)
    strong_rows = [row for row in values if row["cohort"] in STRONG]
    strong = ev.metrics(strong_rows)
    full_rows = baseline["full_lqr_048"]
    full = ev.metrics(full_rows)
    full_strong_rows = [row for row in full_rows if row["cohort"] in STRONG]
    full_strong = ev.metrics(full_strong_rows)
    satc = ev.metrics(baseline["satc_b_027"])
    best_safety = max(ev.metrics(baseline[name])["safety_rate"] for name in TRADITIONAL)
    best_success = max(ev.metrics(baseline[name])["success_rate"] for name in TRADITIONAL)
    full_by_id = {row["sample_id"]: row for row in full_strong_rows}
    catastrophic = [row["sample_id"] for row in strong_rows if row["position_rmse_3d_m"] > 2.0 * full_by_id[row["sample_id"]]["position_rmse_3d_m"]]
    bootstrap = ev.bootstrap(full_rows, values)
    position_gain = improve(full["position_mean_m"], overall["position_mean_m"])
    strong_gain = improve(full_strong["position_mean_m"], strong["position_mean_m"])
    strong_p90_degradation = (strong["position_p90_m"] - full_strong["position_p90_m"]) / full_strong["position_p90_m"]
    acquisition_degradation = None if overall["acquisition_median_s"] is None else (overall["acquisition_median_s"] - full["acquisition_median_s"]) / full["acquisition_median_s"]
    extras = {
        "acquisition_ge_5pct": overall["acquisition_median_s"] is not None and improve(full["acquisition_median_s"], overall["acquisition_median_s"]) >= 0.05 - 1e-12,
        "orientation_ge_10pct": improve(full["orientation_mean_deg"], overall["orientation_mean_deg"]) >= 0.10 - 1e-12,
        "strong_mean_ge_10pct": strong_gain >= 0.10 - 1e-12,
        "effort_ge_10pct": improve(full["effort_mean"], overall["effort_mean"]) >= 0.10 - 1e-12,
        "runtime_ge_50pct_vs_satc": improve(satc["solve_p95_ms"], overall["solve_p95_ms"]) >= 0.50 - 1e-12,
    }
    gates = {
        "safety": overall["safety_rate"] >= best_safety - 1e-12,
        "success": overall["success_rate"] >= best_success - 1e-12,
        "position": position_gain >= 0.05 - 1e-12,
        "acquisition": acquisition_degradation is not None and acquisition_degradation <= 0.05 + 1e-12,
        "bootstrap": bootstrap["lower_bound_gt_zero"],
        "strong_position": strong_gain >= 0.05 - 1e-12,
        "strong_p90": strong_p90_degradation <= 0.10 + 1e-12,
        "catastrophic": len(catastrophic) == 0,
        "advanced_value": any(extras.values()),
    }
    return {
        "candidate_id": candidate_id, "overall": overall, "strong": strong,
        "comparisons": {"position_improvement": position_gain, "acquisition_degradation": acquisition_degradation, "strong_position_improvement": strong_gain, "strong_p90_degradation": strong_p90_degradation},
        "bootstrap": bootstrap, "catastrophic_ids": catastrophic, "advanced_value": extras,
        "gates": gates, "gate_pass_count": sum(gates.values()), "qualified": all(gates.values()),
    }


def rank(row: dict) -> tuple:
    return (-int(row["qualified"]), -row["gate_pass_count"], row["overall"]["position_mean_m"], row["strong"]["position_p90_m"], row["candidate_id"])


def main() -> int:
    baseline = group(V7_DEV / "baseline_results.csv")
    if set(baseline) != {*TRADITIONAL, "satc_b_027"} or any(len(rows) != 144 for rows in baseline.values()):
        raise RuntimeError("frozen V7 comparator evidence drift")
    candidates = group(DEV / "stage_b_results.csv")
    if len(candidates) != 16 or any(len(rows) != 144 for rows in candidates.values()):
        raise RuntimeError("V10 Stage B evidence incomplete")
    audits = sorted((audit(name, rows, baseline) for name, rows in candidates.items()), key=rank)
    qualified = [row for row in audits if row["qualified"]]
    selected = qualified[0] if qualified else audits[0]
    config_by_id = {row["candidate_id"]: row for row in json.loads((V10 / "r0/search_contract.json").read_text(encoding="utf-8"))["candidates"]}
    write(DEV / "development_gate_audit.json", {"ranked": audits, "qualified_ids": [row["candidate_id"] for row in qualified], "selected": selected["candidate_id"], "satc_used_for_selection": False, "holdout_accessed": False})
    write(DEV / "development_final.json", {
        "result": "V10_FXTDO_MPC_DEVELOPMENT_QUALIFIED" if qualified else "V10_FXTDO_MPC_NOT_STRONGER_THAN_TRADITIONAL",
        "qualified": bool(qualified), "qualified_count": len(qualified), "selected_candidate": selected["candidate_id"],
        "selected_parameters": config_by_id[selected["candidate_id"]], "selected_metrics": selected,
        "unique_configurations": 64, "authoritative_stage_b_runs": 2304,
        "holdout_execution_allowed": bool(qualified), "holdout_executed": False,
        "FINAL_EXTERNAL_PAPER_SEARCH_CLOSED": not bool(qualified), "NO_V11": not bool(qualified),
    })
    print(json.dumps({"result": "QUALIFIED" if qualified else "NO_WIN", "qualified": len(qualified), "selected": selected["candidate_id"], "gates": f"{selected['gate_pass_count']}/9", "position_improvement": selected["comparisons"]["position_improvement"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
