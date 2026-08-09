"""Apply the preregistered V6 Paper Development qualification gates."""

from __future__ import annotations

import hashlib
import json
from collections import defaultdict
from pathlib import Path

try:
    from scripts import evaluate_v5_self as ev
except ImportError:
    import evaluate_v5_self as ev


ROOT = Path(__file__).resolve().parents[1]
V6 = ROOT / "reproducibility/v6"
DEV = V6 / "development"
PAPERS = V6 / "papers"
TRADITIONAL = ("hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009")
STRONG = {"STRONG_PREEXISTING", "STRONG_NEAR_SIMULTANEOUS"}
NORMAL = {"NORMAL_CALM", "NORMAL_CONSTANT", "NORMAL_STOCHASTIC"}
ev.BOOTSTRAP_SEED = 20260819


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def group(path: Path) -> dict[str, list[dict]]:
    result: dict[str, list[dict]] = defaultdict(list)
    for row in ev.rows(path):
        result[row["candidate_id"]].append(row)
    return result


def subset(values: list[dict], name: str) -> list[dict]:
    if name == "overall":
        return values
    if name == "normal":
        return [row for row in values if row["cohort"] in NORMAL]
    if name == "strong":
        return [row for row in values if row["cohort"] in STRONG]
    return [row for row in values if row["cohort"] in STRONG and row["directional_stratum"] == name]


def audit(candidate_id: str, values: list[dict], baseline: dict[str, list[dict]]) -> dict:
    overall = ev.metrics(values)
    normal = ev.metrics(subset(values, "normal"))
    strong_rows = subset(values, "strong")
    strong = ev.metrics(strong_rows)
    full_rows = baseline["full_lqr_048"]
    full = ev.metrics(full_rows)
    full_normal = ev.metrics(subset(full_rows, "normal"))
    full_strong_rows = subset(full_rows, "strong")
    full_strong = ev.metrics(full_strong_rows)
    best_safety = max(ev.metrics(baseline[name])["safety_rate"] for name in TRADITIONAL)
    best_success = max(ev.metrics(baseline[name])["success_rate"] for name in TRADITIONAL)
    full_strong_by_id = {row["sample_id"]: row for row in full_strong_rows}
    catastrophic = [
        row["sample_id"] for row in strong_rows
        if row["position_rmse_3d_m"] > 2.0 * full_strong_by_id[row["sample_id"]]["position_rmse_3d_m"]
    ]
    bootstrap = {name: ev.bootstrap(subset(full_rows, name), subset(values, name)) for name in ("overall", "normal", "strong", "aligned", "opposed", "cross")}
    acquisition_degradation = None
    if overall["acquisition_median_s"] is not None:
        acquisition_degradation = ev.degrade(overall["acquisition_median_s"], full["acquisition_median_s"])
    normal_acquisition_improvement = None
    if normal["acquisition_median_s"] is not None:
        normal_acquisition_improvement = ev.improve(full_normal["acquisition_median_s"], normal["acquisition_median_s"])
    position_improvement = ev.improve(full["position_mean_m"], overall["position_mean_m"])
    strong_mean_improvement = ev.improve(full_strong["position_mean_m"], strong["position_mean_m"])
    strong_p90_degradation = ev.degrade(strong["position_p90_m"], full_strong["position_p90_m"])
    meaningful = strong_mean_improvement >= 0.05 - 1.0e-12 or (
        normal_acquisition_improvement is not None and normal_acquisition_improvement >= 0.05 - 1.0e-12
    )
    gates = {
        "safety_no_worse_best_traditional": overall["safety_rate"] >= best_safety - 1.0e-12,
        "success_no_worse_best_traditional": overall["success_rate"] >= best_success - 1.0e-12,
        "position_improvement_vs_full": position_improvement >= 0.05 - 1.0e-12,
        "acquisition_nonregression_vs_full": acquisition_degradation is not None and acquisition_degradation <= 0.05 + 1.0e-12,
        "strong_p90_nonregression_vs_full": strong_p90_degradation <= 0.10 + 1.0e-12,
        "zero_catastrophic_pairs": len(catastrophic) == 0,
        "meaningful_extra_improvement": meaningful,
        "paired_bootstrap_lower_gt_zero": bootstrap["overall"]["lower_bound_gt_zero"],
    }
    directional = {
        name: {
            "candidate": ev.metrics(subset(values, name)),
            "full_lqr": ev.metrics(subset(full_rows, name)),
            "bootstrap": bootstrap[name],
        }
        for name in ("aligned", "opposed", "cross")
    }
    return {
        "candidate_id": candidate_id, "overall": overall, "normal": normal, "strong": strong,
        "directional": directional, "bootstrap": bootstrap,
        "comparisons": {
            "position_improvement_vs_full": position_improvement,
            "acquisition_degradation_vs_full": acquisition_degradation,
            "strong_mean_improvement_vs_full": strong_mean_improvement,
            "strong_p90_degradation_vs_full": strong_p90_degradation,
            "normal_acquisition_improvement_vs_full": normal_acquisition_improvement,
        },
        "catastrophic_pair_ids": catastrophic, "gates": gates,
        "gate_pass_count": int(sum(gates.values())), "gate_count": len(gates), "final_pass": bool(all(gates.values())),
    }


def rank(value: dict) -> tuple:
    overall = value["overall"]
    return (
        -int(value["final_pass"]), -value["gate_pass_count"], -overall["safety_rate"],
        overall["position_mean_m"], float("inf") if overall["acquisition_median_s"] is None else overall["acquisition_median_s"],
        -overall["success_rate"], overall["effort_mean"], overall["solve_p95_ms"], value["candidate_id"],
    )


def main() -> int:
    baseline = group(DEV / "baseline_results.csv")
    if set(baseline) != {"hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009", "satc_b_027"}:
        raise RuntimeError("V6 comparator set drift")
    comparator = {name: {cohort: ev.metrics(subset(values, cohort)) for cohort in ("overall", "normal", "strong", "aligned", "opposed", "cross")} for name, values in baseline.items()}
    write(DEV / "comparator_gate.json", {
        "primary": "full_lqr_048", "best_traditional_safety": max(comparator[name]["overall"]["safety_rate"] for name in TRADITIONAL),
        "best_traditional_success": max(comparator[name]["overall"]["success_rate"] for name in TRADITIONAL),
        "participants": comparator, "satc_role": "FROZEN_REFERENCE_NOT_QUALIFICATION_BASELINE", "holdout_accessed": False,
    })
    protocol = read(PAPERS / "development_protocol.json")
    implementation = read(PAPERS / "implementation_freeze.json")
    expected_hash = implementation["sha256"]["src/uav_sway/v6/paper_controllers.py"]
    if sha(ROOT / "src/uav_sway/v6/paper_controllers.py") != expected_hash:
        raise RuntimeError("Paper implementation changed after freeze")
    qualified = []
    paper_results = {}
    for paper_id in ("YU2026", "SEP2026"):
        folder = PAPERS / paper_id.lower()
        candidates = group(folder / "stage_b_results.csv")
        if len(candidates) != 24 or any(len(rows) != 120 for rows in candidates.values()):
            raise RuntimeError(f"{paper_id} result budget drift")
        audits = sorted((audit(name, rows, baseline) for name, rows in candidates.items()), key=rank)
        passes = [value for value in audits if value["final_pass"]]
        parameters = {row["candidate_id"]: row for row in protocol["papers"][paper_id]["stage_b"]}
        if passes:
            best = passes[0]
            status = "PAPER_DEVELOPMENT_QUALIFIED_FROZEN"
            holdout = "ELIGIBLE_NOT_RUN"
            write(folder / "paper_freeze.json", {
                "paper": paper_id, "status": status, "candidate_id": best["candidate_id"],
                "parameters": parameters[best["candidate_id"]], "metrics": best,
                "mutable": False, "holdout_status": holdout,
            })
            qualified.append({"paper": paper_id, "candidate_id": best["candidate_id"]})
        else:
            best = audits[0]
            status = "PAPER_DEVELOPMENT_INELIGIBLE"
            holdout = "NOT_RUN_DEVELOPMENT_INELIGIBLE"
        write(folder / "development_gate_audit.json", {
            "paper": paper_id, "candidate_count": len(audits), "sample_count_each": 120,
            "ranked": audits, "qualified_ids": [value["candidate_id"] for value in passes],
            "best_candidate": best["candidate_id"], "final_status": status, "holdout_status": holdout,
            "search_budget_extended": False, "replacement_paper_selected": False,
        })
        write(folder / "paper_final_status.json", {
            "paper": paper_id, "development_status": status, "candidate": best["candidate_id"],
            "holdout_status": holdout, "claims_about_original_authors_on_v6_benchmark": False,
        })
        paper_results[paper_id] = {"status": status, "best": best, "holdout": holdout}
    final_status = "V6_PAPERS_DEVELOPMENT_QUALIFIED" if qualified else "V6_NO_QUALIFIED_RECENT_PAPER_BASELINE"
    write(DEV / "development_final.json", {
        "result": final_status, "qualified_paper_count": len(qualified), "qualified": qualified,
        "papers": {name: {"development_status": value["status"], "best_candidate": value["best"]["candidate_id"], "holdout_status": value["holdout"]} for name, value in paper_results.items()},
        "authoritative_runs": {"comparators": 480, "paper_stage_a": 384, "paper_stage_b": 5760, "total": 6624},
        "technical_retries": [{
            "phase": "comparator launcher", "reason": "initial orchestration command timeout after one second",
            "resolution": "same implementation commit, parameters, and Development samples completed using persistent launcher/cache; no contract or scientific setting changed",
        }],
        "holdout_execution_allowed": bool(qualified), "holdout_executed": False,
    })
    write(V6 / "holdout_status.json", {
        "executed": False, "reason": "ZERO_QUALIFIED_PAPERS" if not qualified else "QUALIFIED_PAPERS_AWAIT_PROTOCOL_FREEZE",
        "v6_holdout_manifest_execution_allowed_at_r0": False,
        "v6_holdout_results_exist": False,
    })
    print(json.dumps({"result": final_status, "qualified": len(qualified), "papers": {name: {"best": value["best"]["candidate_id"], "gates": f"{value['best']['gate_pass_count']}/{value['best']['gate_count']}"} for name, value in paper_results.items()}, "holdout_executed": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
