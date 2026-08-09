"""Apply the frozen Overall gate and freeze or close the sole V5 Paper route."""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

try:
    from scripts import evaluate_v5_self as ev
except ImportError:
    import evaluate_v5_self as ev


ROOT = Path(__file__).resolve().parents[1]
PAPER = ROOT / "reproducibility/v5/paper"
SELF = ROOT / "reproducibility/v5/self"
TRADITIONAL = ev.TRADITIONAL


def read(path: Path) -> dict: return json.loads(path.read_text(encoding="utf-8"))
def write(name: str, value: object) -> None: (PAPER / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def audit(candidate_id: str, values: list[dict], baseline: dict[str, list[dict]]) -> dict:
    metric = ev.metrics(values); full = ev.metrics(baseline["full_lqr_048"]); boot = ev.bootstrap(baseline["full_lqr_048"], values)
    best_safety = max(ev.metrics(baseline[x])["safety_rate"] for x in TRADITIONAL); best_success = max(ev.metrics(baseline[x])["success_rate"] for x in TRADITIONAL)
    gates = {"safety": metric["safety_rate"] >= best_safety - 1e-12, "success": metric["success_rate"] >= best_success - 1e-12, "position": ev.improve(full["position_mean_m"], metric["position_mean_m"]) >= .05 - 1e-12, "acquisition": metric["acquisition_median_s"] is not None and metric["acquisition_median_s"] <= 1.05 * full["acquisition_median_s"] + 1e-12, "bootstrap": boot["lower_bound_gt_zero"]}
    strong = ev.metrics([r for r in values if r["cohort"] in ev.STRONG]); normal = ev.metrics([r for r in values if r["cohort"] in ev.NORMAL])
    return {"candidate_id": candidate_id, "overall": metric, "strong": strong, "normal": normal, "bootstrap": boot, "gates": gates, "gate_pass_count": sum(gates.values()), "gate_count": len(gates), "final_pass": all(gates.values()), "position_improvement_vs_full": ev.improve(full["position_mean_m"], metric["position_mean_m"]), "acquisition_degradation_vs_full": ev.degrade(metric["acquisition_median_s"], full["acquisition_median_s"])}


def rank(value: dict) -> tuple: return (-int(value["final_pass"]), -value["gate_pass_count"], value["overall"]["position_mean_m"], float("inf") if value["overall"]["acquisition_median_s"] is None else value["overall"]["acquisition_median_s"], -value["overall"]["success_rate"], value["overall"]["effort_mean"], value["overall"]["solve_p95_ms"], value["candidate_id"])


def main() -> int:
    baseline: dict[str, list[dict]] = defaultdict(list)
    for row in ev.rows(SELF / "baseline_development_results.csv"): baseline[row["candidate_id"]].append(row)
    candidates: dict[str, list[dict]] = defaultdict(list)
    for row in ev.rows(PAPER / "stage_b_results.csv"): candidates[row["candidate_id"]].append(row)
    audits = sorted((audit(name, rows, baseline) for name, rows in candidates.items()), key=rank); passes = [row for row in audits if row["final_pass"]]
    protocol = read(PAPER / "adaptation_protocol.json")
    if passes:
        selected = passes[0]; parameters = next(row for row in protocol["search"]["stage_b"]["candidates"] if row["candidate_id"] == selected["candidate_id"])
        status = "PAPER_DEVELOPMENT_QUALIFIED_FROZEN"; holdout = "ELIGIBLE_NOT_YET_RUN"; write("paper_freeze.json", {"status": status, "candidate_id": selected["candidate_id"], "parameters": parameters, "metrics": selected, "mutable": False, "holdout_status": holdout})
    else:
        selected = audits[0]; status = "PAPER_DEVELOPMENT_INELIGIBLE"; holdout = "NOT_RUN_DEVELOPMENT_INELIGIBLE"; write("near_miss.json", {"status": status, "best_candidate": selected, "no_replacement_paper": True, "holdout_status": holdout})
    write("development_gate_audit.json", {"candidate_count": len(audits), "ranked": audits, "qualified_ids": [row["candidate_id"] for row in passes], "final_status": status, "holdout_status": holdout})
    write("paper_final_status.json", {"source": "Jirousek et al., ICINCO 2025, DOI 10.5220/0013789200003982", "method": "incremental MPC adapted to frozen five-link model", "development_status": status, "candidate": selected["candidate_id"], "holdout_status": holdout, "claims_about_original_authors_on_v5_benchmark": False})
    print(json.dumps({"status": status, "candidate": selected["candidate_id"], "gates": f"{selected['gate_pass_count']}/{selected['gate_count']}", "holdout": holdout}, indent=2)); return 0


if __name__ == "__main__": raise SystemExit(main())
