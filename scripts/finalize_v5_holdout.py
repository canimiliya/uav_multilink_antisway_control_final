"""Apply frozen V5 Holdout statistics and final claim levels."""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

try:
    from scripts import evaluate_v5_self as ev
except ImportError:
    import evaluate_v5_self as ev


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v5/holdout"
SELF_ID = "satc_b_027"


def write(name: str, value: object) -> None: (OUT / name).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def main() -> int:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in ev.rows(OUT / "holdout_results.csv"): grouped[row["candidate_id"]].append(row)
    candidate = ev.audit(SELF_ID, grouped[SELF_ID], grouped)
    strict = candidate["final_pass"]; overall = all(candidate["gates"]["overall"].values())
    final_status = "V5_SELF_STRICT_HOLDOUT_WIN" if strict else "V5_SELF_OVERALL_HOLDOUT_WIN" if overall else "V5_SELF_DEVELOPMENT_WIN_NOT_CONFIRMED_ON_HOLDOUT"
    subsets = {
        "overall": lambda r: True, "normal": lambda r: r["cohort"] in ev.NORMAL, "strong": lambda r: r["cohort"] in ev.STRONG,
        "aligned": lambda r: r["cohort"] in ev.STRONG and r["directional_stratum"] == "aligned",
        "opposed": lambda r: r["cohort"] in ev.STRONG and r["directional_stratum"] == "opposed",
        "cross": lambda r: r["cohort"] in ev.STRONG and r["directional_stratum"] == "cross",
    }
    bootstrap = {name: ev.bootstrap([r for r in grouped["full_lqr_048"] if predicate(r)], [r for r in grouped[SELF_ID] if predicate(r)]) for name, predicate in subsets.items()}
    summaries = {name: {"overall": ev.metrics(rows), "strong": ev.metrics([r for r in rows if r["cohort"] in ev.STRONG]), "normal": ev.metrics([r for r in rows if r["cohort"] in ev.NORMAL]), "directional": {s: ev.metrics([r for r in rows if r["cohort"] in ev.STRONG and r["directional_stratum"] == s]) for s in ("aligned", "opposed", "cross")}} for name, rows in grouped.items()}
    comparisons = {}
    self_metric = summaries[SELF_ID]["overall"]
    for name in ev.TRADITIONAL:
        ref = summaries[name]["overall"]
        comparisons[name] = {"success_candidate": self_metric["success_rate"], "success_reference": ref["success_rate"], "position_improvement": ev.improve(ref["position_mean_m"], self_metric["position_mean_m"]), "acquisition_degradation": ev.degrade(self_metric["acquisition_median_s"], ref["acquisition_median_s"]), "orientation_improvement": ev.improve(ref["orientation_mean_deg"], self_metric["orientation_mean_deg"]), "effort_change": ev.degrade(self_metric["effort_mean"], ref["effort_mean"])}
    write("paired_bootstrap.json", bootstrap)
    write("cohort_analysis.json", summaries)
    write("directional_analysis.json", {"self": candidate["directional"], "formal_bootstrap": {name: bootstrap[name] for name in ("aligned", "opposed", "cross")}, "all_frozen_directional_gates_pass": all(value for data in candidate["directional"].values() for value in data["gates"].values())})
    write("self_vs_primary.json", {"primary": "full_lqr_048", "audit": candidate, "final_status": final_status})
    write("self_vs_all_traditionals.json", comparisons)
    write("holdout_safety_audit.json", {"all_methods": {name: {"sample_count": len(rows), "safe_count": sum(r["safe"] for r in rows), "safety_rate": ev.metrics(rows)["safety_rate"], "failure_ids": [r["sample_id"] for r in rows if not r["safe"]]} for name, rows in grouped.items()}, "compromised": False})
    write("paper_final_status.json", {"development_status": "PAPER_DEVELOPMENT_INELIGIBLE", "holdout_status": "NOT_RUN_DEVELOPMENT_INELIGIBLE", "self_vs_paper_holdout_claim_allowed": False})
    write("gate.json", {"result": final_status, "strict_all_frozen_gates": strict, "overall_gate": overall, "audit": candidate, "holdout_executed": True, "sample_count": 96, "participants": len(grouped), "authoritative_runs": sum(len(v) for v in grouped.values()), "retries": [], "compromised": False})
    print(json.dumps({"result": final_status, "strict": strict, "overall": overall, "gates": f"{candidate['hard_gate_pass_count']}/{candidate['hard_gate_count']}", "catastrophic": len(candidate["catastrophic_pair_ids"])}, indent=2)); return 0


if __name__ == "__main__": raise SystemExit(main())
