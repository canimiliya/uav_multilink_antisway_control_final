"""Complete the preregistered CART level and summarize V5 ablation evidence."""

from __future__ import annotations

import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

try:
    from scripts import evaluate_v5_self as ev
    from scripts import run_v5_self_development as runner
except ImportError:
    import evaluate_v5_self as ev
    import run_v5_self_development as runner


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v5/r0"
SELF = ROOT / "reproducibility/v5/self"
STRONG = ev.STRONG
NORMAL = ev.NORMAL


def read(path: Path) -> dict: return json.loads(path.read_text(encoding="utf-8"))
def write(path: Path, value: object) -> None: path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]), lineterminator="\n"); writer.writeheader(); writer.writerows(rows)


def summarize(rows: list[dict], full_rows: list[dict]) -> dict:
    strong = [r for r in rows if r["cohort"] in STRONG]; normal = [r for r in rows if r["cohort"] in NORMAL]
    full_strong = {r["sample_id"]: r for r in full_rows if r["cohort"] in STRONG}
    catastrophic = [r["sample_id"] for r in strong if r["position_rmse_3d_m"] > 2.0 * full_strong[r["sample_id"]]["position_rmse_3d_m"]]
    return {"overall": ev.metrics(rows), "strong": ev.metrics(strong), "normal": ev.metrics(normal), "catastrophic_pair_ids": catastrophic, "directional": {name: ev.metrics([r for r in strong if r["directional_stratum"] == name]) for name in ("aligned", "opposed", "cross")}}


def main() -> int:
    manifest = read(R0 / "development_manifest.json")
    cart = next(row for row in read(ROOT / "reproducibility/v4/r1/stage_b_summary.json")["summaries"] if row["candidate_id"] == "cart_b_014")["parameters"]
    cart_summary, cart_rows = runner.run_bank("cart_ofmpc", cart, manifest["samples"], "ablation_cart", 24)
    write_csv(SELF / "ablation_cart_b014_results.csv", cart_rows)
    write(SELF / "ablation_cart_b014_summary.json", cart_summary)

    baseline: dict[str, list[dict]] = defaultdict(list)
    for row in ev.rows(SELF / "baseline_development_results.csv"): baseline[row["candidate_id"]].append(row)
    diagnostic: dict[str, list[dict]] = defaultdict(list)
    for row in ev.rows(SELF / "ablation_new_results.csv"): diagnostic[row["candidate_id"]].append(row)
    full_rows = ev.rows(SELF / "development_results.csv")
    levels = [
        ("legacy_self_a_034", baseline["self_a_034"]), ("cart_b_014", ev.rows(SELF / "ablation_cart_b014_results.csv")),
        ("ablation_shock_only", diagnostic["ablation_shock_only"]), ("ablation_plus_bumpless", diagnostic["ablation_plus_bumpless"]),
        ("ablation_plus_coordination_headroom", diagnostic["ablation_plus_coordination_headroom"]), ("satc_b_027", full_rows),
    ]
    payload = {
        "status": "POST_FREEZE_DIAGNOSTIC_COMPLETE", "selection_authority": False, "return_to_tuning": False,
        "protocol_source_label_issue": {"field": "cart_b_014.source", "declared": "frozen V5 baseline reuse", "actual": "no matching V5-bank baseline existed", "resolution": "ran the already-preregistered frozen cart_b_014 parameters on the exact V5 Development bank; no layer, parameter, or selection change"},
        "technical_retries": [{"phase": "ablation_shock_only", "reason": "MuJoCo model allocation returned Could not allocate memory after 92 cached samples", "same_commit_config_samples": True, "completed_on_retry": True}],
        "levels": [{"id": name, **summarize(rows, baseline["full_lqr_048"])} for name, rows in levels],
    }
    write(SELF / "ablation.json", payload)

    traces = {}
    for path in sorted((SELF / "traces").glob("*.csv")):
        with path.open(encoding="utf-8", newline="") as stream: rows = list(csv.DictReader(stream))
        numeric = {key: np.asarray([float(row[key]) for row in rows]) for key in ("shock_score", "innovation_norm", "innovation_rate", "residual_unrepresented_norm", "trust", "conflict_index", "cancellation_index", "offset_engagement", "coordination_weight", "slew_headroom", "position_error_m", "orientation_error_deg", "joint_angle_max_deg")}
        traces[path.stem] = {"updates": len(rows), **{f"{key}_mean": float(np.mean(value)) for key, value in numeric.items()}, **{f"{key}_max": float(np.max(value)) for key, value in numeric.items()}, "constraint_activity_fraction": float(np.mean([row["constraint_activity"].lower() == "true" for row in rows]))}
    write(SELF / "mechanism_report.json", {"method": "SATC-OFMPC", "candidate": "satc_b_027", "trace_source": "four preregistered representative Development trajectories", "traces": traces, "mechanisms_observable": ["shock detection", "bumpless engagement", "backbone-QP cancellation coordination", "slew reserve", "geometric conflict index"], "causal_claim_scope": "mechanism-consistent Development evidence, not proof of physical causality", "holdout_executed": False})
    print(json.dumps({"cart_b014_runs": len(cart_rows), "ablation_levels": len(levels), "trace_strata": sorted(traces)}, indent=2))
    return 0


if __name__ == "__main__": raise SystemExit(main())
