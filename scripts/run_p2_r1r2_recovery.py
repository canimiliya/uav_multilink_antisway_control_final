"""Execute frozen P2-R1R2 Stage A/B candidate evaluations."""

from __future__ import annotations

import argparse
import csv
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from uav_sway.native_stack.r1r2_evaluation import aggregate, competence, evaluate_case


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "reproducibility/native_stack/r1r2"
FOLDERS = {"native_pid": "traditional/pid", "native_full_lqr": "traditional/full_lqr", "native_task_lqr": "traditional/task_lqr"}


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)


def execute(spec: dict, identities: list[dict], workers: int) -> list[dict]:
    payloads = [(spec, identity) for identity in identities]
    if workers == 1:
        return [evaluate_case(payload) for payload in payloads]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        return list(pool.map(evaluate_case, payloads, chunksize=1))


def stage_a_key(summary: dict) -> tuple:
    return (summary["catastrophic_count"], -summary["success_rate"], summary["large_endpoint_failure_fraction"],
            -summary["safety_rate"], summary["setpoint_position_rmse_m"], summary["trajectory_position_rmse_m"], summary["method_id"])


def family_key(summary: dict) -> tuple:
    return (-int(summary["competence_pass"]), -summary["safety_rate"], -summary["success_rate"],
            summary["setpoint_position_rmse_m"], summary["trajectory_position_rmse_m"], summary["strong_p90_m"],
            float("inf") if summary["acquisition_time_s"] is None else summary["acquisition_time_s"],
            summary["physical_effort"], summary["runtime_p95_ms"], summary["method_id"])


def add_stage_fields(summary: dict, rows: list[dict]) -> dict:
    value = dict(summary)
    value["large_endpoint_failure_fraction"] = sum(r["endpoint_position_error_m"] > 1.5 for r in rows) / len(rows)
    return value


def main() -> None:
    parser = argparse.ArgumentParser(); parser.add_argument("--family", choices=FOLDERS, required=True)
    parser.add_argument("--stage", choices=["a", "b"], required=True); parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args(); folder = BASE / FOLDERS[args.family]; folder.mkdir(parents=True, exist_ok=True)
    registry_path = BASE / "protocol" / f"{args.family}_candidate_registry.json"
    registry = json.loads(registry_path.read_text(encoding="utf-8"))
    manifest = json.loads((ROOT / "reproducibility/native_stack/r0s/resolved_development_manifest.json").read_text(encoding="utf-8"))
    by_id = {case["sample_id"]: case["identity"] for case in manifest["cases"]}
    if args.stage == "a":
        ids = json.loads((BASE / "protocol/stage_a_manifest.json").read_text(encoding="utf-8"))["case_ids"]
        all_rows: list[dict] = []; summaries = []
        for index, spec in enumerate(registry, 1):
            print(f"STAGE_A {args.family} {index}/48 {spec['method_id']} rate={spec['outer_rate_hz']}/{spec['inner_rate_hz']}", flush=True)
            rows = execute(spec, [by_id[value] for value in ids], args.workers); all_rows.extend(rows)
            summaries.append(add_stage_fields(aggregate(rows), rows))
        summaries.sort(key=stage_a_key); write_csv(folder / "stage_a_results.csv", all_rows); write_csv(folder / "stage_a_summary.csv", summaries)
        selected = [row["method_id"] for row in summaries[:6]]
        (folder / "stage_a_selection.json").write_text(json.dumps({"family": args.family, "selected_for_stage_b": selected,
            "case_count": 48, "selection_rule": "frozen R1R2 Stage-A lexicographic rule"}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"family": args.family, "selected_for_stage_b": selected}), flush=True)
    else:
        selected_ids = json.loads((folder / "stage_a_selection.json").read_text(encoding="utf-8"))["selected_for_stage_b"]
        specs = [spec for spec in registry if spec["method_id"] in selected_ids]; identities = [case["identity"] for case in manifest["cases"]]
        all_rows = []; summaries = []
        for index, spec in enumerate(specs, 1):
            print(f"STAGE_B {args.family} {index}/{len(specs)} {spec['method_id']} cases=200", flush=True)
            rows = execute(spec, identities, args.workers); all_rows.extend(rows)
            summary = aggregate(rows); summary["competence_pass"] = competence(summary); summaries.append(summary)
        summaries.sort(key=family_key); write_csv(folder / "development_results.csv", all_rows); write_csv(folder / "development_summary.csv", summaries)
        selected = summaries[0]; selected_spec = next(spec for spec in specs if spec["method_id"] == selected["method_id"])
        freeze = {"family": args.family, "development_iteration": "P2-R1R2", "initial_unique_configs": 48,
                  "maximum_unique_configs": 96, "stage_b_complete_candidates": len(specs), "complete_development_cases": 200,
                  "selected": selected["method_id"], "selected_spec": selected_spec, "metrics": selected,
                  "competence_pass": selected["competence_pass"]}
        (folder / "freeze.json").write_text(json.dumps(freeze, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"family": args.family, "selected": selected["method_id"], "competence": selected["competence_pass"],
                          "safety": selected["safety_rate"], "success": selected["success_rate"]}), flush=True)


if __name__ == "__main__":
    main()

