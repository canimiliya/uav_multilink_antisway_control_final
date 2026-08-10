"""Run frozen staged Traditional qualification for P2-R1R3."""

from __future__ import annotations

import argparse
import csv
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from uav_sway.native_stack.r1r3_evaluation import aggregate_v2, competence_v2, evaluate_case


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "reproducibility/native_stack/r1r3"
PROTOCOL = BASE / "protocol"
FOLDERS = {"native_pid": "traditional/pid", "native_full_lqr": "traditional/full_lqr"}


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def execute(spec: dict, identities: list[dict], workers: int) -> list[dict]:
    payloads = [(spec, identity) for identity in identities]
    if workers == 1:
        return [evaluate_case(payload) for payload in payloads]
    with ProcessPoolExecutor(max_workers=workers) as pool:
        return list(pool.map(evaluate_case, payloads, chunksize=1))


def stage_a_key(summary: dict) -> tuple:
    nominal_balance = min(
        summary["nominal_calm_success_rate"], summary["nominal_moderate_success_rate"],
        summary["nominal_stochastic_success_rate"], summary["nominal_setpoint_success_rate"],
        summary["nominal_trajectory_success_rate"],
    )
    nominal_rmse = summary["nominal_setpoint_rmse_m"] + summary["nominal_trajectory_rmse_m"]
    return (
        summary["all_catastrophic_count"], -summary["all_safety_rate"],
        -summary["nominal_success_rate"], -nominal_balance,
        summary["nominal_endpoint_position_error_m"], nominal_rmse, summary["method_id"],
    )


def qualification_key(summary: dict) -> tuple:
    nominal_balance = min(
        summary["nominal_calm_success_rate"], summary["nominal_moderate_success_rate"],
        summary["nominal_stochastic_success_rate"], summary["nominal_setpoint_success_rate"],
        summary["nominal_trajectory_success_rate"],
    )
    nominal_rmse = summary["nominal_setpoint_rmse_m"] + summary["nominal_trajectory_rmse_m"]
    return (
        -int(summary["competence_v2"]), -summary["all_safety_rate"],
        -summary["nominal_success_rate"], -nominal_balance, nominal_rmse,
        -summary["challenge_success_rate"], summary["strong_p90_m"],
        summary["physical_effort"], summary["runtime_p95_ms"], summary["method_id"],
    )


def load_context(family: str, round_name: str) -> tuple[list[dict], list[dict], dict[str, dict]]:
    registry_root = PROTOCOL if round_name == "initial" else BASE / "recovery_protocol"
    suffix = "candidate_registry.json" if round_name == "initial" else "local_candidate_registry.json"
    registry = json.loads((registry_root / f"{family}_{suffix}").read_text(encoding="utf-8"))
    manifest = json.loads((ROOT / "reproducibility/native_stack/r0s/resolved_development_manifest.json").read_text(encoding="utf-8"))
    by_id = {case["sample_id"]: case["identity"] for case in manifest["cases"]}
    return registry, manifest["cases"], by_id


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--family", choices=FOLDERS, required=True)
    parser.add_argument("--stage", choices=("a", "b", "confirm"), required=True)
    parser.add_argument("--round", choices=("initial", "local"), default="initial")
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()
    folder = BASE / FOLDERS[args.family]
    folder.mkdir(parents=True, exist_ok=True)
    prefix = "" if args.round == "initial" else "local_"
    registry, cases, by_id = load_context(args.family, args.round)
    if args.stage == "a":
        ids = json.loads((PROTOCOL / "stage_a_manifest.json").read_text(encoding="utf-8"))["case_ids"]
        identities = [by_id[case_id] for case_id in ids]
        all_rows: list[dict] = []
        summaries: list[dict] = []
        for index, spec in enumerate(registry, 1):
            print(f"STAGE_A {args.family} {index}/{len(registry)} {spec['method_id']}", flush=True)
            rows = execute(spec, identities, args.workers)
            all_rows.extend(rows)
            summaries.append(aggregate_v2(rows))
        summaries.sort(key=stage_a_key)
        selected = [summary["method_id"] for summary in summaries[:6]]
        write_csv(folder / f"{prefix}stage_a_results.csv", all_rows)
        write_csv(folder / f"{prefix}stage_a_summary.csv", summaries)
        (folder / f"{prefix}stage_a_selection.json").write_text(json.dumps({
            "family": args.family, "evaluated_configs": len(registry), "case_count": len(ids),
            "selected_for_full_200": selected, "selection_rule": "frozen P2-R1R3 Stage-A order",
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"selected_for_full_200": selected}), flush=True)
        return

    identities = [case["identity"] for case in cases]
    if args.stage == "b":
        selected_ids = json.loads((folder / f"{prefix}stage_a_selection.json").read_text(encoding="utf-8"))["selected_for_full_200"]
        specs = [spec for spec in registry if spec["method_id"] in selected_ids]
        all_rows = []
        summaries = []
        for index, spec in enumerate(specs, 1):
            print(f"FULL_200 {args.family} {index}/{len(specs)} {spec['method_id']}", flush=True)
            rows = execute(spec, identities, args.workers)
            all_rows.extend(rows)
            summary = aggregate_v2(rows)
            summary["competence_v2"] = competence_v2(summary)
            summaries.append(summary)
        summaries.sort(key=qualification_key)
        write_csv(folder / f"{prefix}development_results.csv", all_rows)
        write_csv(folder / f"{prefix}development_summary.csv", summaries)
        selected = summaries[0]
        selected_spec = next(spec for spec in specs if spec["method_id"] == selected["method_id"])
        (folder / f"{prefix}provisional_selection.json").write_text(json.dumps({
            "family": args.family, "stage_a_configs": len(registry), "full_200_configs": len(specs),
            "selected": selected["method_id"], "selected_spec": selected_spec,
            "metrics": selected, "competence_v2": selected["competence_v2"],
            "confirmation_required": bool(selected["competence_v2"]),
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"selected": selected["method_id"], "competence_v2": selected["competence_v2"],
                          "safety": selected["all_safety_rate"], "nominal": selected["nominal_success_rate"]}), flush=True)
        return

    provisional = json.loads((folder / f"{prefix}provisional_selection.json").read_text(encoding="utf-8"))
    if not provisional["competence_v2"]:
        raise RuntimeError("confirmation is forbidden because the provisional candidate did not pass")
    spec = provisional["selected_spec"]
    print(f"CONFIRM_200 {args.family} {spec['method_id']}", flush=True)
    rows = execute(spec, identities, args.workers)
    summary = aggregate_v2(rows)
    summary["competence_v2"] = competence_v2(summary)
    write_csv(folder / f"{prefix}confirmation_results.csv", rows)
    prior = provisional["metrics"]
    performance_keys = [key for key in summary if key not in {
        "runtime_mean_ms", "runtime_p95_ms", "runtime_p99_ms", "runtime_max_ms", "all_deadline_miss_rate"
    }]
    reproducible = all(
        (summary[key] == prior[key]) if isinstance(summary[key], (str, bool, int)) or summary[key] is None
        else abs(float(summary[key]) - float(prior[key])) <= 1.0e-12
        for key in performance_keys
    )
    freeze = {
        "family": args.family, "selected": spec["method_id"], "selected_spec": spec,
        "qualification_metrics": prior, "confirmation_metrics": summary,
        "performance_reproducible": reproducible,
        "competence_v2": bool(summary["competence_v2"] and provisional["competence_v2"] and reproducible),
        "authoritative_development_runs_per_confirmation": 200,
    }
    (folder / f"{prefix}freeze.json").write_text(json.dumps(freeze, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"selected": spec["method_id"], "confirmed": freeze["competence_v2"],
                      "reproducible": reproducible}), flush=True)


if __name__ == "__main__":
    main()
