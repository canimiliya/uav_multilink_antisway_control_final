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
BOUNDS = {
    "native_pid": {
        "kp.0": (.20, 1.80), "kp.1": (.20, 1.80), "kp.2": (.15, 1.20),
        "kd.0": (.60, 2.40), "kd.1": (.60, 2.40), "kd.2": (.40, 1.80),
        "ki.0": (0.0, .30), "ki.1": (0.0, .30), "ki.2": (0.0, .22),
        "integral_limit": (.50, 3.0), "acceleration_limit": (3.0, 8.0),
        "acceleration_slew_per_s": (8.0, 60.0), "tip_correction_kp": (.05, .80),
        "tip_correction_kd": (.02, .40), "tip_correction_limit_m": (.15, .80),
        "swing_angle_gain": (0.0, .30), "swing_rate_gain": (.02, .30),
        "reference_acceleration_gain": (.65, 1.25),
    },
    "native_full_lqr": {
        "q_position": (1.0, 120.0), "q_velocity": (.5, 50.0), "q_attitude": (10.0, 250.0),
        "q_rate": (.5, 30.0), "q_integral": (.5, 150.0), "r_thrust": (.05, 8.0),
        "r_torque": (.05, 8.0), "integral_limit": (.3, 3.0), "reference_acceleration_gain": (.6, 1.3),
        "swing_angle_gain": (0.0, .30), "swing_rate_gain": (.01, .30), "constraint_margin": (.70, .95),
    },
}
BOUNDS["native_task_lqr"] = BOUNDS["native_full_lqr"]


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


def set_parameter(parameters: dict, path: str, value: float) -> None:
    parts = path.split(".")
    if len(parts) == 1:
        parameters[path] = value
    else:
        values = list(parameters[parts[0]]); values[int(parts[1])] = value; parameters[parts[0]] = values


def stage_c_registry(family: str, folder: Path) -> list[dict]:
    summaries = list(csv.DictReader((folder / "development_summary.csv").open(encoding="utf-8", newline="")))
    top_ids = [row["method_id"] for row in summaries[:2]]
    initial = json.loads((BASE / "protocol" / f"{family}_candidate_registry.json").read_text(encoding="utf-8"))
    anchors = [next(spec for spec in initial if spec["method_id"] == method_id) for method_id in top_ids]
    bounds = BOUNDS[family]; paths = list(bounds); registry = []
    for anchor_index, anchor in enumerate(anchors):
        for index in range(24):
            spec = json.loads(json.dumps(anchor)); path = paths[index % len(paths)]; low, high = bounds[path]
            current = anchor["parameters"][path] if "." not in path else anchor["parameters"][path.split(".")[0]][int(path.split(".")[1])]
            target = low if (index // len(paths)) % 2 == 0 else high
            fraction = (.35, .60, .80)[index % 3]
            set_parameter(spec["parameters"], path, current + fraction * (target - current))
            spec["outer_rate_hz"] = (20, 50, 100, 200, 500, 1000)[index % 6]
            spec["inner_rate_hz"] = max(500, spec["outer_rate_hz"])
            spec["method_id"] = f"r1r2_{'pid' if family == 'native_pid' else 'full_lqr' if family == 'native_full_lqr' else 'task_lqr'}_c_{anchor_index}_{index:02d}"
            spec["stage_c_parent"] = anchor["method_id"]; spec["stage_c_coordinate"] = path
            registry.append(spec)
    if len(registry) != 48 or len({spec["method_id"] for spec in registry}) != 48:
        raise AssertionError("Stage-C registry must contain 48 unique configurations")
    return registry


def main() -> None:
    parser = argparse.ArgumentParser(); parser.add_argument("--family", choices=FOLDERS, required=True)
    parser.add_argument("--stage", choices=["a", "b", "c-a", "c-b"], required=True); parser.add_argument("--workers", type=int, default=8)
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
    elif args.stage == "b":
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
    elif args.stage == "c-a":
        registry = stage_c_registry(args.family, folder)
        (folder / "stage_c_candidate_registry.json").write_text(json.dumps(registry, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        ids = json.loads((BASE / "protocol/stage_a_manifest.json").read_text(encoding="utf-8"))["case_ids"]
        all_rows = []; summaries = []
        for index, spec in enumerate(registry, 1):
            print(f"STAGE_C_A {args.family} {index}/48 {spec['method_id']} parent={spec['stage_c_parent']}", flush=True)
            rows = execute(spec, [by_id[value] for value in ids], args.workers); all_rows.extend(rows)
            summaries.append(add_stage_fields(aggregate(rows), rows))
        summaries.sort(key=stage_a_key); write_csv(folder / "stage_c_a_results.csv", all_rows); write_csv(folder / "stage_c_a_summary.csv", summaries)
        selected = [row["method_id"] for row in summaries[:6]]
        (folder / "stage_c_a_selection.json").write_text(json.dumps({"family": args.family,
            "selected_for_stage_c_b": selected, "case_count": 48,
            "selection_rule": "frozen R1R2 Stage-A lexicographic rule"}, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"family": args.family, "selected_for_stage_c_b": selected}), flush=True)
    else:
        registry = json.loads((folder / "stage_c_candidate_registry.json").read_text(encoding="utf-8"))
        selected_ids = json.loads((folder / "stage_c_a_selection.json").read_text(encoding="utf-8"))["selected_for_stage_c_b"]
        specs = [spec for spec in registry if spec["method_id"] in selected_ids]; identities = [case["identity"] for case in manifest["cases"]]
        all_rows = []; summaries = []
        for index, spec in enumerate(specs, 1):
            print(f"STAGE_C_B {args.family} {index}/{len(specs)} {spec['method_id']} cases=200", flush=True)
            rows = execute(spec, identities, args.workers); all_rows.extend(rows)
            summary = aggregate(rows); summary["competence_pass"] = competence(summary); summaries.append(summary)
        summaries.sort(key=family_key); write_csv(folder / "stage_c_development_results.csv", all_rows); write_csv(folder / "stage_c_development_summary.csv", summaries)
        initial = list(csv.DictReader((folder / "development_summary.csv").open(encoding="utf-8", newline="")))
        numeric = {key for key in summaries[0] if key not in {"method_id", "family", "competence_pass"}}
        for row in initial:
            row["competence_pass"] = row["competence_pass"].lower() == "true"
            for key in numeric:
                if key in row and row[key] not in {"", "None"}:
                    row[key] = int(row[key]) if key in {"case_count", "catastrophic_count"} else float(row[key])
                elif key in row:
                    row[key] = None
        combined = initial + summaries; combined.sort(key=family_key); selected = combined[0]
        all_specs = json.loads(registry_path.read_text(encoding="utf-8")) + registry
        selected_spec = next(spec for spec in all_specs if spec["method_id"] == selected["method_id"])
        previous_freeze = folder / "freeze.json"
        if previous_freeze.exists() and not (folder / "stage_b_initial_freeze.json").exists():
            (folder / "stage_b_initial_freeze.json").write_bytes(previous_freeze.read_bytes())
        freeze = {"family": args.family, "development_iteration": "P2-R1R2", "initial_unique_configs": 48,
                  "stage_c_unique_configs": 48, "total_unique_configs": 96, "stage_b_complete_candidates": 6,
                  "stage_c_b_complete_candidates": 6, "complete_development_cases": 200,
                  "selected": selected["method_id"], "selected_spec": selected_spec, "metrics": selected,
                  "competence_pass": bool(selected["competence_pass"])}
        previous_freeze.write_text(json.dumps(freeze, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"family": args.family, "selected": selected["method_id"], "competence": bool(selected["competence_pass"]),
                          "safety": selected["safety_rate"], "success": selected["success_rate"]}), flush=True)


if __name__ == "__main__":
    main()
