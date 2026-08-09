"""Run frozen V5 Development only; this module cannot load V5 Holdout."""

from __future__ import annotations

import argparse
import csv
import json
import os
from collections import Counter
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

try:
    from scripts import run_v4_r1_development as core
except ImportError:  # direct ``python scripts/run_v5_self_development.py`` execution
    import run_v4_r1_development as core
from uav_sway.v3.controllers import V3CascadedTaskPID, V3FullStateLQR, V3TaskWeightedLQR
from uav_sway.v3.dr_tsrmpc import V3DRTSRMPC
from uav_sway.v3.metrics import load_r0_linear_matrices
from uav_sway.v4.cart_ofmpc import CARTOFMPC
from uav_sway.v5.satc_ofmpc import SATCOFMPC


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v5/r0"
SELF = ROOT / "reproducibility/v5/self"
core.R0 = R0
core.R1 = SELF


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns: columns.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n")
        writer.writeheader(); writer.writerows(rows)


def signed_wind_value(spec: dict, time_s: float, stochastic: np.ndarray | None, index: int) -> float:
    sign = float(spec.get("direction_sign", 1))
    unsigned = core._v5_original_wind_value(spec, time_s, stochastic, index)
    return sign * unsigned


def signed_stochastic_series(spec: dict, duration_s: float) -> np.ndarray | None:
    return core._v5_original_stochastic_series(spec, duration_s)


if not hasattr(core, "_v5_original_wind_value"):
    core._v5_original_wind_value = core.wind_value
    core._v5_original_stochastic_series = core.stochastic_series
core.wind_value = signed_wind_value
core.stochastic_series = signed_stochastic_series


def model_data() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    a, b = load_r0_linear_matrices(ROOT)
    metric = read(ROOT / "reproducibility/v3/r1/task_metric_alignment_audit.json")
    c_task = np.vstack([metric[name] for name in ("C_pos", "C_vel", "C_dir", "C_omega_perp")])
    return a, b, c_task


def build_controller(kind: str, parameters: dict):
    if kind == "corrected_pid":
        return V3CascadedTaskPID(
            np.asarray(parameters["uav_kp"]), np.asarray(parameters["uav_kd"]), np.asarray(parameters["uav_ki"]),
            np.asarray(parameters["tip_kp"]), np.asarray(parameters["tip_kd"]), np.asarray(parameters["correction_limit_m"]),
            float(parameters["correction_slew_m_per_update"]), float(parameters["integral_limit"]), str(parameters["tip_velocity_mode"]),
        )
    if kind == "full_lqr": return V3FullStateLQR(np.asarray(parameters["K"]))
    if kind == "task_lqr": return V3TaskWeightedLQR(np.asarray(parameters["K"]))
    a, b, c_task = model_data()
    if kind == "legacy_self": return V3DRTSRMPC(a, b, c_task, np.asarray(parameters["K"]), parameters)
    task_gain = np.asarray(read(ROOT / "reproducibility/v3/r1/task_lqr_freeze.json")["parameters"]["K"])
    if kind == "cart_ofmpc": return CARTOFMPC(a, b, c_task, task_gain, parameters)
    if kind == "satc_ofmpc":
        full_gain = np.asarray(read(ROOT / "reproducibility/v3/r1/full_lqr_freeze.json")["parameters"]["K"])
        return SATCOFMPC(a, b, c_task, task_gain, full_gain, parameters)
    raise KeyError(kind)


core.controller = build_controller


def comparator_specs() -> list[tuple[str, dict]]:
    frozen = [
        ("corrected_pid", "reproducibility/v3/r1r1/pid_freeze.json"),
        ("full_lqr", "reproducibility/v3/r1/full_lqr_freeze.json"),
        ("task_lqr", "reproducibility/v3/r1/task_lqr_freeze.json"),
        ("legacy_self", "reproducibility/v3/r2/self_freeze.json"),
    ]
    result = [(kind, read(ROOT / path)["parameters"]) for kind, path in frozen]
    result.append(("cart_ofmpc", read(ROOT / "reproducibility/v4/r1/near_miss.json")["parameters"]))
    return result


def cache_path(phase: str, candidate_id: str, sample_id: str) -> Path:
    return SELF / "_cache" / phase / candidate_id / f"{sample_id}.json"


def execute(kind: str, parameters: dict, sample: dict, phase: str) -> dict:
    path = cache_path(phase, parameters["candidate_id"], sample["sample_id"])
    if path.exists(): return read(path)
    row = core.run_case(kind, parameters, sample)
    row["directional_stratum"] = sample["directional_stratum"]
    row["wind_direction_sign"] = sample["wind"].get("direction_sign", 0)
    write_json(path, row)
    return row


def run_bank(kind: str, parameters: dict, samples: list[dict], phase: str, workers: int) -> tuple[dict, list[dict]]:
    rows: dict[str, dict] = {}
    if workers <= 1:
        for index, sample in enumerate(samples, 1):
            rows[sample["sample_id"]] = execute(kind, parameters, sample, phase)
            if index % 12 == 0 or index == len(samples): print(f"{phase}:{parameters['candidate_id']} {index}/{len(samples)}", flush=True)
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            futures = {pool.submit(execute, kind, parameters, sample, phase): sample["sample_id"] for sample in samples}
            for index, future in enumerate(as_completed(futures), 1):
                rows[futures[future]] = future.result()
                if index % 24 == 0 or index == len(samples): print(f"{phase}:{parameters['candidate_id']} {index}/{len(samples)}", flush=True)
    ordered = [rows[sample["sample_id"]] for sample in samples]
    summary = core.aggregate(parameters["candidate_id"], parameters, ordered)
    summary["cohorts"] = core.cohort_summaries(parameters, ordered)
    return summary, ordered


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("baseline", "stage-a", "stage-b", "stage-c"), required=True)
    parser.add_argument("--workers", type=int, default=min(24, os.cpu_count() or 1))
    args = parser.parse_args()
    manifest = read(R0 / "development_manifest.json")
    if manifest["name"] != "V5_FROZEN_DEVELOPMENT_BANK" or len(manifest["samples"]) != 120: raise RuntimeError("Development manifest drift")
    samples = manifest["samples"]
    if args.mode == "baseline":
        all_rows: list[dict] = []; summaries: dict[str, dict] = {}
        for kind, parameters in comparator_specs():
            summary, rows = run_bank(kind, parameters, samples, "baseline", args.workers)
            summaries[parameters["candidate_id"]] = summary; all_rows.extend(rows)
        write_csv(SELF / "baseline_development_results.csv", all_rows)
        write_json(SELF / "baseline_summary.json", {"authoritative_runs": len(all_rows), "controllers": summaries, "cohort_counts": dict(Counter(row["cohort"] for row in samples)), "holdout_executed": False})
        return 0
    protocol = read(SELF / "development_protocol.json")
    key = args.mode.replace("-", "_")
    selected_samples = samples
    if key == "stage_a":
        ids = set(protocol["search"]["stage_a"]["sample_ids"]); selected_samples = [row for row in samples if row["sample_id"] in ids]
        candidates = protocol["search"]["stage_a"]["candidates"]
    elif key == "stage_b": candidates = protocol["search"]["stage_b"]["candidates"]
    else:
        history = read(SELF / "search_history.json")
        ids = history["stage_c_selected_candidate_ids"]
        by_id = {row["candidate_id"]: row for row in protocol["search"]["stage_b"]["candidates"]}
        candidates = [by_id[value] for value in ids]
    summaries = []; all_rows = []
    for parameters in candidates:
        summary, rows = run_bank("satc_ofmpc", parameters, selected_samples, key, args.workers)
        summaries.append(summary); all_rows.extend(rows)
    write_json(SELF / f"{key}_summary.json", {"stage": key, "candidate_count": len(candidates), "sample_count_each": len(selected_samples), "summaries": summaries, "holdout_executed": False})
    write_csv(SELF / f"{key}_results.csv", all_rows)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
