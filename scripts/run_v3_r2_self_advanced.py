"""Development-only search runner for V3-R2 full-3D DR-TSRMPC."""

from __future__ import annotations

import argparse
import itertools
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

from run_v3_r1_baselines import (  # noqa: E402
    case_subset, read_json, run_candidates, summary_without_rows, write_csv, write_json,
)


R1 = ROOT / "reproducibility/v3/r1"
R1R1 = ROOT / "reproducibility/v3/r1r1"
R2 = ROOT / "reproducibility/v3/r2"


def _gain(backbone: str) -> list[list[float]]:
    name = "full_lqr_freeze.json" if backbone == "full_lqr_048" else "task_lqr_freeze.json"
    return read_json(R1 / name)["parameters"]["K"]


def round_a_candidates() -> list[dict]:
    candidates = []
    values = itertools.product(
        ("full_lqr_048", "task_lqr_009"), (8, 12), (0.05, 0.15, 0.30),
        (0.08, 0.20), (40.0, 120.0),
    )
    for index, (backbone, horizon, beta, clip, position_weight) in enumerate(values):
        candidates.append({
            "candidate_id": f"self_a_{index:03d}",
            "architecture_version": "3D-DR-TSRMPC-A1",
            "backbone": backbone,
            "K": _gain(backbone),
            "horizon_updates": horizon,
            "residual_beta": beta,
            "residual_clip_norm": clip,
            "task_position_weight": position_weight,
            "task_velocity_weight": 2.0,
            "orientation_weight": 1.0,
            "angular_velocity_weight": 0.2,
            "residual_correction_weight": 0.2,
            "command_rate_weight": 1.0,
        })
    assert len(candidates) == 48
    return candidates


def search_key(row: dict) -> tuple:
    return (
        -row["safe_sample_count"], -row["task_success_count"], row["position_rmse_3d_m"],
        float("inf") if row["acquisition_median_s"] is None else row["acquisition_median_s"],
        row["ramp_steady_state_position_error_m"], row["ramp_peak_position_error_m"],
        row["solve_time_p95_ms"], row["candidate_id"],
    )


def core_samples(samples: list[dict]) -> list[dict]:
    result = case_subset(samples, "core")
    extra = next(sample for sample in samples if sample["sample_id"] == "calm_face_diagonal_00")
    result.append(extra)
    assert len(result) == 14
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--round-b", action="store_true")
    args = parser.parse_args()
    if args.smoke and args.round_b:
        raise ValueError("smoke and Round B are mutually exclusive")
    manifest = read_json(R1 / "development_evaluation_manifest.json")
    if manifest["split"] != "development":
        raise RuntimeError("R2 runner refuses non-development manifests")
    samples = manifest["samples"]
    candidates = round_a_candidates()
    write_json(R2 / "self_round_a_candidates.json", {
        "written_before_round_a_performance": True,
        "candidate_count": len(candidates),
        "development_case_count": 14,
        "candidate_ids": [row["candidate_id"] for row in candidates],
        "holdout_executed": False,
    })
    if args.round_b:
        protocol = read_json(R2 / "self_round_b_protocol.json")
        ids = protocol["SEARCH_SPACE"]["candidate_ids"]
        by_id = {row["candidate_id"]: row for row in candidates}
        selected_candidates = [by_id[candidate_id] for candidate_id in ids]
        selected_samples = samples
    else:
        selected_candidates = candidates[:1] if args.smoke else candidates
        selected_samples = core_samples(samples)[:1] if args.smoke else core_samples(samples)
    summaries = run_candidates("self_dr_tsrmpc", selected_candidates, selected_samples, args.workers, "v3-r2-a")
    summaries.sort(key=search_key)
    suffix = "round_b" if args.round_b else "smoke" if args.smoke else "round_a"
    write_csv(R2 / f"self_{suffix}.csv", [summary_without_rows(row) for row in summaries])
    write_csv(R2 / f"development_results_{suffix}.csv", [
        {**sample, "candidate_id": row["candidate_id"]} for row in summaries for sample in row["rows"]
    ])
    write_json(R2 / f"self_{suffix}_summary.json", {
        "scope": "Development only",
        "candidate_count": len(selected_candidates),
        "case_count": len(selected_samples),
        "best": summary_without_rows(summaries[0]),
        "holdout_executed": False,
    })
    print(json.dumps(summary_without_rows(summaries[0]), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
