"""Development-only runner for LV2026-SPACC-ADAPTED-3D.

The frozen V3 benchmark runner remains byte-unchanged.  Each worker injects
the Paper controller through a local factory and then calls the frozen case
execution path.  There is no code path that loads the Holdout manifest.
"""

from __future__ import annotations

import argparse
import itertools
import json
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

from uav_sway.v3.lv2026_spacc import V3LV2026SPACC

import run_v3_r1_baselines as base


ROOT = Path(__file__).resolve().parents[1]
R1 = ROOT / "reproducibility" / "v3" / "r1"
R3 = ROOT / "reproducibility" / "v3" / "r3"
PAPER_PROTOCOL_FREEZE_HEAD = "39a01592429897df943f306ec61c70deedabad3e"


def _gain(backbone: str) -> list[list[float]]:
    filename = "full_lqr_freeze.json" if backbone == "full_lqr_048" else "task_lqr_freeze.json"
    return base.read_json(R1 / filename)["parameters"]["K"]


def candidates() -> list[dict]:
    rows = []
    values = itertools.product(
        ("full_lqr_048", "task_lqr_009"),
        (0.25, 0.5),
        (0.5, 1.0),
        (0.0625, 0.125, 0.25),
        (0.1, 0.25),
    )
    for index, (backbone, position_gain, velocity_gain, swing_gain_scale, swing_scale) in enumerate(values):
        rows.append({
            "candidate_id": f"paper_a_{index:03d}",
            "method": "LV2026-SPACC-ADAPTED-3D",
            "backbone": backbone,
            "K": _gain(backbone),
            "outer_position_gain": position_gain,
            "outer_velocity_gain": velocity_gain,
            "desired_tip_speed_limit_m_s": 0.5,
            "swing_gain_scale_of_reported_3_2": swing_gain_scale,
            "swing_correction_scale": swing_scale,
            "swing_correction_clip_m_s2": 0.5,
        })
    assert len(rows) == 48
    return rows


def core_samples(samples: list[dict]) -> list[dict]:
    result = base.case_subset(samples, "core")
    result.append(next(row for row in samples if row["sample_id"] == "calm_face_diagonal_00"))
    assert len(result) == 14
    return result


def search_key(row: dict) -> tuple:
    return (
        -row["safe_sample_count"], -row["task_success_count"], row["position_rmse_3d_m"],
        float("inf") if row["acquisition_median_s"] is None else row["acquisition_median_s"],
        row["ramp_steady_state_position_error_m"], row["ramp_peak_position_error_m"],
        row["solve_time_p95_ms"], row["candidate_id"],
    )


def run_paper_case(parameters: dict, sample: dict, output_csv: str | None = None) -> dict:
    original_factory = base._controller

    def paper_factory(kind: str, values: dict, gains: dict | None = None):
        if kind == "paper_lv2026":
            return V3LV2026SPACC(values["K"], values)
        return original_factory(kind, values, gains)

    base._controller = paper_factory
    try:
        return base.run_case("paper_lv2026", parameters, sample, output_csv)
    finally:
        base._controller = original_factory


def _job(payload: tuple[dict, dict]) -> dict:
    return run_paper_case(*payload)


def run_candidates(selected: list[dict], samples: list[dict], workers: int, label: str) -> list[dict]:
    jobs = [(candidate, sample) for candidate in selected for sample in samples]
    results: dict[tuple[str, str], dict] = {}
    if workers <= 1:
        for index, job in enumerate(jobs, 1):
            row = _job(job)
            results[(job[0]["candidate_id"], job[1]["sample_id"])] = row
            if index % 10 == 0 or index == len(jobs):
                print(f"{label} {index}/{len(jobs)}", flush=True)
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            futures = [(job, pool.submit(_job, job)) for job in jobs]
            for index, (job, future) in enumerate(futures, 1):
                results[(job[0]["candidate_id"], job[1]["sample_id"])] = future.result()
                if index % 25 == 0 or index == len(jobs):
                    print(f"{label} {index}/{len(jobs)}", flush=True)
    summaries = []
    for candidate in selected:
        rows = [results[(candidate["candidate_id"], sample["sample_id"])] for sample in samples]
        summaries.append(base.aggregate(candidate["candidate_id"], rows, candidate))
    return summaries


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--round-b", action="store_true")
    args = parser.parse_args()
    if args.smoke and args.round_b:
        raise ValueError("smoke and Round B are mutually exclusive")
    manifest = base.read_json(R1 / "development_evaluation_manifest.json")
    if manifest["split"] != "development":
        raise RuntimeError("Paper runner refuses non-development manifests")
    all_candidates = candidates()
    samples = manifest["samples"]
    if args.round_b:
        protocol = base.read_json(R3 / "paper_round_b_protocol.json")
        by_id = {row["candidate_id"]: row for row in all_candidates}
        selected = [by_id[name] for name in protocol["candidate_ids"]]
        selected_samples = samples
        suffix = "round_b"
    else:
        selected = all_candidates[:1] if args.smoke else all_candidates
        selected_samples = core_samples(samples)[:1] if args.smoke else core_samples(samples)
        suffix = "smoke" if args.smoke else "round_a"
    summaries = run_candidates(selected, selected_samples, args.workers, f"v3-r3-{suffix}")
    summaries.sort(key=search_key)
    base.write_csv(R3 / f"paper_{suffix}.csv", [base.summary_without_rows(row) for row in summaries])
    base.write_csv(R3 / f"development_results_{suffix}.csv", [
        {**sample, "candidate_id": row["candidate_id"]}
        for row in summaries for sample in row["rows"]
    ])
    base.write_json(R3 / f"paper_{suffix}_summary.json", {
        "paper_protocol_freeze_head": PAPER_PROTOCOL_FREEZE_HEAD,
        "scope": "Development only",
        "candidate_count": len(selected),
        "case_count": len(selected_samples),
        "best": base.summary_without_rows(summaries[0]),
        "holdout_executed": False,
    })
    print(json.dumps(base.summary_without_rows(summaries[0]), indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
