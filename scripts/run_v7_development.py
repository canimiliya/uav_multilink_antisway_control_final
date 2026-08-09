"""Run only the frozen V7 Development bank."""

from __future__ import annotations

import argparse
import json
import os
from collections import Counter
from pathlib import Path

try:
    from scripts import run_v5_self_development as runner
except ImportError:
    import run_v5_self_development as runner
from uav_sway.v7.kang2026_fas_dob import Kang2026FASDOB


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v7/r0"
PAPER = ROOT / "reproducibility/v7/paper"
OUT = ROOT / "reproducibility/v7/development"
runner.R0 = R0
runner.SELF = OUT
runner.core.R0 = R0
runner.core.R1 = OUT
base_build_controller = runner.build_controller


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def build_controller(kind: str, parameters: dict):
    if kind == "kang2026":
        return Kang2026FASDOB(parameters)
    return base_build_controller(kind, parameters)


runner.core.controller = build_controller


def comparator_specs() -> list[tuple[str, dict]]:
    frozen = [
        ("corrected_pid", "reproducibility/v3/r1r1/pid_freeze.json"),
        ("full_lqr", "reproducibility/v3/r1/full_lqr_freeze.json"),
        ("task_lqr", "reproducibility/v3/r1/task_lqr_freeze.json"),
        ("satc_ofmpc", "reproducibility/v5/self/self_freeze.json"),
    ]
    return [(kind, read(ROOT / path)["parameters"]) for kind, path in frozen]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("baseline", "stage-a", "stage-b", "stage-c"), required=True)
    parser.add_argument("--workers", type=int, default=min(24, os.cpu_count() or 1))
    args = parser.parse_args()
    manifest = read(R0 / "development_manifest.json")
    if manifest["name"] != "V7_FROZEN_DEVELOPMENT_BANK" or len(manifest["samples"]) != 144:
        raise RuntimeError("V7 Development manifest drift")
    samples = manifest["samples"]
    if args.mode == "baseline":
        rows = []
        summaries = {}
        for kind, parameters in comparator_specs():
            summary, values = runner.run_bank(kind, parameters, samples, "v7_baseline", args.workers)
            summaries[parameters["candidate_id"]] = summary
            rows.extend(values)
        runner.write_csv(OUT / "baseline_results.csv", rows)
        runner.write_json(OUT / "baseline_summary.json", {
            "sample_count": 144, "participants": summaries, "authoritative_runs": len(rows),
            "cohort_counts": dict(Counter(row["cohort"] for row in samples)),
            "traditional_retuned": False, "satc_retuned": False, "holdout_accessed": False,
        })
        return 0
    protocol = read(PAPER / "development_protocol.json")
    stage = args.mode.replace("-", "_")
    selected = samples
    if stage == "stage_a":
        ids = set(protocol["stage_a_sample_ids"])
        selected = [row for row in samples if row["sample_id"] in ids]
    candidates = protocol["stages"][stage]["candidates"]
    rows = []
    summaries = []
    for parameters in candidates:
        summary, values = runner.run_bank("kang2026", parameters, selected, f"v7_{stage}", args.workers)
        summaries.append(summary)
        rows.extend(values)
    runner.write_csv(PAPER / f"{stage}_results.csv", rows)
    runner.write_json(PAPER / f"{stage}_summary.json", {
        "stage": stage, "candidate_count": len(candidates), "sample_count_each": len(selected),
        "summaries": summaries, "holdout_accessed": False,
    })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

