"""Run only the frozen V6 Development bank."""

from __future__ import annotations

import argparse
import json
import os
from collections import Counter
from pathlib import Path

import numpy as np

try:
    from scripts import run_v5_self_development as runner
except ImportError:
    import run_v5_self_development as runner
from uav_sway.v6.paper_controllers import SEP2026PassivityNMPC, Yu2026FiniteTimeCFO


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v6/r0"
PAPERS = ROOT / "reproducibility/v6/papers"
OUT = ROOT / "reproducibility/v6/development"
runner.R0 = R0
runner.SELF = OUT
runner.core.R0 = R0
runner.core.R1 = OUT


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


base_build_controller = runner.build_controller


def build_controller(kind: str, parameters: dict):
    if kind == "yu2026":
        return Yu2026FiniteTimeCFO(parameters)
    if kind == "sep2026":
        a, b, c_task = runner.model_data()
        return SEP2026PassivityNMPC(a, b, c_task, parameters)
    return base_build_controller(kind, parameters)


runner.core.controller = build_controller


def comparator_specs() -> list[tuple[str, dict]]:
    frozen = [
        ("corrected_pid", "reproducibility/v3/r1r1/pid_freeze.json"),
        ("full_lqr", "reproducibility/v3/r1/full_lqr_freeze.json"),
        ("task_lqr", "reproducibility/v3/r1/task_lqr_freeze.json"),
    ]
    result = [(kind, read(ROOT / path)["parameters"]) for kind, path in frozen]
    result.append(("satc_ofmpc", read(ROOT / "reproducibility/v5/self/self_freeze.json")["parameters"]))
    return result


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("baseline", "stage-a", "stage-b"), required=True)
    parser.add_argument("--paper", choices=("YU2026", "SEP2026", "all"), default="all")
    parser.add_argument("--workers", type=int, default=min(16, os.cpu_count() or 1))
    args = parser.parse_args()
    manifest = read(R0 / "development_manifest.json")
    if manifest["name"] != "V6_FROZEN_DEVELOPMENT_BANK" or len(manifest["samples"]) != 120:
        raise RuntimeError("V6 Development manifest drift")
    samples = manifest["samples"]
    if args.mode == "baseline":
        rows = []
        summaries = {}
        for kind, parameters in comparator_specs():
            summary, values = runner.run_bank(kind, parameters, samples, "v6_baseline", args.workers)
            summaries[parameters["candidate_id"]] = summary
            rows.extend(values)
        runner.write_csv(OUT / "baseline_results.csv", rows)
        runner.write_json(OUT / "baseline_summary.json", {
            "sample_count": 120, "participants": summaries, "authoritative_runs": len(rows),
            "cohort_counts": dict(Counter(row["cohort"] for row in samples)),
            "satc_tuned_on_v6": False, "holdout_accessed": False,
        })
        return 0
    protocol = read(PAPERS / "development_protocol.json")
    paper_ids = ("YU2026", "SEP2026") if args.paper == "all" else (args.paper,)
    stage_key = args.mode.replace("-", "_")
    selected_samples = samples
    if stage_key == "stage_a":
        ids = set(protocol["stage_a_sample_ids"])
        selected_samples = [row for row in samples if row["sample_id"] in ids]
    for paper_id in paper_ids:
        kind = "yu2026" if paper_id == "YU2026" else "sep2026"
        candidates = protocol["papers"][paper_id][stage_key]
        rows = []
        summaries = []
        for parameters in candidates:
            summary, values = runner.run_bank(kind, parameters, selected_samples, f"v6_{paper_id.lower()}_{stage_key}", args.workers)
            summaries.append(summary)
            rows.extend(values)
        target = PAPERS / paper_id.lower()
        runner.write_csv(target / f"{stage_key}_results.csv", rows)
        runner.write_json(target / f"{stage_key}_summary.json", {
            "paper": paper_id, "stage": stage_key, "candidate_count": len(candidates),
            "sample_count_each": len(selected_samples), "summaries": summaries,
            "holdout_accessed": False,
        })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
