"""Run only the frozen V8 Development bank; no Holdout path is imported."""

from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path

try:
    from scripts import run_v5_self_development as runner
except ImportError:
    import run_v5_self_development as runner
from uav_sway.v8.xu2025_cbs_ftdo import Xu2025CBSFTDO


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v8/r0"
PAPER = ROOT / "reproducibility/v8/paper"
OUT = ROOT / "reproducibility/v8/development"
V7_DEV = ROOT / "reproducibility/v7/development"
runner.R0 = R0
runner.SELF = OUT
runner.core.R0 = R0
runner.core.R1 = OUT
base_build_controller = runner.build_controller


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def build_controller(kind: str, parameters: dict):
    if kind == "xu2025":
        return Xu2025CBSFTDO(parameters)
    return base_build_controller(kind, parameters)


runner.core.controller = build_controller


def registry() -> dict[str, dict]:
    rows = read(PAPER / "candidate_registry.json")["candidates"]
    return {row["candidate_id"]: row for row in rows}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("baseline", "stage-a", "stage-b", "stage-c"), required=True)
    parser.add_argument("--workers", type=int, default=min(12, os.cpu_count() or 1))
    args = parser.parse_args()
    manifest = read(R0 / "development_manifest.json")
    if manifest["name"] != "V7_FROZEN_DEVELOPMENT_BANK" or len(manifest["samples"]) != 144:
        raise RuntimeError("V8 carried-forward Development manifest drift")
    if args.mode == "baseline":
        OUT.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(V7_DEV / "baseline_results.csv", OUT / "baseline_results.csv")
        shutil.copyfile(V7_DEV / "baseline_summary.json", OUT / "baseline_summary.json")
        runner.write_json(
            OUT / "baseline_carryforward_audit.json",
            {
                "source": "reproducibility/v7/development/baseline_results.csv",
                "source_bytes": (V7_DEV / "baseline_results.csv").stat().st_size,
                "byte_identical": (V7_DEV / "baseline_results.csv").read_bytes() == (OUT / "baseline_results.csv").read_bytes(),
                "traditional_rerun": False,
                "satc_rerun": False,
                "traditional_retuned": False,
                "satc_retuned": False,
                "holdout_accessed": False,
            },
        )
        return 0

    protocol = read(PAPER / "development_protocol.json")
    by_id = registry()
    samples = manifest["samples"]
    if args.mode == "stage-a":
        candidate_ids = protocol["stages"]["stage_a"]["candidate_ids"]
        sample_ids = set(protocol["stages"]["stage_a"]["sample_ids"])
        selected_samples = [row for row in samples if row["sample_id"] in sample_ids]
        phase = "v8_stage_a"
    elif args.mode == "stage-b":
        candidate_ids = protocol["stages"]["stage_b"]["candidate_ids"]
        selected_samples = samples
        phase = "v8_stage_b"
    else:
        provisional = read(PAPER / "development_final.json")
        if provisional["result"] != "V8_PAPER_QUALIFIED_PENDING_CONFIRMATION":
            raise RuntimeError("Stage C requires a preregistered provisional winner")
        candidate_ids = [provisional["selected_candidate"]]
        selected_samples = samples
        phase = "v8_stage_c_fresh_confirmation"

    summaries = []
    rows = []
    for candidate_id in candidate_ids:
        summary, values = runner.run_bank("xu2025", by_id[candidate_id], selected_samples, phase, args.workers)
        summaries.append(summary)
        rows.extend(values)
    stage = args.mode.replace("-", "_")
    runner.write_csv(PAPER / f"{stage}_results.csv", rows)
    runner.write_json(
        PAPER / f"{stage}_summary.json",
        {
            "stage": stage,
            "candidate_count": len(candidate_ids),
            "sample_count_each": len(selected_samples),
            "summaries": summaries,
            "selection_authority": bool(args.mode in {"stage-b", "stage-c"}),
            "holdout_accessed": False,
        },
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
