"""Run only the frozen V10 Paper Development protocol."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

try:
    from scripts import run_v5_self_development as runner
except ImportError:
    import run_v5_self_development as runner
from uav_sway.v10.fxtdo_mpc import XuFxTDOMPC


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v10/r0"
OUT = ROOT / "reproducibility/v10/development"
runner.R0 = R0
runner.SELF = OUT
runner.core.R0 = R0
runner.core.R1 = OUT
base_build_controller = runner.build_controller


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def build_controller(kind: str, parameters: dict):
    if kind == "fxtdo_mpc":
        a, b, c_task = runner.model_data()
        return XuFxTDOMPC(a, b, c_task, parameters)
    return base_build_controller(kind, parameters)


runner.core.controller = build_controller


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("stage-a", "stage-b"), required=True)
    parser.add_argument("--workers", type=int, default=min(24, os.cpu_count() or 1))
    args = parser.parse_args()
    manifest = read(R0 / "development_manifest.json")
    if manifest["name"] != "V7_FROZEN_DEVELOPMENT_BANK" or len(manifest["samples"]) != 144:
        raise RuntimeError("V10 Development manifest drift")
    protocol = read(R0 / "search_contract.json")
    stage = args.mode.replace("-", "_")
    if stage == "stage_a":
        ids = set(protocol["stage_a"]["sample_ids"])
        samples = [row for row in manifest["samples"] if row["sample_id"] in ids]
        candidates = protocol["candidates"]
    else:
        selection = read(OUT / "stage_b_selection.json")
        if selection["source"] != "stage_a_summary.json" or len(selection["selected_ids"]) != protocol["stage_b"]["advance_top"]:
            raise RuntimeError("Stage B selection drift")
        selected = set(selection["selected_ids"])
        candidates = [row for row in protocol["candidates"] if row["candidate_id"] in selected]
        samples = manifest["samples"]
    all_rows = []
    summaries = []
    for parameters in candidates:
        summary, rows = runner.run_bank("fxtdo_mpc", parameters, samples, f"v10_{stage}", args.workers)
        summaries.append(summary)
        all_rows.extend(rows)
    runner.write_csv(OUT / f"{stage}_results.csv", all_rows)
    runner.write_json(OUT / f"{stage}_summary.json", {
        "stage": stage, "candidate_count": len(candidates), "sample_count_each": len(samples),
        "summaries": summaries, "unique_configs_total": 64, "holdout_accessed": False,
    })
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
