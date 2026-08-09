"""Run only the frozen V5 Paper Development protocol."""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path

import numpy as np

try:
    from scripts import run_v5_self_development as runner
except ImportError:
    import run_v5_self_development as runner
from uav_sway.v5.jirousek2025_incremental_mpc import Jirousek2025IncrementalMPC


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v5/r0"
PAPER = ROOT / "reproducibility/v5/paper"


def read(path: Path) -> dict: return json.loads(path.read_text(encoding="utf-8"))


def paper_controller(kind: str, parameters: dict):
    if kind != "paper_incremental_mpc": return runner.build_controller(kind, parameters)
    a, b, c = runner.model_data(); return Jirousek2025IncrementalMPC(a, b, c, parameters)


runner.core.controller = paper_controller


def execute(kind: str, parameters: dict, sample: dict, phase: str) -> dict:
    path = PAPER / "_cache" / phase / parameters["candidate_id"] / f"{sample['sample_id']}.json"
    if path.exists(): return read(path)
    row = runner.core.run_case(kind, parameters, sample); row["directional_stratum"] = sample["directional_stratum"]
    runner.write_json(path, row); return row


runner.execute = execute


def write_csv(path: Path, rows: list[dict]) -> None:
    columns: list[str] = []
    for row in rows:
        for key in row:
            if key not in columns: columns.append(key)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns, lineterminator="\n"); writer.writeheader(); writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--mode", choices=("stage-a", "stage-b"), required=True); parser.add_argument("--workers", type=int, default=min(24, os.cpu_count() or 1)); args = parser.parse_args()
    protocol = read(PAPER / "adaptation_protocol.json"); manifest = read(R0 / "development_manifest.json"); samples = manifest["samples"]
    key = args.mode.replace("-", "_"); candidates = protocol["search"][key]["candidates"]
    if key == "stage_a":
        smoke_ids = set(read(ROOT / "reproducibility/v5/self/development_protocol.json")["search"]["stage_a"]["sample_ids"]); samples = [row for row in samples if row["sample_id"] in smoke_ids]
    summaries = []; all_rows = []
    for parameters in candidates:
        summary, rows = runner.run_bank("paper_incremental_mpc", parameters, samples, f"paper_{key}", args.workers); summaries.append(summary); all_rows.extend(rows)
    runner.write_json(PAPER / f"{key}_summary.json", {"candidate_count": len(candidates), "sample_count_each": len(samples), "summaries": summaries, "holdout_executed": False})
    write_csv(PAPER / f"{key}_results.csv", all_rows)
    return 0


if __name__ == "__main__": raise SystemExit(main())
