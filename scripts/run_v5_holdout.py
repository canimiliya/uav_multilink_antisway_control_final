"""Execute the frozen V5 Holdout exactly once for all eligible participants."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

try:
    from scripts import run_v5_self_development as runner
except ImportError:
    import run_v5_self_development as runner


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v5/holdout"
SELF = ROOT / "reproducibility/v5/self"


def read(path: Path) -> dict: return json.loads(path.read_text(encoding="utf-8"))


def execute(kind: str, parameters: dict, sample: dict, phase: str) -> dict:
    path = OUT / "_cache" / parameters["candidate_id"] / f"{sample['sample_id']}.json"
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


def parameters() -> dict[str, tuple[str, dict]]:
    values = {p[1]["candidate_id"]: p for p in runner.comparator_specs()[:4]}
    self_freeze = read(SELF / "self_freeze.json"); values[self_freeze["candidate_id"]] = ("satc_ofmpc", self_freeze["parameters"])
    return values


def main() -> int:
    parser = argparse.ArgumentParser(); parser.add_argument("--workers", type=int, default=12); args = parser.parse_args()
    protocol = read(OUT / "holdout_protocol_freeze.json"); manifest = read(OUT / "execution_manifest.json")
    if protocol["holdout_executed_at_freeze"] is not False or manifest["sample_count"] != 96: raise RuntimeError("Holdout protocol drift")
    specs = parameters(); all_rows = []; summaries = {}
    for participant in manifest["participants"]:
        kind, value = specs[participant["candidate_id"]]; summary, rows = runner.run_bank(kind, value, manifest["samples"], "v5_holdout", args.workers)
        summaries[participant["candidate_id"]] = summary; all_rows.extend(rows)
    write_csv(OUT / "holdout_results.csv", all_rows)
    runner.write_json(OUT / "holdout_controller_summary.json", {"sample_count": 96, "participants": summaries, "authoritative_runs": len(all_rows), "paper": "NOT_RUN_DEVELOPMENT_INELIGIBLE", "retries": [], "compromised": False})
    print(json.dumps({"samples": 96, "participants": len(summaries), "runs": len(all_rows), "compromised": False}, indent=2)); return 0


if __name__ == "__main__": raise SystemExit(main())
