"""Post-freeze Development-only ablation for V3-R2 Self-Advanced."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

from run_v3_r1_baselines import read_json, run_candidates, summary_without_rows, write_csv, write_json  # noqa: E402


R1 = ROOT / "reproducibility/v3/r1"
R1R1 = ROOT / "reproducibility/v3/r1r1"
R2 = ROOT / "reproducibility/v3/r2"
FREEZE_COMMIT = "58b6544ab6a123f08d1cf04f3b9a7b38a40647e5"


def committed_freeze() -> dict:
    content = subprocess.check_output(
        ["git", "show", f"{FREEZE_COMMIT}:reproducibility/v3/r2/self_freeze.json"], cwd=ROOT, text=True
    )
    committed = json.loads(content)
    current = read_json(R2 / "self_freeze.json")
    if committed != current:
        raise RuntimeError("committed Self freeze changed before ablation")
    return current


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()
    freeze = committed_freeze()
    manifest = read_json(R1 / "development_evaluation_manifest.json")
    if manifest["split"] != "development" or len(manifest["samples"]) != 75:
        raise RuntimeError("ablation refuses non-development or drifted sample banks")
    base = freeze["parameters"]
    variants = []
    for name, residual_enabled, predictive_enabled in (
        ("predictive_only", False, True),
        ("residual_only", True, False),
    ):
        parameters = dict(base)
        parameters.update({
            "candidate_id": f"self_a_034__{name}",
            "residual_enabled": residual_enabled,
            "predictive_enabled": predictive_enabled,
        })
        variants.append(parameters)
    summaries = run_candidates("self_dr_tsrmpc", variants, manifest["samples"], args.workers, "v3-r2-ablation")
    write_csv(R2 / "ablation_results.csv", [summary_without_rows(row) for row in summaries])
    write_csv(R2 / "ablation_development_results.csv", [
        {**sample, "candidate_id": row["candidate_id"]} for row in summaries for sample in row["rows"]
    ])
    backbone = read_json(R1R1 / "traditional_development_summary.json")["final_selected"]["task_lqr"]
    payload = {
        "executed_after_committed_self_freeze": True,
        "self_freeze_commit": FREEZE_COMMIT,
        "frozen_candidate": freeze["candidate_id"],
        "no_ablation_retuning": True,
        "development_only": True,
        "backbone": backbone,
        "predictive_only": summary_without_rows(summaries[0]),
        "residual_only": summary_without_rows(summaries[1]),
        "full": freeze["metrics"],
        "holdout_executed": False,
    }
    write_json(R2 / "self_ablation.json", payload)
    print(json.dumps({
        "predictive_only": summaries[0]["candidate_id"],
        "residual_only": summaries[1]["candidate_id"],
        "full": freeze["candidate_id"],
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
