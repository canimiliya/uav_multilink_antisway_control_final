"""Post-freeze V3-R2 Self ablation and representative Development traces."""

from __future__ import annotations

import csv
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "scripts") not in sys.path:
    sys.path.insert(0, str(ROOT / "scripts"))

from run_v3_r1_baselines import read_json, run_candidates, run_case, summary_without_rows, write_csv, write_json  # noqa: E402


R1 = ROOT / "reproducibility/v3/r1"
R1R1 = ROOT / "reproducibility/v3/r1r1"
R2 = ROOT / "reproducibility/v3/r2"
SELF_FREEZE_HEAD = "58b6544"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def scalar(summary: dict) -> dict:
    return summary_without_rows(summary)


def representative_traces(parameters: dict, samples: list[dict]) -> dict:
    representative_ids = (
        "calm_axis_00",
        "constant_3_face_diagonal_00",
        "stochastic_2000",
        "ramp_wind_equilibrium_hold",
    )
    directory = R2 / "representative"
    directory.mkdir(parents=True, exist_ok=True)
    csv_paths = []
    for sample_id in representative_ids:
        sample = next(item for item in samples if item["sample_id"] == sample_id)
        path = directory / f"{sample_id}.csv"
        run_case("self_dr_tsrmpc", parameters, sample, str(path))
        csv_paths.append(path)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for axis, path in zip(axes.flat, csv_paths):
        with path.open("r", encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        time_s = np.asarray([float(row["time"]) for row in rows])
        position = np.asarray([float(row["position_error_3d_m"]) for row in rows])
        axis.plot(time_s, position, color="#1565c0", linewidth=1.25)
        axis.axhline(0.05, color="#222222", linestyle="--", linewidth=0.8, label="acquisition position gate")
        axis.set_title(path.stem)
        axis.set_xlabel("time [s]")
        axis.set_ylabel("3D cutter-tip error [m]")
        axis.grid(alpha=0.25)
    figure_path = directory / "selected_position_error.png"
    fig.savefig(figure_path, dpi=180)
    plt.close(fig)
    manifest = {
        "selected_candidate": parameters["candidate_id"],
        "development_only": True,
        "traces": [
            {"sample_id": path.stem, "path": path.relative_to(ROOT).as_posix(), "sha256": sha256(path)}
            for path in csv_paths
        ],
        "plots": [{
            "path": figure_path.relative_to(ROOT).as_posix(),
            "sha256": sha256(figure_path),
            "purpose": "representative frozen Self 3D cutter-tip position-error trajectories",
        }],
        "holdout_executed": False,
    }
    write_json(R2 / "visual_manifest.json", manifest)
    return manifest


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args()
    if git("branch", "--show-current") != "research-v3":
        raise RuntimeError("V3-R2 ablation may run only on research-v3")
    if subprocess.run(
        ["git", "merge-base", "--is-ancestor", SELF_FREEZE_HEAD, "HEAD"], cwd=ROOT, check=False
    ).returncode != 0:
        raise RuntimeError("the committed Self freeze must precede ablation")
    if subprocess.run(
        ["git", "diff", "--quiet", "HEAD", "--", "reproducibility/v3/r2/self_freeze.json"],
        cwd=ROOT,
        check=False,
    ).returncode != 0:
        raise RuntimeError("self_freeze.json changed after the freeze commit")

    freeze = read_json(R2 / "self_freeze.json")
    if freeze["candidate_id"] != "self_a_034" or freeze["frozen_before_ablation"] is not True:
        raise RuntimeError("unexpected Self freeze")
    manifest = read_json(R1 / "development_evaluation_manifest.json")
    if manifest["split"] != "development" or len(manifest["samples"]) != 75:
        raise RuntimeError("ablation refuses non-Development samples")
    samples = manifest["samples"]

    candidates = []
    for name, residual_enabled, predictive_enabled in (
        ("predictive_only", False, True),
        ("residual_only", True, False),
    ):
        parameters = dict(freeze["parameters"])
        parameters.update({
            "candidate_id": f"self_a_034__{name}",
            "residual_enabled": residual_enabled,
            "predictive_enabled": predictive_enabled,
        })
        candidates.append(parameters)
    summaries = run_candidates("self_dr_tsrmpc", candidates, samples, args.workers, "v3-r2-ablation")
    by_id = {summary["candidate_id"]: summary for summary in summaries}
    predictive = scalar(by_id["self_a_034__predictive_only"])
    residual = scalar(by_id["self_a_034__residual_only"])
    backbone = read_json(R1R1 / "traditional_development_summary.json")["final_selected"]["task_lqr"]
    full = freeze["metrics"]
    rows = []
    for name, value in (("backbone", backbone), ("predictive_only", predictive), ("residual_only", residual), ("full", full)):
        rows.append({"ablation": name, **{key: item for key, item in value.items() if key != "parameters"}})
    write_csv(R2 / "ablation_results.csv", rows)
    write_json(R2 / "self_ablation.json", {
        "executed_after_committed_self_freeze": True,
        "self_freeze_head": git("rev-parse", SELF_FREEZE_HEAD),
        "frozen_candidate": freeze["candidate_id"],
        "frozen_parameters_unchanged": True,
        "no_ablation_retuning": True,
        "interpretation": {
            "backbone": "frozen task_lqr_009 evidence; no rerun",
            "predictive_only": "frozen backbone plus predictive QP with dynamic residual compensation disabled",
            "residual_only": "frozen backbone plus dynamic residual steady compensation with predictive correction disabled",
            "full": "frozen full 3D-DR-TSRMPC evidence; no rerun",
        },
        "backbone": backbone,
        "predictive_only": predictive,
        "residual_only": residual,
        "full": full,
        "development_only": True,
        "holdout_executed": False,
    })
    representative_traces(freeze["parameters"], samples)
    gate = read_json(R2 / "gate.json")
    gate["ablation_executed"] = True
    gate["ablation_after_committed_freeze"] = True
    gate["result"] = "V3_SELF_ADVANCED_FROZEN"
    write_json(R2 / "gate.json", gate)
    print(json.dumps({
        "result": gate["result"],
        "selected": freeze["candidate_id"],
        "win_level": freeze["win_level"],
        "ablation": {
            "backbone_success": backbone["success_rate"],
            "predictive_only_success": predictive["success_rate"],
            "residual_only_success": residual["success_rate"],
            "full_success": full["success_rate"],
        },
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
