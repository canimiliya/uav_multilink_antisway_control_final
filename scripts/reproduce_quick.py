"""Run a small deterministic release smoke across the public controllers."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np
import yaml

from uav_sway.control.task_lqr import build_task_lqr
from uav_sway.disturbances.wind_io import read_wind_csv
from uav_sway.evaluation.da_pmpc_runner import run_scene
from uav_sway.evaluation.lqr_runner import run_lqr_scenario
from uav_sway.evaluation.task_baseline_runner import run_task_baseline_scenario


ROOT = Path(__file__).resolve().parents[1]
FROZEN = ROOT / "reproducibility/frozen"
RUNTIME = FROZEN / "model/model_5link_controlled.xml"
EXPECTED_RUNTIME_SHA = "19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d"


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def calm_wind(path: Path, times: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["time", "wind_x", "wind_y", "wind_z", "profile", "seed"])
        for t in times:
            writer.writerow([format(float(t), ".17g"), "0", "0", "0", "calm", ""])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="outputs/quick/result.json")
    args = parser.parse_args()
    output = ROOT / args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(RUNTIME))
    runtime_sha = sha256_file(RUNTIME)
    wind = FROZEN / "ls_pmpc/constant_crosswind.csv"
    reference = FROZEN / "ls_pmpc/crosswind_hover.csv"
    times = read_wind_csv(wind)["time"]
    calm = output.parent / "calm.csv"
    calm_wind(calm, times)

    lqr_config = yaml.safe_load((ROOT / "configs/lqr.yaml").read_text(encoding="utf-8"))
    lqr_metrics = run_lqr_scenario(ROOT / "configs/model_5link.yaml", lqr_config, "crosswind_hover", wind, reference, output.parent / "lqr.csv", ROOT, True, gain=np.load(FROZEN / "linear_model/K.npy"))
    da_config = yaml.safe_load((ROOT / "configs/da_pmpc.yaml").read_text(encoding="utf-8"))
    pmpc_metrics = run_scene(ROOT / "configs/model_5link.yaml", da_config, "crosswind_hover", wind, reference, output.parent / "ls_pmpc.csv", mode="preview", duration_s=12.0)

    task_result = build_task_lqr(
        np.load(FROZEN / "linear_model/A.npy"), np.load(FROZEN / "linear_model/B.npy"),
        np.load(FROZEN / "task_lqr/C_task.npy"), 80.0, 5.0, 1.0,
        np.diag([80, 4, 8, 2, 4, 1, 20, 20, 20, 20, 20, 12, 12, 12, 12, 12]),
    )
    task_candidate = {"candidate_id": "lqr_011", "index": 11, "w_p": 80.0, "w_theta": 5.0, "R": 1.0, "K": task_result["K"], "spectral_radius": task_result["spectral_radius"], "dare_residual_norm": task_result["dare_residual_norm"]}
    task_protocol = json.loads((FROZEN / "task_space_contract/protocol.json").read_text(encoding="utf-8"))
    task_protocol["protocol_sha256"] = sha256_file(FROZEN / "task_space_contract/protocol.json")
    task_ref = FROZEN / "task_space_contract/task_acquire_calm.csv"
    task_protocol["reference_sha256_by_scene"] = {"task_acquire_calm": sha256_file(task_ref)}
    task_protocol["wind_sha256_by_scene"] = {"task_acquire_calm": sha256_file(calm)}
    task_metrics = run_task_baseline_scenario(
        ROOT / "configs/model_5link.yaml", "task_lqr", task_candidate,
        "approach_stop", calm, task_ref, output.parent / "task_lqr_calm.csv", ROOT,
        duration_s=12.0, c_task=np.load(FROZEN / "task_lqr/C_task.npy"),
    )
    checks = {
        "runtime_model_sha256": runtime_sha == EXPECTED_RUNTIME_SHA,
        "runtime_model_finite": bool(np.isfinite(model.body_mass).all()),
        "lqr_finite": bool(lqr_metrics.get("finite_outputs", True)),
        "ls_pmpc_finite": bool(np.isfinite(pmpc_metrics["tip_rms_m"])) and pmpc_metrics.get("primary_safety_failures", 0) == 0,
        "ls_pmpc_solver": bool(pmpc_metrics.get("qp_status_code", 1.0) > 0),
        "task_lqr_finite": bool(task_metrics.get("finite_outputs", True)),
        "task_lqr_acquired": bool(task_metrics.get("task_acquired", False)),
    }
    result = {"runtime_model_sha256": runtime_sha, "controllers": {"lqr": lqr_metrics, "ls_pmpc": pmpc_metrics, "task_lqr_calm": task_metrics}, "checks": checks, "pass": bool(all(checks.values()))}
    output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps(result, indent=2))
    return 0 if result["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
