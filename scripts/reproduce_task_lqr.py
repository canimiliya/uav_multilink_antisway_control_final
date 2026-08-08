"""Reproduce the frozen S6T2 Task-LQR calm and crosswind baselines."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import mujoco

from uav_sway.control.task_lqr import build_task_lqr
from uav_sway.evaluation.task_baseline_runner import run_task_baseline_scenario
from uav_sway.evaluation.task_space_metrics import compute_task_metrics
from uav_sway.evaluation.task_space_runner import run_task_space_scenario
from uav_sway.disturbances.wind_io import read_wind_csv


ROOT = Path(__file__).resolve().parents[1]
FROZEN = ROOT / "reproducibility/frozen"
MODEL_CONFIG = ROOT / "configs/model_5link.yaml"
RUNTIME_XML = FROZEN / "model/model_5link_controlled.xml"
SCENES = ("task_acquire_calm", "task_acquire_crosswind")


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def ensure_calm_wind(path: Path, times: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["time", "wind_x", "wind_y", "wind_z", "profile", "seed"])
        for value in times:
            writer.writerow([format(float(value), ".17g"), "0", "0", "0", "calm", ""])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", default="outputs/task_lqr")
    args = parser.parse_args()
    output = ROOT / args.output_dir
    output.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(RUNTIME_XML))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    times = read_wind_csv(FROZEN / "ls_pmpc/constant_crosswind.csv")["time"]
    calm_wind = output / "inputs/calm.csv"
    ensure_calm_wind(calm_wind, times)

    refs = {
        "task_acquire_calm": FROZEN / "task_space_contract/task_acquire_calm.csv",
        "task_acquire_crosswind": FROZEN / "task_space_contract/task_acquire_crosswind.csv",
    }
    winds = {
        "task_acquire_calm": calm_wind,
        "task_acquire_crosswind": FROZEN / "ls_pmpc/constant_crosswind.csv",
    }
    source_scenes = {
        "task_acquire_calm": {"name": "task_acquire_calm", "task_start_time_s": 1.0, "source_scenario": "approach_stop"},
        "task_acquire_crosswind": {"name": "task_acquire_crosswind", "task_start_time_s": 3.0, "source_scenario": "crosswind_hover"},
    }
    protocol = json.loads((FROZEN / "task_space_contract/protocol.json").read_text(encoding="utf-8"))
    protocol["protocol_sha256"] = sha256_file(FROZEN / "task_space_contract/protocol.json")
    protocol["reference_sha256_by_scene"] = {name: sha256_file(path) for name, path in refs.items()}
    protocol["wind_sha256_by_scene"] = {name: sha256_file(path) for name, path in winds.items()}

    old_metrics: dict[str, dict] = {}
    task_metrics: dict[str, dict] = {}
    q_s4 = np.diag([80, 4, 8, 2, 4, 1, 20, 20, 20, 20, 20, 12, 12, 12, 12, 12])
    a = np.load(FROZEN / "linear_model/A.npy")
    b = np.load(FROZEN / "linear_model/B.npy")
    c_task = np.load(FROZEN / "task_lqr/C_task.npy")
    task_result = build_task_lqr(a, b, c_task, 80.0, 5.0, 1.0, q_s4)
    candidate = {"candidate_id": "lqr_011", "index": 11, "w_p": 80.0, "w_theta": 5.0, "R": 1.0, "K": task_result["K"], "spectral_radius": task_result["spectral_radius"], "dare_residual_norm": task_result["dare_residual_norm"]}

    for scene in SCENES:
        source_scene = source_scenes[scene]
        old_path = output / "old_lqr" / scene / "run.csv"
        old_metrics[scene] = run_task_space_scenario(
            MODEL_CONFIG, "lqr", source_scene["source_scenario"], winds[scene], refs[scene], old_path, ROOT
        )
        task_path = output / "task_lqr" / scene / "run.csv"
        task_metrics[scene] = run_task_baseline_scenario(
            MODEL_CONFIG, "task_lqr", candidate, source_scene["source_scenario"], winds[scene], refs[scene],
            task_path, ROOT, duration_s=12.0, c_task=c_task
        )

    calm_pos = 100.0 * (old_metrics[SCENES[0]]["tip_task_position_rmse_m"] - task_metrics[SCENES[0]]["tip_task_position_rmse_m"]) / old_metrics[SCENES[0]]["tip_task_position_rmse_m"]
    calm_ori = 100.0 * (old_metrics[SCENES[0]]["cutter_orientation_rmse_deg"] - task_metrics[SCENES[0]]["cutter_orientation_rmse_deg"]) / old_metrics[SCENES[0]]["cutter_orientation_rmse_deg"]
    cross_pos = 100.0 * (task_metrics[SCENES[1]]["tip_task_position_rmse_m"] - old_metrics[SCENES[1]]["tip_task_position_rmse_m"]) / old_metrics[SCENES[1]]["tip_task_position_rmse_m"]
    cross_ori = 100.0 * (old_metrics[SCENES[1]]["cutter_orientation_rmse_deg"] - task_metrics[SCENES[1]]["cutter_orientation_rmse_deg"]) / old_metrics[SCENES[1]]["cutter_orientation_rmse_deg"]
    result = {
        "controller": "Task-LQR",
        "candidate_id": "lqr_011",
        "runtime_model_sha256": sha256_file(RUNTIME_XML),
        "calm": {"position_improvement_percent": calm_pos, "orientation_improvement_percent": calm_ori, "acquired": bool(task_metrics[SCENES[0]]["task_acquired"]), "acquisition_s": task_metrics[SCENES[0]]["task_acquisition_time_s"]},
        "crosswind": {"position_change_percent": cross_pos, "orientation_improvement_percent": cross_ori, "acquired": bool(task_metrics[SCENES[1]]["task_acquired"]), "acquisition_s": task_metrics[SCENES[1]]["task_acquisition_time_s"]},
        "expected": {"calm_position_improvement_percent": 17.407892, "calm_orientation_improvement_percent": 51.721920, "calm_acquisition_s": 2.455, "crosswind_position_change_percent": 6.328867, "crosswind_orientation_improvement_percent": 41.985099},
    }
    checks = {
        "calm_position": abs(calm_pos - result["expected"]["calm_position_improvement_percent"]) <= 0.10,
        "calm_orientation": abs(calm_ori - result["expected"]["calm_orientation_improvement_percent"]) <= 0.10,
        "calm_acquired": result["calm"]["acquired"],
        "calm_acquisition": abs(float(result["calm"]["acquisition_s"]) - 2.455) <= 0.01,
        "crosswind_position": abs(cross_pos - result["expected"]["crosswind_position_change_percent"]) <= 0.10,
        "crosswind_orientation": abs(cross_ori - result["expected"]["crosswind_orientation_improvement_percent"]) <= 0.10,
        "crosswind_not_acquired": not result["crosswind"]["acquired"],
    }
    result["checks"] = checks
    result["pass"] = bool(all(checks.values()))
    (output / "result.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps(result, indent=2))
    return 0 if result["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
