"""Freeze the V9 Neural Predictor benchmark before training or performance."""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
V7 = ROOT / "reproducibility/v7"
V9 = ROOT / "reproducibility/v9"
R0 = V9 / "r0"
SOURCE_HEAD = "92188e0eee0e92ed9405af07e80839360e30a19c"
PREDICTOR_SEARCH_SEED = 20260830
CONTROLLER_SEARCH_SEED = 20260831
BOOTSTRAP_SEED = 20260901


def write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
        newline="\n",
    )


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def predictor_registry() -> list[dict]:
    rng = np.random.Generator(np.random.PCG64(PREDICTOR_SEARCH_SEED))
    embedding = [18, 24, 32, 40]
    width = [64, 96, 128]
    depth = [2, 3]
    horizon = [10, 20]
    learning_rate = [3.0e-4, 6.0e-4, 1.0e-3]
    gamma = [0.90, 1.00, 1.10]
    result: list[dict] = []
    anchors = [
        (24, 128, 2, 20, 1.0e-3, 1.00, 1.0, 1.0, 1.0),
        (32, 128, 2, 20, 6.0e-4, 1.00, 1.0, 0.5, 1.0),
        (24, 96, 3, 10, 6.0e-4, 0.90, 1.0, 1.0, 0.5),
        (40, 128, 3, 20, 3.0e-4, 1.10, 1.0, 1.0, 1.0),
    ]
    values = list(anchors)
    while len(values) < 24:
        row = (
            int(rng.choice(embedding)),
            int(rng.choice(width)),
            int(rng.choice(depth)),
            int(rng.choice(horizon)),
            float(rng.choice(learning_rate)),
            float(rng.choice(gamma)),
            float(rng.choice([0.5, 1.0, 2.0])),
            float(rng.choice([0.5, 1.0])),
            float(rng.choice([0.5, 1.0, 2.0])),
        )
        if row not in values:
            values.append(row)
    for index, row in enumerate(values):
        result.append(
            {
                "model_id": f"np_v9_{index:02d}",
                "embedding_dim": row[0],
                "hidden_width": row[1],
                "hidden_depth": row[2],
                "prediction_horizon": row[3],
                "learning_rate": row[4],
                "spectral_gamma": row[5],
                "forward_weight": row[6],
                "backward_weight": row[7],
                "reconstruction_weight": row[8],
                "epochs_max": 240,
                "early_stopping_patience": 30,
                "window_batch_size": 4096,
                "training_seed": 20261000 + index,
            }
        )
    return result


def identification_manifest() -> dict:
    rng = np.random.Generator(np.random.PCG64(20260828))
    wind_kinds = ("calm", "constant_1p5", "constant_3p0", "stochastic")
    rows = []
    for index in range(240):
        direction = rng.normal(size=3)
        direction /= np.linalg.norm(direction)
        rows.append(
            {
                "trajectory_id": f"v9_ident_{index:03d}",
                "seed": 9000 + index,
                "split": "VALIDATION" if index % 5 == 4 else "TRAIN",
                "controller": "full_lqr_048",
                "duration_s": 12.0,
                "sample_period_s": 0.05,
                "reference": {
                    "kind": "bounded_multisine",
                    "amplitude_m": [0.30, 0.30, 0.20],
                    "frequency_hz": [
                        round(float(rng.uniform(0.06, 0.16)), 8),
                        round(float(rng.uniform(0.07, 0.18)), 8),
                        round(float(rng.uniform(0.05, 0.13)), 8),
                    ],
                    "phase_rad": [round(float(v), 8) for v in rng.uniform(0.0, 2.0 * np.pi, size=3)],
                },
                "wind": {
                    "kind": wind_kinds[index % 4],
                    "direction_world": [round(float(v), 10) for v in direction],
                    "seed": 12000 + index if index % 4 == 3 else None,
                },
                "physical_authority": {
                    "acceleration_limit_m_s2": 2.0,
                    "slew_limit_m_s2_per_update": 0.25,
                    "outer_period_s": 0.05,
                },
            }
        )
    return {
        "name": "V9_IDENTIFICATION_TRAINING_BANK",
        "generation_seed": 20260828,
        "trajectory_count": 240,
        "train_trajectories": 192,
        "validation_trajectories": 48,
        "split_rule": "whole trajectory; index mod 5 equals 4 is validation",
        "expected_timesteps": 57840,
        "controller": "frozen full_lqr_048 only",
        "satc_used": False,
        "wind_truth_used_as_training_input": False,
        "future_state_used_as_runtime_input": False,
        "holdout_overlap": False,
        "development_overlap": False,
        "trajectories": rows,
    }


def main() -> int:
    source = read(V9 / "upstream/source_freeze.json")
    if source["upstream_commit"] != "e2ababf519e7b7cca8e23f42dcb8c34fae927037":
        raise RuntimeError("upstream source drift")
    R0.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(V7 / "r0/development_manifest.json", R0 / "development_manifest.json")
    shutil.copyfile(V7 / "r0/holdout_manifest.json", R0 / "holdout_manifest.json")
    if sha(R0 / "development_manifest.json") != "b51d9958b706354867d593d61a93336c702561e1e3b3e9dfcd5488ced0886cc3":
        raise RuntimeError("Development carry-forward drift")
    if sha(R0 / "holdout_manifest.json") != "3c903826ec0d6ba3ea0c941a07f7499f0863c38b8474a40c5acc2a1d14b87d65":
        raise RuntimeError("Holdout carry-forward drift")
    write(R0 / "identification_manifest.json", identification_manifest())
    write(
        R0 / "predictor_registry.json",
        {
            "search_seed": PREDICTOR_SEARCH_SEED,
            "maximum_allowed": 64,
            "registered_count": 24,
            "models": predictor_registry(),
            "selection_inputs": [
                "validation force RMSE",
                "validation torque RMSE",
                "validation multi-step RMSE",
                "lifted spectral radius",
                "trajectory-level consistency",
            ],
            "satc_performance_used_for_selection": False,
        },
    )
    write(
        R0 / "research_contract.json",
        {
            "task": "V9-NEURAL-PREDICTOR-MPC-RECENT-PAPER-STRONG-BASELINE-AND-FINAL-CLOSURE-R1",
            "source_head": SOURCE_HEAD,
            "branch": "research-v9",
            "paper": source,
            "primary_goal": "Recent Paper stronger than frozen Traditional on Development and inherited unseen Holdout",
            "protected": {
                "versions": "V1-V8 tags and evidence",
                "traditional": ["hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009"],
                "self": ["self_a_034", "satc_b_027"],
                "plant": "reproducibility/frozen/model/model_5link_controlled.xml",
            },
            "common_interface": {
                "command": "[ax, ay, az] world-frame acceleration",
                "outer_rate_hz": 20,
                "component_limit_m_s2": 2.0,
                "component_slew_limit_m_s2_per_update": 0.25,
                "inner_loop": "same geometric inner loop",
                "paper_torque_actuator_authority": False,
            },
            "data_isolation": [
                "V9 identification/training bank",
                "V7 frozen 144-sample Development",
                "V7 frozen 112-sample one-shot Holdout",
            ],
            "training_data": {
                "trajectory_count": 240,
                "split": "192 TRAIN / 48 VALIDATION by whole trajectory",
                "controller": "full_lqr_048",
                "label_source": "finite differences plus actual command and nominal dynamics",
                "wind_truth_input": False,
            },
            "search_limits": {
                "neural_predictor_models_max": 64,
                "neural_predictor_models_registered": 24,
                "np_mpc_configurations_max": 128,
                "np_mpc_configurations_planned": 48,
            },
            "np_mpc": {
                "nominal_model": "frozen 20-state discrete linear model",
                "learned_term": "predicted force/torque state mapped inside every horizon transition",
                "optimizer": "OSQP constrained receding-horizon QP",
                "online_lifted_update_window": 40,
                "satc_modules_forbidden": True,
            },
            "stopping": {
                "predictor_incompetent": "BLOCKED_V9_NEURAL_PREDICTOR_NOT_VALIDATED",
                "development_fail": "V9_NEURAL_PREDICTOR_PAPER_NOT_STRONGER_THAN_TRADITIONAL",
                "paper_replacement": False,
                "automatic_v10": False,
            },
            "performance_accessed": False,
            "holdout_accessed": False,
        },
    )
    write(
        R0 / "win_contract.json",
        {
            "primary": "full_lqr_048",
            "best_traditional_scope": [
                "hybrid_x007_y041_z041",
                "full_lqr_048",
                "task_lqr_009",
            ],
            "development_and_holdout_gates": {
                "safety_rate": "not below best Traditional",
                "success_rate": "not below best Traditional",
                "position_mean": "at least 5 percent better than full_lqr_048",
                "acquisition_median": "no more than 5 percent worse than full_lqr_048",
                "paired_bootstrap": "95 percent lower bound greater than zero",
                "strong_mean": "at least 5 percent better than full_lqr_048",
                "strong_p90": "no more than 10 percent worse than full_lqr_048",
                "catastrophic_pairs": 0,
            },
            "additional_value_any_one": [
                "acquisition at least 5 percent better",
                "orientation at least 10 percent better",
                "strong mean at least 10 percent better",
                "effort at least 10 percent better",
                "another preregistered paper-native advantage",
            ],
            "bootstrap": {
                "resamples": 10000,
                "seed": BOOTSTRAP_SEED,
                "pairing": "exact sample_id",
            },
            "holdout_unlock": [
                "predictor frozen",
                "Paper Development qualified",
                "NP-MPC frozen",
                "all search stopped",
            ],
        },
    )
    write(
        R0 / "controller_search_contract.json",
        {
            "search_seed": CONTROLLER_SEARCH_SEED,
            "maximum_unique_configurations": 128,
            "planned_unique_configurations": 48,
            "stage_a": {
                "candidates": 48,
                "representative_samples": 24,
                "selection_authority": "rank top 8 for Stage B",
            },
            "stage_b": {
                "candidates": 8,
                "samples_each": 144,
                "qualification_authority": True,
            },
            "stage_c": {
                "trigger": "at least one Stage-B candidate passes all gates",
                "candidate": "single preregistered lexicographic winner",
                "fresh_confirmation_samples": 144,
            },
            "parameter_families": [
                "prediction horizon",
                "position/velocity/orientation/angular task weights",
                "input and rate weights",
                "learned-force mapping scale",
                "online lifted update ridge and blending",
            ],
            "satc_used_for_search": False,
        },
    )
    write(
        R0 / "contract_hashes.json",
        {
            "development_manifest": sha(R0 / "development_manifest.json"),
            "holdout_manifest": sha(R0 / "holdout_manifest.json"),
            "identification_manifest": sha(R0 / "identification_manifest.json"),
            "predictor_registry": sha(R0 / "predictor_registry.json"),
            "research_contract": sha(R0 / "research_contract.json"),
            "win_contract": sha(R0 / "win_contract.json"),
            "controller_search_contract": sha(R0 / "controller_search_contract.json"),
        },
    )
    print(
        json.dumps(
            {
                "training_trajectories": 240,
                "predictor_models": 24,
                "mpc_candidates_planned": 48,
                "development_sha256": sha(R0 / "development_manifest.json"),
                "holdout_sha256": sha(R0 / "holdout_manifest.json"),
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
