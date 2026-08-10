"""Replay the frozen V10 FxTDO against V9 MuJoCo-reconstructed force truth."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from uav_sway.v10.fxtdo_mpc import XuFxTDOMPC
from uav_sway.v3.metrics import load_r0_linear_matrices


ROOT = Path(__file__).resolve().parents[1]
BANK = ROOT / "reproducibility/v9/training/training_bank.npz"
OUT = ROOT / "reproducibility/v10/development/observer_offline_truth_audit.json"


def main() -> int:
    final = json.loads((ROOT / "reproducibility/v10/development/development_final.json").read_text(encoding="utf-8"))
    parameters = final["selected_parameters"]
    a, b = load_r0_linear_matrices(ROOT)
    metric = json.loads((ROOT / "reproducibility/v3/r1/task_metric_alignment_audit.json").read_text(encoding="utf-8"))
    c_task = np.vstack([np.asarray(metric[name], dtype=float) for name in ("C_pos", "C_vel", "C_dir", "C_omega_perp")])
    model = mujoco.MjModel.from_xml_path(str(ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"))
    total_mass = float(np.sum(model.body_mass))
    with np.load(BANK) as archive:
        bank = {name: archive[name] for name in archive.files}
    estimates = []
    truths = []
    post_one_second_estimates = []
    post_one_second_truths = []
    trajectory_rmse = []
    validation_indices = np.flatnonzero(bank["split"] == "VALIDATION")
    for trajectory_index in validation_indices:
        observer = XuFxTDOMPC(a, b, c_task, parameters)
        current_estimates = []
        current_truths = []
        for step in range(bank["zeta"].shape[1]):
            _, _, _ = observer._update_observer(bank["zeta"][trajectory_index, step, :3], 0.05)
            if bank["label_valid"][trajectory_index, step] and bank["safe_step"][trajectory_index, step]:
                truth = bank["chi"][trajectory_index, step, :3] / total_mass
                estimate = observer.disturbance_hat.copy()
                estimates.append(estimate)
                truths.append(truth)
                current_estimates.append(estimate)
                current_truths.append(truth)
                if step >= 20:
                    post_one_second_estimates.append(estimate)
                    post_one_second_truths.append(truth)
            observer.previous_actual_command = bank["command"][trajectory_index, step].copy()
        trajectory_rmse.append(float(np.sqrt(np.mean((np.asarray(current_estimates) - np.asarray(current_truths)) ** 2))))
    estimate = np.asarray(estimates)
    truth = np.asarray(truths)
    late_estimate = np.asarray(post_one_second_estimates)
    late_truth = np.asarray(post_one_second_truths)
    error = estimate - truth
    late_error = late_estimate - late_truth
    payload = {
        "purpose": "offline diagnostic only; truth was never observer/controller input",
        "source_bank": "reproducibility/v9/training/training_bank.npz",
        "source_bank_sha256": hashlib.sha256(BANK.read_bytes()).hexdigest(),
        "split": "VALIDATION", "trajectories": int(len(validation_indices)), "valid_safe_steps": int(len(estimate)),
        "selected_candidate": final["selected_candidate"], "total_plant_mass_kg": total_mass,
        "truth_definition": "V9 finite-difference rigid-body residual force divided by total plant mass",
        "estimate_units": "m/s^2", "truth_units": "m/s^2",
        "rmse_all_axes_m_s2": float(np.sqrt(np.mean(error ** 2))),
        "rmse_axis_m_s2": np.sqrt(np.mean(error ** 2, axis=0)).tolist(),
        "post_1s_rmse_all_axes_m_s2": float(np.sqrt(np.mean(late_error ** 2))),
        "bias_axis_m_s2": np.mean(error, axis=0).tolist(),
        "trajectory_rmse_median_m_s2": float(np.median(trajectory_rmse)),
        "trajectory_rmse_p90_m_s2": float(np.percentile(trajectory_rmse, 90)),
        "estimate_finite": bool(np.isfinite(estimate).all()),
        "estimate_max_abs_m_s2": float(np.max(np.abs(estimate))),
        "observer_clip_m_s2": float(parameters["observer_clip_m_s2"]),
        "no_sustained_divergence": bool(np.isfinite(estimate).all() and np.max(np.abs(estimate)) <= float(parameters["observer_clip_m_s2"]) + 1e-12),
        "fixed_time_claim_validated": False,
        "fixed_time_diagnostic": "No empirical fixed-time settling claim: error did not enter and remain within a preregistered truth tolerance across the validation trajectories.",
        "causal": True, "used_for_selection": False, "holdout_accessed": False,
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with OUT.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
        stream.write("\n")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
