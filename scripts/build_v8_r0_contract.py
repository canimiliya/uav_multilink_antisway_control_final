"""Freeze the V8 physics audit, carried-forward splits, and research contract.

This script performs no controller evaluation.  Modal selection uses only the
frozen MuJoCo plant at its hanging equilibrium.
"""

from __future__ import annotations

import hashlib
import json
import shutil
from pathlib import Path

import mujoco
import numpy as np
from scipy.integrate import solve_ivp
from scipy.linalg import eigh


ROOT = Path(__file__).resolve().parents[1]
V7 = ROOT / "reproducibility/v7"
R0 = ROOT / "reproducibility/v8/r0"
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
SOURCE_HEAD = "86420873934f9e459a69676468059b9bf65177d5"


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
        newline="\n",
    )


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def modal_audit() -> tuple[dict, dict, dict]:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    data.eq_active[:] = 0
    mujoco.mj_forward(model, data)

    joint_ids = [
        int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}"))
        for i in range(1, 6)
    ]
    qpos_adr = np.asarray([model.jnt_qposadr[value] for value in joint_ids], dtype=int)
    dof_adr = np.asarray([model.jnt_dofadr[value] for value in joint_ids], dtype=int)
    full_mass = np.zeros((model.nv, model.nv), dtype=float)
    mujoco.mj_fullM(model, full_mass, data.qM)
    mass = full_mass[np.ix_(dof_adr, dof_adr)]
    damping = np.diag(np.asarray(model.dof_damping[dof_adr], dtype=float))

    gravity_stiffness = np.zeros((5, 5), dtype=float)
    epsilon = 1.0e-6
    for column, address in enumerate(qpos_adr):
        data.qpos[address] = epsilon
        mujoco.mj_forward(model, data)
        plus = np.asarray(data.qfrc_bias[dof_adr], dtype=float).copy()
        data.qpos[address] = -epsilon
        mujoco.mj_forward(model, data)
        minus = np.asarray(data.qfrc_bias[dof_adr], dtype=float).copy()
        data.qpos[address] = 0.0
        gravity_stiffness[:, column] = (plus - minus) / (2.0 * epsilon)
    gravity_stiffness = 0.5 * (gravity_stiffness + gravity_stiffness.T)
    mujoco.mj_forward(model, data)

    eigenvalues, modes = eigh(gravity_stiffness, mass)
    frequencies = np.sqrt(np.maximum(eigenvalues, 0.0)) / (2.0 * np.pi)
    for index in range(5):
        pivot = int(np.argmax(np.abs(modes[:, index])))
        if modes[pivot, index] < 0.0:
            modes[:, index] *= -1.0

    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    jacp = np.zeros((3, model.nv), dtype=float)
    jacr = np.zeros((3, model.nv), dtype=float)
    mujoco.mj_jacSite(model, data, jacp, jacr, tip_id)
    tip_position_jacobian = jacp[:, dof_adr]
    tip_orientation_jacobian = jacr[:, dof_adr]
    base_translation_coupling = full_mass[np.ix_(dof_adr, [0, 1, 2])]

    rows = []
    for index in range(5):
        vector = modes[:, index]
        omega = 2.0 * np.pi * frequencies[index]
        modal_damping = float(vector @ damping @ vector)
        damping_ratio = modal_damping / (2.0 * omega)
        tip_position = float(np.linalg.norm(tip_position_jacobian @ vector))
        tip_orientation = float(np.linalg.norm(tip_orientation_jacobian @ vector))
        control_participation = float(np.linalg.norm(vector @ base_translation_coupling))
        cutter_kinetic_participation = float(
            model.body_mass[int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cutter"))]
            * tip_position**2
        )
        rows.append(
            {
                "mode_id": index + 1,
                "natural_frequency_hz": float(frequencies[index]),
                "damping_ratio": float(damping_ratio),
                "mass_normalized_joint_shape": vector.tolist(),
                "cutter_tip_position_participation_m": tip_position,
                "cutter_tip_orientation_participation_rad": tip_orientation,
                "cutter_tip_kinetic_energy_proxy": cutter_kinetic_participation,
                "uav_translation_control_participation": control_participation,
            }
        )

    # A base-translation half-sine pulse is a plant-only diagnostic.  Modes 1
    # and 2 are the only modes below 2 Hz (>=10 outer-loop updates per cycle).
    # Their linear tip-response parity is evaluated without a controller.
    selected = [0, 1]
    input_vector = -base_translation_coupling[:, 0]
    inverse_mass = np.linalg.inv(mass)

    def dynamics(time_s: float, state: np.ndarray) -> np.ndarray:
        q = state[:5]
        qdot = state[5:]
        acceleration = 0.1 * np.sin(np.pi * time_s / 0.25) if time_s <= 0.25 else 0.0
        qddot = inverse_mass @ (
            input_vector * acceleration - damping @ qdot - gravity_stiffness @ q
        )
        return np.r_[qdot, qddot]

    times = np.linspace(0.0, 12.0, 12001)
    response = solve_ivp(
        dynamics,
        (0.0, 12.0),
        np.zeros(10),
        t_eval=times,
        rtol=1.0e-10,
        atol=1.0e-12,
    )
    joint_response = response.y[:5]
    full_tip = tip_position_jacobian @ joint_response
    selected_modes = modes[:, selected]
    coordinates = selected_modes.T @ mass @ joint_response
    reduced_tip = tip_position_jacobian @ selected_modes @ coordinates
    residual = full_tip - reduced_tip
    full_rms = float(np.sqrt(np.mean(full_tip[0] ** 2)))
    residual_rms = float(np.sqrt(np.mean(residual[0] ** 2)))
    normalized_rmse = residual_rms / full_rms
    peak_ratio = float(np.max(np.abs(reduced_tip[0])) / np.max(np.abs(full_tip[0])))

    audit = {
        "model_path": "reproducibility/frozen/model/model_5link_controlled.xml",
        "model_sha256": sha256(MODEL),
        "plant_modified": False,
        "equilibrium": {"quadrotor_position_m": [0.0, 0.0, 3.2], "joint_angles_rad": [0.0] * 5},
        "linearization": {
            "method": "MuJoCo mass matrix plus centered finite-difference gravity stiffness",
            "finite_difference_step_rad": epsilon,
            "mass_matrix_kg_m2": mass.tolist(),
            "gravity_stiffness_Nm_per_rad": gravity_stiffness.tolist(),
            "joint_damping_Nm_s_per_rad": np.diag(damping).tolist(),
        },
        "passive_mode_count": 5,
        "modes": rows,
        "performance_accessed": False,
    }
    selection = {
        "selection_before_paper_performance": True,
        "performance_accessed": False,
        "criteria": [
            "natural frequency below 2 Hz so the frozen 20 Hz outer loop provides at least ten updates per cycle",
            "nonzero cutter-tip position and orientation observability",
            "nonzero UAV-translation control participation",
            "largest low-frequency plant-energy participation",
        ],
        "eligible_mode_ids": [1, 2],
        "selected_mode_ids": [1, 2],
        "selected_frequencies_hz": [float(frequencies[0]), float(frequencies[1])],
        "modal_basis_mass_normalized": selected_modes.tolist(),
        "joint_to_modal_projection": (selected_modes.T @ mass).tolist(),
        "selection_authority": "FROZEN_PLANT_PHYSICS_ONLY",
    }
    identification = {
        "representation": "DIRECT_TWO_MODE_MODAL_COORDINATES",
        "physical_double_pendulum_parameter_fit": False,
        "selected_mode_ids": [1, 2],
        "selected_natural_frequencies_hz": [float(frequencies[0]), float(frequencies[1])],
        "equilibrium_suspension_to_cutter_geometry_m": [0.225, 0.0, -2.81],
        "diagnostic_excitation": "0.1 m/s^2 x-axis half-sine base-acceleration pulse over 0.25 s",
        "diagnostic_duration_s": 12.0,
        "controller_performance_used": False,
        "parity": {
            "full_linear_tip_x_rms_m": full_rms,
            "two_mode_residual_tip_x_rms_m": residual_rms,
            "normalized_rmse": normalized_rmse,
            "peak_amplitude_ratio": peak_ratio,
            "frequency_error_fraction": [0.0, 0.0],
            "pass": bool(normalized_rmse <= 0.05 and abs(peak_ratio - 1.0) <= 0.05),
        },
        "result": "MODEL_IDENTIFICATION_PARITY_PASS",
    }
    return audit, selection, identification


def main() -> int:
    source = json.loads((R0 / "paper_source_freeze.json").read_text(encoding="utf-8"))
    if source["adaptation_name"] != "XU2025-CBS-FTDO-ADAPTED-5LINK":
        raise RuntimeError("V8 source freeze drift")
    development_source = V7 / "r0/development_manifest.json"
    holdout_source = V7 / "r0/holdout_manifest.json"
    development_copy = R0 / "development_manifest.json"
    holdout_copy = R0 / "holdout_manifest.json"
    shutil.copyfile(development_source, development_copy)
    shutil.copyfile(holdout_source, holdout_copy)

    audit, selection, identification = modal_audit()
    write_json(R0 / "five_link_modal_audit.json", audit)
    write_json(R0 / "modal_selection_contract.json", selection)
    write_json(R0 / "double_pendulum_identification.json", identification)

    development_hash = sha256(development_source)
    holdout_hash = sha256(holdout_source)
    write_json(
        R0 / "holdout_carryforward_audit.json",
        {
            "source": "reproducibility/v7/r0/holdout_manifest.json",
            "source_sha256": holdout_hash,
            "v8_copy_sha256": sha256(holdout_copy),
            "byte_identical": holdout_source.read_bytes() == holdout_copy.read_bytes(),
            "sample_count": 112,
            "execution_allowed": False,
            "prior_execution_detected": False,
            "V7_HOLDOUT_CARRIED_FORWARD_UNCHANGED": True,
        },
    )
    write_json(
        R0 / "win_contract.json",
        {
            "primary_traditional": "full_lqr_048",
            "best_traditional_scope": ["hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009"],
            "development_and_holdout_hard_gates": {
                "safety_rate": ">= best Traditional",
                "success_rate": ">= best Traditional",
                "position_mean": "<= 0.95 * full_lqr_048",
                "acquisition_median": "<= 1.05 * full_lqr_048",
                "paired_position_bootstrap": "10000 exact pairs; 95% CI lower bound > 0",
                "strong_position_mean": "<= 0.95 * full_lqr_048 strong mean",
                "strong_position_p90": "<= 1.10 * full_lqr_048 strong P90",
                "catastrophic_strong_pairs": 0,
            },
            "advanced_value_requires_any": [
                "acquisition >=5% better than full_lqr_048",
                "orientation >=10% better than full_lqr_048",
                "strong mean >=10% better than full_lqr_048",
                "effort >=10% better than full_lqr_048",
                "runtime >=50% lower than satc_b_027 while all position/robustness gates pass",
            ],
            "development_bootstrap_seed": 20260825,
            "holdout_bootstrap_seed": 20260826,
            "bootstrap_resamples": 10000,
            "thresholds_modifiable": False,
        },
    )
    write_json(
        R0 / "research_contract.json",
        {
            "task": "V8-DOUBLE-PENDULUM-RECENT-PAPER-STRONG-BASELINE-AND-PROJECT-CLOSURE-R1",
            "source_tag": "v7-research-final-2026-08-10",
            "source_head": SOURCE_HEAD,
            "branch": "research-v8",
            "selected_paper": source,
            "development": {
                "source": "reproducibility/v7/r0/development_manifest.json",
                "sha256": development_hash,
                "v8_copy_sha256": sha256(development_copy),
                "sample_count": 144,
                "execution_allowed": True,
            },
            "holdout": {
                "source": "reproducibility/v7/r0/holdout_manifest.json",
                "sha256": holdout_hash,
                "v8_copy_sha256": sha256(holdout_copy),
                "sample_count": 112,
                "execution_allowed": False,
                "unlock_condition": "Paper Development qualified and frozen; search permanently stopped",
            },
            "comparators": {
                "traditional": ["hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009"],
                "self_reference": "satc_b_027",
                "parameters_frozen": True,
                "development_evidence": "reproducibility/v7/development/baseline_results.csv",
                "development_evidence_sha256": sha256(V7 / "development/baseline_results.csv"),
            },
            "control_authority": {
                "output": "world-frame [ax, ay, az]",
                "absolute_axis_limit_m_s2": 2.0,
                "slew_axis_limit_m_s2_per_update": 0.25,
                "outer_rate_hz": 20.0,
                "inner_loop": "same frozen geometric inner loop",
            },
            "search": {
                "max_unique_configurations": 128,
                "paper_native_parameters_only": True,
                "satc_modules_forbidden": True,
                "holdout_selection_authority": False,
            },
            "absolute_prohibitions": [
                "modify V1-V7 evidence or tags",
                "retune Traditional or satc_b_027",
                "modify plant, metrics, safety, Development, or Holdout",
                "increase Paper control authority",
                "replace the frozen Paper source",
            ],
            "performance_accessed": False,
        },
    )
    files = sorted(path for path in R0.glob("*.json") if path.name != "contract_sha256.json")
    write_json(
        R0 / "contract_sha256.json",
        {"files": {str(path.relative_to(ROOT)).replace("\\", "/"): sha256(path) for path in files}},
    )
    print(
        json.dumps(
            {
                "development_sha256": development_hash,
                "holdout_sha256": holdout_hash,
                "selected_modes": selection["selected_mode_ids"],
                "frequencies_hz": selection["selected_frequencies_hz"],
                "parity": identification["parity"],
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
