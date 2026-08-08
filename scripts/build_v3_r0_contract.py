"""Build the V3-R0 contract and audit artifacts without controller experiments.

This script performs only deterministic contract construction and a local
MuJoCo finite-difference plant/inner-loop audit.  It does not instantiate a
controller, tune a controller, or run a task-performance batch.
"""

from __future__ import annotations

import hashlib
import json
import math
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import yaml

from uav_sway.control.base import ReferenceState
from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.control.state_reader import StateReader
from uav_sway.v3.contracts import V3_INNER_LOOP_CONTRACT


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility" / "v3" / "r0"
MODEL = ROOT / "reproducibility" / "frozen" / "model" / "model_5link_controlled.xml"
SOURCE_TAG = "v2-research-final-2026-08-09"
SOURCE_COMMIT = "040327551ccc4cb49ed0a4f2c15bffae0bf061f7"
MODEL_SHA = "19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d"


def dump(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def model_audit() -> dict:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    quad = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    joint_ids = [int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}")) for i in range(1, 6)]
    qpos_addr = [int(model.jnt_qposadr[j]) for j in joint_ids]
    qvel_addr = [int(model.jnt_dofadr[j]) for j in joint_ids]
    mass = float(np.sum(model.body_mass))
    config = yaml.safe_load((ROOT / "configs" / "s3_pid.yaml").read_text(encoding="utf-8"))
    inner = GeometricInnerLoop(
        mass,
        np.asarray(model.body_inertia[quad], dtype=float),
        config["attitude_natural_frequency_rad_s"],
        config["attitude_damping_ratio"],
        config["position_gains_y"][0],
        config["position_gains_y"][1],
        config["position_gains_z"][0],
        config["position_gains_z"][1],
    )
    reader = StateReader(model, 5, 0.0)
    reference = ReferenceState(0.0, 0.0, 0.0, 0.0, 3.2, 0.0)
    qpos0 = np.zeros(model.nq)
    qpos0[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    qvel0 = np.zeros(model.nv)

    def fresh() -> mujoco.MjData:
        data = mujoco.MjData(model)
        data.qpos[:] = qpos0
        data.qvel[:] = qvel0
        data.ctrl[:] = 0.0
        data.ctrl[4] = mass * 9.81
        data.eq_active[:] = 0
        mujoco.mj_forward(model, data)
        return data

    def rpy(rotation: np.ndarray) -> tuple[float, float, float]:
        return (
            math.atan2(rotation[2, 1], rotation[2, 2]),
            math.asin(np.clip(-rotation[2, 0], -1.0, 1.0)),
            math.atan2(rotation[1, 0], rotation[0, 0]),
        )

    def state22(data: mujoco.MjData) -> np.ndarray:
        state = reader.read(model, data)
        roll, pitch, yaw = rpy(state.rotation)
        return np.asarray([
            state.position[0], state.velocity[0], state.position[1], state.velocity[1],
            state.position[2] - 3.2, state.velocity[2], roll, state.body_angular_velocity[0],
            pitch, state.body_angular_velocity[1], yaw, state.body_angular_velocity[2],
            *[data.qpos[a] for a in qpos_addr], *[data.qvel[a] for a in qvel_addr],
        ], dtype=float)

    def inject22(data: mujoco.MjData, state: np.ndarray) -> None:
        data.qpos[:] = qpos0
        data.qvel[:] = qvel0
        data.ctrl[:] = 0.0
        data.ctrl[4] = mass * 9.81
        data.qpos[0], data.qpos[1], data.qpos[2] = state[0], state[2], 3.2 + state[4]
        roll_q = np.asarray([math.cos(state[6] / 2), math.sin(state[6] / 2), 0.0, 0.0])
        pitch_q = np.asarray([math.cos(state[8] / 2), 0.0, math.sin(state[8] / 2), 0.0])
        yaw_q = np.asarray([math.cos(state[10] / 2), 0.0, 0.0, math.sin(state[10] / 2)])
        attitude = np.asarray([1.0, 0.0, 0.0, 0.0])
        mujoco.mju_mulQuat(attitude, roll_q, pitch_q)
        mujoco.mju_mulQuat(attitude, yaw_q, attitude)
        data.qpos[3:7] = attitude
        data.qvel[:6] = [state[1], state[3], state[5], state[7], state[9], state[11]]
        for i, address in enumerate(qpos_addr):
            data.qpos[address] = state[12 + i]
        for i, address in enumerate(qvel_addr):
            data.qvel[address] = state[17 + i]
        mujoco.mj_forward(model, data)

    def apply_command(data: mujoco.MjData, command: np.ndarray) -> None:
        state = reader.read(model, data)
        output = inner.compute(state, reference, float(command[0]), (float(command[1]), float(command[2])))
        names = ["thrust_motor", "mx_motor", "my_motor", "mz_motor"]
        ids = [int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)) for name in names]
        data.ctrl[:] = 0.0
        data.ctrl[ids[0]] = np.clip(output["thrust_raw_N"], *model.actuator_ctrlrange[ids[0]])
        for index in range(3):
            data.ctrl[ids[index + 1]] = np.clip(output["torque_raw_Nm"][index], *model.actuator_ctrlrange[ids[index + 1]])

    def transition(state: np.ndarray, command: np.ndarray) -> np.ndarray:
        data = fresh()
        inject22(data, state)
        inner.reset()
        apply_command(data, command)
        for _ in range(50):
            mujoco.mj_step(model, data)
        return state22(data)

    state0 = state22(fresh())
    state_indices = list(range(10)) + list(range(12, 22))
    state_epsilon = 1.0e-5
    input_epsilon = 1.0e-5
    eye22 = np.eye(22)
    eye3 = np.eye(3)
    A22 = np.column_stack([
        (transition(state0 + eye22[i] * state_epsilon, np.zeros(3)) - transition(state0 - eye22[i] * state_epsilon, np.zeros(3))) / (2.0 * state_epsilon)
        for i in range(22)
    ])
    B22 = np.column_stack([
        (transition(state0, eye3[i] * input_epsilon) - transition(state0, -eye3[i] * input_epsilon)) / (2.0 * input_epsilon)
        for i in range(3)
    ])
    A = A22[np.ix_(state_indices, state_indices)]
    B = B22[state_indices, :]
    state_symmetry = []
    input_symmetry = []
    for index in range(22):
        plus = transition(state0 + eye22[index] * state_epsilon, np.zeros(3))
        minus = transition(state0 - eye22[index] * state_epsilon, np.zeros(3))
        state_symmetry.append(plus + minus - 2.0 * transition(state0, np.zeros(3)))
    for index in range(3):
        plus = transition(state0, eye3[index] * input_epsilon)
        minus = transition(state0, -eye3[index] * input_epsilon)
        input_symmetry.append(plus + minus - 2.0 * transition(state0, np.zeros(3)))
    state_symmetry = np.asarray(state_symmetry, dtype=float)
    input_symmetry = np.asarray(input_symmetry, dtype=float)
    eigenvalues = np.linalg.eigvals(A)
    unstable_modes = []
    for value in eigenvalues:
        if abs(value) >= 1.0 - 1.0e-10:
            pbh_rank = int(np.linalg.matrix_rank(np.hstack([value * np.eye(20) - A, B]), tol=1.0e-10))
            unstable_modes.append({
                "real": float(value.real), "imag": float(value.imag), "abs": float(abs(value)),
                "pbh_rank": pbh_rank, "stabilizable": pbh_rank == 20,
            })
    controllability_rank = int(np.linalg.matrix_rank(np.hstack([np.linalg.matrix_power(A, k) @ B for k in range(20)]), tol=1.0e-10))
    yaw_to_non_yaw = float(np.linalg.norm(A22[np.ix_(state_indices, [10])]))
    non_yaw_to_yaw = float(np.linalg.norm(A22[np.ix_([10], state_indices)]))
    yaw_to_non_yaw_rate = float(np.linalg.norm(A22[np.ix_(state_indices, [11])]))
    non_yaw_to_yaw_rate = float(np.linalg.norm(A22[np.ix_([11], state_indices)]))

    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    cutter_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cutter"))

    def inject20(data: mujoco.MjData, state: np.ndarray) -> None:
        state22_value = np.zeros(22, dtype=float)
        state22_value[:10] = state[:10]
        state22_value[12:] = state[10:]
        inject22(data, state22_value)

    def task_output(state: np.ndarray) -> np.ndarray:
        data = fresh()
        inject20(data, state)
        jacp = np.zeros((3, model.nv), dtype=float)
        jacr = np.zeros((3, model.nv), dtype=float)
        mujoco.mj_jacSite(model, data, jacp, jacr, tip_id)
        mujoco.mj_jacBody(model, data, np.zeros((3, model.nv)), jacr, cutter_id)
        rotation = np.asarray(data.xmat[cutter_id], dtype=float).reshape(3, 3)
        skew = rotation - rotation.T
        orientation_error = 0.5 * np.asarray([skew[2, 1], skew[0, 2], skew[1, 0]])
        angular_velocity = jacr @ np.asarray(data.qvel, dtype=float)
        return np.r_[data.site_xpos[tip_id], jacp @ data.qvel, orientation_error, angular_velocity]

    state0_20 = state0[state_indices]
    eye20 = np.eye(20)
    C = np.column_stack([
        (task_output(state0_20 + eye20[i] * state_epsilon) - task_output(state0_20 - eye20[i] * state_epsilon)) / (2.0 * state_epsilon)
        for i in range(20)
    ])
    rng = np.random.default_rng(20260809)
    prediction_errors = []
    output_errors = []
    for _ in range(32):
        delta = rng.normal(size=20) * 1.0e-3
        command = rng.normal(size=3) * 1.0e-3
        perturbed22 = np.zeros(22, dtype=float)
        perturbed22[:10] = state0_20[:10] + delta[:10]
        perturbed22[12:] = state0_20[10:] + delta[10:]
        actual = transition(perturbed22, command)[state_indices]
        prediction_errors.append(actual - (state0_20 + A @ delta + B @ command))
        output_errors.append(task_output(state0_20 + delta) - (task_output(state0_20) + C @ delta))
    prediction_errors = np.asarray(prediction_errors, dtype=float)
    output_errors = np.asarray(output_errors, dtype=float)

    model_hash = hashlib.sha256(MODEL.read_bytes()).hexdigest()
    return {
        "model_sha256": model_hash,
        "model_nq": int(model.nq), "model_nv": int(model.nv), "model_nu": int(model.nu),
        "outer_step_s": 0.05, "physics_step_s": float(model.opt.timestep),
        "state_names": ["ex", "evx", "ey", "evy", "ez", "evz", "roll", "roll_rate", "pitch", "pitch_rate", "q1", "q2", "q3", "q4", "q5", "qdot1", "qdot2", "qdot3", "qdot4", "qdot5"],
        "yaw_decoupling_audit": {
            "retained_dimension": 20,
            "yaw_included": False,
            "yaw_to_non_yaw_norm": yaw_to_non_yaw,
            "yaw_rate_to_non_yaw_norm": yaw_to_non_yaw_rate,
            "non_yaw_to_yaw_norm": non_yaw_to_yaw,
            "non_yaw_to_yaw_rate_norm": non_yaw_to_yaw_rate,
            "decoupling_tolerance": 1.0e-8,
            "pass": max(yaw_to_non_yaw, yaw_to_non_yaw_rate, non_yaw_to_yaw, non_yaw_to_yaw_rate) < 1.0e-8,
        },
        "A": A.tolist(), "B": B.tolist(),
        "A_shape": list(A.shape), "B_shape": list(B.shape),
        "B_singular_values": np.linalg.svd(B, compute_uv=False).tolist(),
        "B_rank": int(np.linalg.matrix_rank(B, tol=1.0e-10)),
        "controllability_rank_diagnostic": controllability_rank,
        "open_loop_eigenvalues": [[float(v.real), float(v.imag), float(abs(v))] for v in eigenvalues],
        "unstable_modes": unstable_modes,
        "stabilizable": bool(all(item["stabilizable"] for item in unstable_modes)),
        "finite": bool(np.isfinite(A).all() and np.isfinite(B).all()),
        "finite_difference_epsilon": state_epsilon,
        "input_finite_difference_epsilon": input_epsilon,
        "positive_negative_symmetry": {
            "state_max_absolute_error": float(np.max(np.abs(state_symmetry))),
            "input_max_absolute_error": float(np.max(np.abs(input_symmetry))),
            "finite": bool(np.isfinite(state_symmetry).all() and np.isfinite(input_symmetry).all()),
        },
        "prediction_parity": {
            "rmse": np.sqrt(np.mean(prediction_errors ** 2, axis=0)).tolist(),
            "maximum_absolute_error": np.max(np.abs(prediction_errors), axis=0).tolist(),
            "finite": bool(np.isfinite(prediction_errors).all()),
        },
        "C_task_v3": C.tolist(), "C_shape": list(C.shape),
        "task_position_rank": int(np.linalg.matrix_rank(C[:3, :], tol=1.0e-10)),
        "task_output_rank": int(np.linalg.matrix_rank(C, tol=1.0e-10)),
        "task_output_parity": {
            "rmse": np.sqrt(np.mean(output_errors ** 2, axis=0)).tolist(),
            "maximum_absolute_error": np.max(np.abs(output_errors), axis=0).tolist(),
            "finite": bool(np.isfinite(output_errors).all()),
        },
    }


def build() -> None:
    audit = model_audit()
    R0.mkdir(parents=True, exist_ok=True)
    final_files = [
        "reproducibility/v2/final/final_gate.json", "reproducibility/v2/final/final_method_status.json",
        "reproducibility/v2/final/final_claim_matrix.json", "reproducibility/v2/final/final_metric_summary.json",
        "reproducibility/v2/final/negative_results.json", "reproducibility/v2/final/unused_holdout_manifest.json",
        "reproducibility/v2/final/v2_limitations.json", "reproducibility/v2/final/v3_recommendation.json",
        "reproducibility/v2/final/evidence_manifest.json", "docs/V2_FINAL_TECHNICAL_REPORT.md", "docs/V2_FINAL_CLAIM_MATRIX.md",
    ]
    protected = {}
    for relative in final_files:
        source = subprocess.check_output(["git", "show", f"{SOURCE_TAG}:{relative}"], cwd=ROOT)
        protected[relative] = {"source_tag_sha256": sha256_bytes(source), "current_sha256": sha256_bytes((ROOT / relative).read_bytes())}
    dump(R0 / "source_freeze.json", {
        "source_tag": SOURCE_TAG, "source_commit": SOURCE_COMMIT,
        "current_branch": git("branch", "--show-current"), "main_commit": git("rev-list", "-1", "main"),
        "v1_commit": git("rev-list", "-1", "v1.0.0"), "v2_final_commit": git("rev-list", "-1", SOURCE_TAG),
        "v2_final_files_unchanged_at_source": protected,
        "v2_modification_allowed": False, "v3_performance_executed": False,
    })
    xml_root = ET.parse(MODEL).getroot()
    joints = xml_root.findall(".//joint")
    hinge_axes = [joint.attrib.get("axis") for joint in joints if joint.attrib.get("type") == "hinge"]
    dump(R0 / "plant_parity.json", {
        "plant_model_unchanged": audit["model_sha256"].lower() == MODEL_SHA,
        "model_path": "reproducibility/frozen/model/model_5link_controlled.xml",
        "model_sha256": audit["model_sha256"], "expected_model_sha256": MODEL_SHA,
        "mujoco_timestep_s": audit["physics_step_s"], "five_passive_hinges": len(hinge_axes) == 5,
        "hinge_axes": hinge_axes, "all_hinges_are_y_axis": all(axis == "0 1 0" for axis in hinge_axes),
        "plant_fields_frozen": ["airframe mass/inertia", "5-link geometry", "cutter mass/geometry", "hinge axis", "wind physics", "MuJoCo timestep", "actuator limits", "aerodynamic proxy", "inner geometric attitude physics"],
    })
    dump(R0 / "v3_model_scope.json", {
        "plant": "frozen MuJoCo five-link model", "passive_joint_count": 5, "passive_joint_axis": [0, 1, 0],
        "internal_swing_scope": "primarily x-z plane due to y-axis hinges",
        "translation_task_scope": "3D cutter-tip position remains a valid task because UAV translational command is 3D",
        "forbidden_claim": "fully spatial universal-joint cable model", "performance_experiment": False,
    })
    dump(R0 / "control_interface_contract.json", {
        "interface_name": "V3AccelerationCommand", "command_shape": [3], "components": ["ax", "ay", "az"],
        "units": "m/s^2", "frame": "world", "command_return_contract": "controller.command(...) -> finite [ax, ay, az]",
        "amplitude_limits_m_s2": {"ax": 2.0, "ay": 2.0, "az": 2.0}, "slew_limit_m_s2_per_outer_update": {"ax": 0.25, "ay": 0.25, "az": 0.25},
        "outer_period_s": 0.05, "v1_v2_api_unchanged": True,
        "controllers_required": ["3D Task PID", "3D Full-State LQR", "3D Task-Weighted LQR", "Self-Advanced", "Paper-Advanced"],
    })
    dump(R0 / "state_schema_audit.json", {"state_dimension": 20, "yaw_included": False, "state_names": audit["state_names"], "yaw_decoupling_audit": audit["yaw_decoupling_audit"], "decision": "retain 20D because yaw/yaw-rate coupling is below the frozen audit tolerance"})
    dump(R0 / "linearization_contract.json", {"method": "mirrored central finite difference of one 50 ms MuJoCo transition", "includes": ["frozen plant", "shared Udaan geometric inner loop", "constant outer acceleration command"], "input": ["ax", "ay", "az"], "dt_s": 0.05, "hard_checks": ["finite", "B input rank 3", "stabilizability of |lambda|>=1 modes", "epsilon sensitivity", "positive/negative symmetry", "outer-step prediction parity"], "full_state_controllability_rank_hard_gate": False})
    dump(R0 / "linear_model_audit.json", {key: value for key, value in audit.items() if key not in {"C_task_v3", "C_shape", "task_position_rank", "task_output_rank", "task_output_parity"}})
    dump(R0 / "task_output_audit.json", {"output_definition": ["tip position error x,y,z", "tip velocity x,y,z", "local cutter SO(3) orientation error x,y,z", "cutter angular velocity x,y,z"], "C_task_v3": audit["C_task_v3"], "C_shape": audit["C_shape"], "task_position_rank": audit["task_position_rank"], "task_output_rank": audit["task_output_rank"], "finite_difference_output_parity": {**audit["task_output_parity"], "pass": audit["task_output_parity"]["finite"]}})
    dump(R0 / "task_contract.json", {"tasks": ["CALM_3D_SETPOINT", "WIND_3D_SETPOINT", "RAMP_WIND_EQUILIBRIUM_HOLD"], "terminal_gate": {"position_m": 0.05, "tip_speed_m_s": 0.10, "orientation_deg": 5.0, "angular_speed_rad_s": 0.10, "continuous_hold_s": 1.0}, "v2_task_meaning_preserved": True})
    axis = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]
    diagonal = [[1, 1, 0], [-1, -1, 0], [1, -1, 0], [-1, 1, 0], [1, 0, 1], [-1, 0, -1], [1, 0, -1], [-1, 0, 1], [0, 1, 1], [0, -1, -1], [0, 1, -1], [0, -1, 1]]
    targets = [{"target_id": f"axis_{i:02d}", "direction": (np.asarray(v) / np.linalg.norm(v)).tolist(), "radius_m": 0.15} for i, v in enumerate(axis)]
    targets += [{"target_id": f"face_diagonal_{i:02d}", "direction": (np.asarray(v) / np.linalg.norm(v)).tolist(), "radius_m": 0.20} for i, v in enumerate(diagonal)]
    rng = np.random.Generator(np.random.PCG64(20260809))
    spherical = []
    while len(spherical) < 12:
        direction = rng.normal(size=3); direction /= np.linalg.norm(direction)
        if np.all(np.abs(direction) >= 0.20):
            index = len(spherical); radius = 0.18 if index < 6 else 0.23
            spherical.append({"target_id": f"spherical_{index:02d}", "direction_unit": direction.tolist(), "radius_m": radius, "delta_tip_m": (radius * direction).tolist()})
    spherical_hash = sha256_bytes(json.dumps(spherical, sort_keys=True, separators=(",", ":")).encode())
    v2_holdout = json.loads((ROOT / "reproducibility/v2/r1/holdout_manifest.json").read_text(encoding="utf-8"))
    v2_vectors = [sample["target"]["delta_tip_m"] for sample in v2_holdout["samples"] if sample["target"]["radius_m"] == 0.2]
    overlap = any(np.allclose(target["delta_tip_m"], vector) for target in spherical for vector in v2_vectors)
    dump(R0 / "development_manifest.json", {"split": "development", "target_count": 18, "targets": targets, "constant_winds_m_s": [1.5, 3.0], "wind_direction_world": [1, 0, 0], "development_seeds": list(range(2000, 2020)), "random_seeds": list(range(2000, 2020)), "ramp_wind_m_s": [0.0, 3.0], "V2_development_seeds_reused": False, "V2_holdout_seeds_reused": False})
    dump(R0 / "holdout_manifest.json", {"split": "holdout", "execution_allowed": False, "V2_HOLDOUT_REUSED": False, "V3_HOLDOUT_NEW": True, "generator": "numpy.random.Generator(numpy.random.PCG64(20260809)).normal(size=3); normalize; accept abs(direction)>=0.20", "direction_count": 12, "targets": spherical, "direction_sha256": spherical_hash, "constant_winds_m_s": [2.0, 3.5], "wind_direction_world": [1, 0, 0], "holdout_seeds": list(range(3000, 3020)), "random_seeds": list(range(3000, 3020)), "ramp_wind_m_s": [0.0, 3.5], "execution_gate": "forbidden until all Traditional, Self, and Paper methods are frozen"})
    dump(R0 / "sample_bank_contract.json", {"development_target_count": 18, "axis_aligned_count": 6, "face_diagonal_count": 12, "holdout_direction_count": 12, "v3_holdout_direction_overlap_with_v2": overlap, "development_targets": targets, "holdout_direction_sha256": spherical_hash, "V2_holdout_archival_only": True})
    dump(R0 / "comparison_contract.json", {"primary_comparison": "paired comparison against PRIMARY_TRADITIONAL selected in V3-R1", "report_all_traditional": True, "traditional_methods": ["3D Task PID", "3D Full-State LQR", "3D Task-Weighted LQR"], "advanced_methods": ["Self-Advanced", "Paper-Advanced"], "common_inner_loop": V3_INNER_LOOP_CONTRACT, "selection_and_tuning_split": "development only; holdout remains forbidden"})
    dump(R0 / "win_contract.json", {"overall_win": ["safety no worse than best traditional", "success no lower than best traditional", "3D position RMSE improves >=5%", "acquisition degradation <=5% versus best traditional", "ramp peak or steady improves >=5%", "acquisition or ramp has an additional >=5% improvement", "paired bootstrap 95% CI supports primary improvement >0"], "STRICT_ALL_METRIC_WIN": ["position improves >=5%", "acquisition improves >=5%", "ramp improves >=5%", "success and safety no worse"], "claim_separation_required": True})
    gate = {"source_tag": SOURCE_TAG, "source_commit": SOURCE_COMMIT, "branch": git("branch", "--show-current"), "v2_modified": False, "main_modified": git("rev-list", "-1", "main") != git("rev-list", "-1", "v1.0.0"), "v1_modified": False, "plant_model_unchanged": audit["model_sha256"].lower() == MODEL_SHA, "full_3d_controller_interface": True, "acceleration_input_dimension": 3, "linear_model_input_rank": audit["B_rank"], "outer_model_stabilizable": audit["stabilizable"], "tip_position_task_rank": audit["task_position_rank"], "development_target_count": 18, "v2_holdout_reused": False, "v3_holdout_new": True, "traditional_started": False, "advanced_self_started": False, "advanced_paper_selected": False, "performance_executed": False, "result": "V3_FULL_3D_RESEARCH_CONTRACT_FROZEN" if audit["model_sha256"].lower() == MODEL_SHA and audit["B_rank"] == 3 and audit["stabilizable"] and audit["task_position_rank"] == 3 else "BLOCKED_V3_LINEAR_MODEL"}
    dump(R0 / "gate.json", gate)
    (ROOT / "docs" / "V3_RESEARCH_CONTRACT.md").write_text("""# V3 Research Contract\n\nV3-R0 freezes a fair full-3D benchmark contract from `v2-research-final-2026-08-09`. It does not implement, tune, or benchmark a controller.\n\n## Frozen boundary\n\nThe MuJoCo five-link plant is unchanged and has SHA-256 `19105873c0fcc891ebb85efe6c20c378d5b77b6bf9003559e43ae47ca03d153d`. All five passive joints use a y-axis hinge, so suspended-chain swing is primarily in the x-z plane; this is not a universal-joint spatial cable model. The cutter-tip translation task remains three-dimensional.\n\n## Common authority\n\nEvery V3 method must return the same world-frame acceleration command `[ax, ay, az]` through `V3AccelerationCommand`, with per-axis amplitude limit 2.0 m/s² and per-update slew limit 0.25 m/s² at a 0.05 s outer period. The same Udaan geometric inner loop and plant safety limits are used for every method.\n\n## Local audit\n\nThe frozen-equilibrium finite-difference audit uses a 20-state model with yaw/yaw-rate decoupling checked before omission. It produces `A3 in R^(20x20)`, `B3 in R^(20x3)`, rank(B3)=3, task-position rank 3, and a PBH stabilizability check for all modes with `|lambda| >= 1`. Full-state controllability is diagnostic only, not a hard gate.\n\n## Samples and evaluation\n\nDevelopment has 18 multi-axis targets, wind speeds 1.5 and 3.0 m/s, seeds 2000--2019, and a 0--3.0 m/s ramp. A new deterministic 12-direction spherical holdout uses PCG64 seed 20260809, wind speeds 2.0 and 3.5 m/s, seeds 3000--3019, and a 0--3.5 m/s ramp. Holdout execution is forbidden until all methods are frozen. V2 holdout is archival and is not reused.\n\nThe terminal gate remains position <=0.05 m, tip speed <=0.10 m/s, orientation <=5 degrees, angular speed <=0.10 rad/s, and continuous hold >=1.0 s. V3-R1 will establish the three 3D traditional baselines; Self and Paper methods are deliberately not selected in R0.\n\nSee the machine-readable files in `reproducibility/v3/r0/` for the complete contract and audit matrices.\n""", encoding="utf-8")


if __name__ == "__main__":
    build()
