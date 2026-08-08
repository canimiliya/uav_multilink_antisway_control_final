"""V3 task metric alignment and LQR matrix helpers."""

from __future__ import annotations

import json
import math
from pathlib import Path

import mujoco
import numpy as np
from scipy.linalg import solve_discrete_are


def load_r0_linear_matrices(root: str | Path) -> tuple[np.ndarray, np.ndarray]:
    root = Path(root)
    payload = json.loads((root / "reproducibility/v3/r0/linear_model_audit.json").read_text(encoding="utf-8"))
    return np.asarray(payload["A"], dtype=float).reshape(20, 20), np.asarray(payload["B"], dtype=float).reshape(20, 3)


def build_full_lqr_q(parameters: dict) -> np.ndarray:
    q = np.zeros(20, dtype=float)
    q[[0, 2, 4]] = float(parameters["q_position"])
    q[[1, 3, 5]] = float(parameters["q_velocity"])
    q[[6, 8]] = float(parameters["q_attitude"])
    q[[7, 9]] = 0.25 * float(parameters["q_attitude"])
    q[10:15] = float(parameters["q_joint_angle"])
    q[15:20] = float(parameters["q_joint_velocity"])
    return np.diag(q)


def solve_v3_lqr(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray) -> dict:
    a = np.asarray(a, dtype=float).reshape(20, 20)
    b = np.asarray(b, dtype=float).reshape(20, 3)
    q = np.asarray(q, dtype=float).reshape(20, 20)
    r = np.asarray(r, dtype=float).reshape(3, 3)
    p = solve_discrete_are(a, b, q, r)
    p = 0.5 * (p + p.T)
    k = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
    acl = a - b @ k
    eig = np.linalg.eigvals(acl)
    residual = p - (a.T @ p @ a - a.T @ p @ b @ np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a) + q)
    result = {
        "P": p, "K": k, "Acl": acl, "eigenvalues": eig,
        "spectral_radius": float(np.max(np.abs(eig))),
        "dare_residual_norm": float(np.linalg.norm(residual, ord="fro")),
        "p_symmetry_error": float(np.max(np.abs(p - p.T))),
        "p_min_eigenvalue": float(np.min(np.linalg.eigvalsh(p))),
    }
    if not np.isfinite(k).all() or result["p_min_eigenvalue"] <= 0.0 or result["spectral_radius"] >= 1.0:
        raise ValueError("V3 LQR candidate failed the algebra gate")
    return result


def _quat_from_roll_pitch(roll: float, pitch: float) -> np.ndarray:
    roll_q = np.asarray([math.cos(roll / 2), math.sin(roll / 2), 0.0, 0.0])
    pitch_q = np.asarray([math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0])
    result = np.asarray([1.0, 0.0, 0.0, 0.0])
    mujoco.mju_mulQuat(result, roll_q, pitch_q)
    return result


def build_task_metric_alignment(root: str | Path, epsilon: float = 1.0e-5) -> dict:
    """Identify C_pos/C_vel/C_dir/C_omega_perp on the frozen 20D state."""

    root = Path(root)
    model_path = root / "reproducibility/frozen/model/model_5link_controlled.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    quad_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    cutter_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cutter"))
    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    joint_ids = [int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint_{i}")) for i in range(1, 6)]
    qpos_addr = [int(model.jnt_qposadr[j]) for j in joint_ids]
    qvel_addr = [int(model.jnt_dofadr[j]) for j in joint_ids]
    qpos0 = np.zeros(model.nq); qpos0[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    qvel0 = np.zeros(model.nv)

    def fresh() -> mujoco.MjData:
        value = mujoco.MjData(model)
        value.qpos[:] = qpos0; value.qvel[:] = qvel0; value.ctrl[:] = 0.0; value.eq_active[:] = 0
        mujoco.mj_forward(model, value)
        return value

    def inject(value, state: np.ndarray) -> None:
        value.qpos[:] = qpos0; value.qvel[:] = qvel0
        value.qpos[0:3] = [state[0], state[2], 3.2 + state[4]]
        value.qpos[3:7] = _quat_from_roll_pitch(float(state[6]), float(state[8]))
        value.qvel[:6] = [state[1], state[3], state[5], state[7], state[9], 0.0]
        for index, address in enumerate(qpos_addr): value.qpos[address] = state[10 + index]
        for index, address in enumerate(qvel_addr): value.qvel[address] = state[15 + index]
        mujoco.mj_forward(model, value)

    def output(state: np.ndarray) -> np.ndarray:
        value = fresh(); inject(value, state)
        jacp = np.zeros((3, model.nv)); jacr = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, value, jacp, jacr, tip_id)
        body_jacr = np.zeros((3, model.nv)); body_jacp = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, value, body_jacp, body_jacr, cutter_id)
        rotation = np.asarray(value.xmat[cutter_id], dtype=float).reshape(3, 3)
        axis = rotation @ np.asarray([1.0, 0.0, 0.0])
        axis0 = np.asarray([1.0, 0.0, 0.0])
        position = np.asarray(value.site_xpos[tip_id], dtype=float)
        velocity = jacp @ value.qvel
        angular = body_jacr @ value.qvel
        omega_perp = angular - axis * float(axis @ angular)
        return np.r_[position, velocity, axis - axis0, omega_perp]

    zero = np.zeros(20, dtype=float)
    eye = np.eye(20)
    matrix = np.column_stack([(output(zero + eye[i] * epsilon) - output(zero - eye[i] * epsilon)) / (2.0 * epsilon) for i in range(20)])
    blocks = {"C_pos": matrix[0:3], "C_vel": matrix[3:6], "C_dir": matrix[6:9], "C_omega_perp": matrix[9:12]}
    return {"epsilon": epsilon, "C_pos": blocks["C_pos"].tolist(), "C_vel": blocks["C_vel"].tolist(), "C_dir": blocks["C_dir"].tolist(), "C_omega_perp": blocks["C_omega_perp"].tolist(), "finite": bool(np.isfinite(matrix).all()), "ranks": {name: int(np.linalg.matrix_rank(value, tol=1.0e-10)) for name, value in blocks.items()}, "equilibrium_output": output(zero).tolist()}


def build_task_lqr_q(a: np.ndarray, b: np.ndarray, metric: dict, parameters: dict, q_full_selected: np.ndarray) -> tuple[np.ndarray, np.ndarray, dict]:
    c_pos = np.asarray(metric["C_pos"], dtype=float).reshape(3, 20)
    c_vel = np.asarray(metric["C_vel"], dtype=float).reshape(3, 20)
    c_dir = np.asarray(metric["C_dir"], dtype=float).reshape(3, 20)
    c_omega = np.asarray(metric["C_omega_perp"], dtype=float).reshape(3, 20)
    wp, wt, r_value = float(parameters["w_p"]), float(parameters["w_theta"]), float(parameters["r"])
    q = 0.05 * np.asarray(q_full_selected, dtype=float) + wp * c_pos.T @ c_pos + 0.25 * wp * c_vel.T @ c_vel + wt * c_dir.T @ c_dir + 0.25 * wt * c_omega.T @ c_omega
    r = r_value * np.eye(3)
    return q, r, {"C_pos": c_pos.tolist(), "C_vel": c_vel.tolist(), "C_dir": c_dir.tolist(), "C_omega_perp": c_omega.tolist()}
