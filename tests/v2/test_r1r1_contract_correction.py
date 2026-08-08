"""Contract tests for the V2-R1R1 corrected traditional-baseline path."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

from uav_sway.control.base import ControlState, ReferenceState
from uav_sway.control.geometric_inner_loop import GeometricInnerLoop
from uav_sway.task_space.state import CutterTaskState, CutterTaskSpaceReader
from uav_sway.task_space.v2_reference import Shared3DControlLimits
from scripts.run_v2_r1_baselines import summarize_trace, pid_grid, lqr_grid, task_lqr_grid


ROOT = Path(__file__).resolve().parents[2]
R1 = ROOT / "reproducibility/v2/r1"
R1R1 = ROOT / "reproducibility/v2/r1r1"
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"


def _state() -> ControlState:
    return ControlState(
        position=np.zeros(3), velocity=np.zeros(3), rotation=np.eye(3),
        body_angular_velocity=np.zeros(3), joint_angles=np.zeros(5),
        joint_velocities=np.zeros(5), tip_displacement=0.0,
    )


def _trace(n: int = 401) -> dict[str, np.ndarray]:
    zeros = np.zeros(n)
    return {
        "time": np.arange(n, dtype=float) * 0.005,
        "position_error_3d_m": np.full(n, 0.01), "orientation_error_deg": np.full(n, 1.0),
        "tip_speed_m_s": np.full(n, 0.01), "cutter_angular_speed_rad_s": np.full(n, 0.01),
        "ax_cmd": zeros.copy(), "ay_cmd": zeros.copy(), "az_cmd": zeros.copy(),
        "uav_z": np.full(n, 3.2), "tip_x": np.full(n, 0.225), "tip_y": zeros.copy(), "tip_z": np.full(n, 0.39),
        "target_x": np.full(n, 0.225), "target_y": zeros.copy(), "target_z": np.full(n, 0.39),
        "wind_x": zeros.copy(), "thrust": np.full(n, 100.0), "torque": np.zeros((n, 3)),
        "joint_angles": np.zeros((n, 5)), "roll": zeros.copy(), "pitch": zeros.copy(),
        "anchor_active": np.zeros(n, dtype=bool), "rotor_commands": np.zeros((n, 4)),
    }


def _sample() -> dict:
    return {"sample_id": "unit", "scenario": "CALM_3D_SETPOINT", "target_issue_time_s": 0.0, "wind": {"kind": "constant", "speed_m_s": 0.0}}


def test_cutter_state_contains_jacobian_angular_velocity() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = np.linspace(-0.2, 0.2, model.nv)
    mujoco.mj_forward(model, data)
    reader = CutterTaskSpaceReader(model)
    state = reader.read(model, data)
    jacp = np.zeros((3, model.nv)); jacr = np.zeros((3, model.nv))
    tip_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
    mujoco.mj_jacSite(model, data, jacp, jacr, tip_id)
    assert np.allclose(state.cutter_angular_velocity_world, jacr @ data.qvel)


def test_equilibrium_angular_velocity_is_zero() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL)); data = mujoco.MjData(model)
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]; data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    state = CutterTaskSpaceReader(model).read(model, data)
    assert np.allclose(state.cutter_angular_velocity_world, 0.0)


def test_angular_speed_is_a_formal_acquisition_condition() -> None:
    trace = _trace(); trace["cutter_angular_speed_rad_s"][200] = 0.100001
    result = summarize_trace(trace, _sample(), None)
    assert result["task_success"] is False


def test_acquisition_requires_one_second_continuously() -> None:
    trace = _trace(); trace["cutter_angular_speed_rad_s"][200] = 0.2
    result = summarize_trace(trace, _sample(), None)
    assert result["task_success"] is False
    trace = _trace(); result = summarize_trace(trace, _sample(), None)
    assert result["task_success"] is True


def test_shared_yz_uses_tip_state_and_target_error() -> None:
    inner = GeometricInnerLoop(10.0, np.ones(3), ay_kp=1.0, ay_kd=1.0, az_kp=1.0, az_kd=1.0, shared_limits=Shared3DControlLimits())
    task = CutterTaskState(np.zeros(3), np.zeros(3), np.zeros(3), np.array([1.0, 0.0, 0.0]), np.eye(3))
    reference = ReferenceState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    ay, az = inner.shared_yz_command(_state(), reference, task, np.array([0.0, 0.15, 0.15]))
    assert ay > 0.0 and az > 0.0


def test_shared_limits_cover_amplitude_and_slew() -> None:
    limits = Shared3DControlLimits()
    ay, az = limits.apply(9.0, -9.0)
    assert (ay, az) == (0.25, -0.25)
    ay, az = limits.apply(9.0, -9.0, ay, az)
    assert abs(ay) <= 0.5 and abs(az) <= 0.5


def test_safety_detects_y_amplitude_and_z_slew() -> None:
    trace = _trace(); trace["ay_cmd"][100] = 2.1; trace["az_cmd"][101] = 0.5
    result = summarize_trace(trace, _sample(), None)
    assert result["safe"] is False
    assert result["safety_reasons"]["ay_amplitude"] is False
    assert result["safety_reasons"]["az_slew"] is False


def test_sample_bank_hash_and_grid_sizes_are_unchanged() -> None:
    digest = hashlib.sha256((R1 / "sample_bank_contract.json").read_bytes()).hexdigest()
    parity = json.loads((R1R1 / "sample_bank_parity.json").read_text(encoding="utf-8"))
    assert digest == parity["source_sample_bank_contract_sha256"] == parity["r1r1_sample_bank_contract_sha256"]
    assert len(pid_grid()) == 18; assert len(lqr_grid()) == 64; assert len(task_lqr_grid()) == 27


def test_holdout_is_manifest_only() -> None:
    holdout = json.loads((R1 / "holdout_manifest.json").read_text(encoding="utf-8"))
    assert holdout["execution_allowed"] is False
    assert all(row["execution_allowed"] is False for row in holdout["samples"])
    runs = R1R1 / "runs"
    assert not runs.exists() or not any(path.name.startswith("holdout") for path in runs.rglob("*.csv"))
