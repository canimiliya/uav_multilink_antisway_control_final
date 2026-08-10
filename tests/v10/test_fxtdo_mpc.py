from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from uav_sway.task_space.state import CutterTaskState
from uav_sway.v10.fxtdo_mpc import XuFxTDOMPC, _phi, build_fxtdo_mpc_qp
from uav_sway.v3.metrics import load_r0_linear_matrices
from uav_sway.v3.observation import V3Observation, V3Reference


ROOT = Path(__file__).resolve().parents[2]


def parameters() -> dict:
    return json.loads((ROOT / "reproducibility/v10/r0/search_contract.json").read_text())["candidates"][0]


def model() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    a, b = load_r0_linear_matrices(ROOT)
    metric = json.loads((ROOT / "reproducibility/v3/r1/task_metric_alignment_audit.json").read_text())
    c = np.vstack([np.asarray(metric[name]) for name in ("C_pos", "C_vel", "C_dir", "C_omega_perp")])
    return a, b, c


def observation(state=None, velocity=(0.0, 0.0, 0.0)) -> V3Observation:
    state = np.zeros(20) if state is None else np.asarray(state, dtype=float)
    task = CutterTaskState(np.zeros(3), np.zeros(3), np.zeros(3), np.asarray([1.0, 0.0, 0.0]), np.eye(3))
    return V3Observation(state, np.asarray([0.0, 0.0, 3.2]), np.asarray(velocity), task)


def reference() -> V3Reference:
    return V3Reference(np.asarray([0.0, 0.0, 3.2]), np.zeros(3), np.zeros(3), 0.0)


def test_bihomogeneous_phi_retains_all_paper_terms() -> None:
    p = parameters()
    value = np.asarray([0.3, -0.4, 0.2])
    assert np.isfinite(_phi(value, p, 1)).all()
    assert np.isfinite(_phi(value, p, 2)).all()
    assert np.linalg.norm(_phi(10.0 * value, p, 1)) > 10.0 * np.linalg.norm(_phi(value, p, 1))
    assert np.linalg.norm(_phi(10.0 * value, p, 2)) > 10.0 * np.linalg.norm(_phi(value, p, 2))


def test_qp_injects_disturbance_and_constrains_every_move() -> None:
    a, b, c = model()
    p = parameters()
    p["horizon_updates"] = 4
    zero = build_fxtdo_mpc_qp(a, b, c, np.zeros(20), np.zeros(3), np.zeros(3), p)
    disturbed = build_fxtdo_mpc_qp(a, b, c, np.zeros(20), np.zeros(3), np.asarray([0.5, -0.2, 0.1]), p)
    assert zero.P.shape == disturbed.P.shape == (12, 12)
    assert zero.A.shape == disturbed.A.shape == (24, 12)
    assert not np.allclose(zero.affine_states, disturbed.affine_states)
    assert np.max(disturbed.upper[0::2]) == 2.0 and np.min(disturbed.lower[0::2]) == -2.0
    assert np.max(disturbed.upper[1::2]) <= 0.25 and np.min(disturbed.lower[1::2]) >= -0.25


def test_controller_is_causal_finite_and_respects_common_authority() -> None:
    a, b, c = model()
    controller = XuFxTDOMPC(a, b, c, parameters())
    state = np.zeros(20)
    state[[0, 2, 4]] = [1.0, -1.0, 0.5]
    commands = []
    for index in range(20):
        commands.append(controller.command(observation(state, velocity=(0.01 * index, 0.0, 0.0)), reference()))
    assert np.isfinite(commands).all()
    assert all(np.max(np.abs(value)) <= 2.0 + 1.0e-12 for value in commands)
    assert all(np.max(np.abs(right - left)) <= 0.25 + 1.0e-12 for left, right in zip([np.zeros(3), *commands[:-1]], commands))
    assert controller.diagnostics.observer_stable is True
    assert controller.diagnostics.qp_status in {"solved", "solved inaccurate"}


def test_reset_is_deterministic_and_wrong_rate_rejected() -> None:
    a, b, c = model()
    controller = XuFxTDOMPC(a, b, c, parameters())
    first = controller.command(observation(), reference())
    controller.command(observation(velocity=(0.1, 0.0, 0.0)), reference())
    controller.reset()
    repeated = controller.command(observation(), reference())
    np.testing.assert_allclose(repeated, first, atol=0.0, rtol=0.0)
    try:
        controller.command(observation(), reference(), 0.1)
    except ValueError as error:
        assert "20 Hz" in str(error)
    else:
        raise AssertionError("wrong outer rate was accepted")
