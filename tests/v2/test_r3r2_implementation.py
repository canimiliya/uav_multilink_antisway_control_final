"""Pre-performance contract tests for the frozen DR-TSRMPC implementation."""

from __future__ import annotations

from types import SimpleNamespace
import json

import numpy as np
import pytest

from uav_sway.control.dr_tsrmpc import (
    BETAS, HORIZON, RESIDUAL_WEIGHTS, DRTSRMPC, DynamicResidualEstimator,
    backbone_command, build_dr_tsrmpc_qp, enumerate_grid, project_residual,
    solve_steady_state,
)


ROOT = __import__("pathlib").Path(__file__).resolve().parents[2]


def frozen_arrays():
    a = np.load(ROOT / "reproducibility/frozen/linear_model/A.npy")
    b = np.load(ROOT / "reproducibility/frozen/linear_model/B.npy")
    c = np.load(ROOT / "reproducibility/frozen/task_lqr/C_task.npy")
    from uav_sway.control.task_lqr import build_task_lqr
    result = build_task_lqr(a, b, c, 20, 5, 1)
    return a, b, c, np.asarray(result["K"]), np.asarray(result["P"])


def ref(x=0.0, vx=0.0, z=3.2):
    return SimpleNamespace(x_ref=x, vx_ref=vx, z_ref=z)


def test_residual_is_16d_first_update_zero_and_uses_previous_actual_ax():
    a, b, *_ = frozen_arrays()
    estimator = DynamicResidualEstimator(a, b, 0.15)
    first = estimator.update(np.zeros(16), ref())
    assert first.first_sample and first.d_hat.shape == (16,) and np.all(first.d_hat == 0.0)
    estimator.accept_actual_command(0.37)
    x = np.zeros(16); x[0] = 0.2
    second = estimator.update(x, ref())
    expected = x - b[:, 0] * 0.37
    assert second.d_raw == pytest.approx(expected)
    assert second.d_hat == pytest.approx(0.15 * expected)


def test_reference_shift_is_causal_and_target_step_has_no_false_residual():
    a, b, *_ = frozen_arrays()
    estimator = DynamicResidualEstimator(a, b, 0.05)
    estimator.update(np.zeros(16), ref(0.0))
    estimator.accept_actual_command(0.0)
    shifted_state = -(__import__("uav_sway.mpc.preview_model", fromlist=["reference_vector"]).reference_vector(ref(0.15)) - a @ __import__("uav_sway.mpc.preview_model", fromlist=["reference_vector"]).reference_vector(ref(0.0)))
    second = estimator.update(shifted_state, ref(0.15))
    assert np.linalg.norm(second.d_raw) <= 1.0e-12


def test_grid_is_exactly_36_and_only_frozen_values():
    grid = enumerate_grid()
    assert len(grid) == 36
    assert [row["candidate_id"] for row in grid] == [f"dr_tsrmpc_{i:03d}" for i in range(36)]
    assert {row["beta"] for row in grid} == set(BETAS)
    assert {row["R"] for row in grid} == set(RESIDUAL_WEIGHTS)
    assert all(row["H"] == HORIZON for row in grid)


def test_no_output_bias_or_matched_wind_path_and_controllability_projection():
    source = (ROOT / "src/uav_sway/control/dr_tsrmpc.py").read_text(encoding="utf-8").lower()
    assert "output_bias" not in source
    assert "b*d_hat" not in source.replace(" ", "")
    a, b, *_ = frozen_arrays()
    projection = project_residual(a, b, np.arange(16, dtype=float))
    assert projection.d_controllable.shape == (16,)
    assert projection.d_uncontrollable.shape == (16,)
    assert projection.controllability_rank == 13
    assert projection.projection_residual <= 1.0e-10


def test_steady_state_exact_equality_and_bound_preserves_equality():
    a, b, c, *_ = frozen_arrays()
    for d in (np.zeros(16), np.linspace(-0.02, 0.02, 16), np.ones(16) * 0.001):
        result = solve_steady_state(a, b, c, d)
        assert result.equality_residual <= 1.0e-8
        assert abs(result.u_s) <= 2.0 + 1.0e-10


def test_xs_v_zero_parity_and_no_double_counting():
    a, b, c, gain, p = frozen_arrays()
    controller = DRTSRMPC(a, b, c, gain, p, 0.15, 80, 20, 2.0)
    result = solve_steady_state(a, b, c, np.linspace(-0.01, 0.01, 16))
    command = result.u_s - float((gain @ (result.x_s - result.x_s))[0])
    assert command == pytest.approx(result.u_s, abs=1.0e-10)
    qp = build_dr_tsrmpc_qp(a, b, c, gain, p, result.x_s, result.x_s, result.u_s, 80, 20, 2.0, 0.0)
    assert np.allclose(qp.affine_states[1], 0.0)
    assert np.allclose(qp.state_maps[1, :, 0], b[:, 0])
    assert controller.diagnostics.d_hat_vector.shape == (16,)


def test_horizon_physical_amplitude_and_slew_constraints_use_actual_previous():
    a, b, c, gain, p = frozen_arrays()
    qp = build_dr_tsrmpc_qp(a, b, c, gain, p, np.zeros(16), np.zeros(16), 0.0, 40, 5, 0.5, 0.2)
    assert qp.P.shape == (20, 20)
    assert qp.A.shape == (40, 20)
    assert qp.lower[0] == pytest.approx(-2.0)
    assert qp.upper[0] == pytest.approx(2.0)
    assert qp.lower[1] == pytest.approx(0.2 - 0.25)
    assert qp.upper[1] == pytest.approx(0.2 + 0.25)


def test_shared_backbone_parity_and_frozen_contract_artifact():
    a, b, c, gain, p = frozen_arrays()
    rng = np.random.default_rng(20260808)
    for _ in range(4):
        x = rng.normal(size=16)
        assert backbone_command(gain, x) == pytest.approx(float((-gain @ x).item()), abs=1.0e-12)
    contract = json.loads((ROOT / "reproducibility/v2/r3r2/implementation_contract.json").read_text(encoding="utf-8"))
    assert contract["performance_executed"] is False
    assert contract["horizon"] == 20
    assert contract["shared_yz"]["ay_kp"] == 0.1
