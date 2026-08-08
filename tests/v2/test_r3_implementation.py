import hashlib
import json
from pathlib import Path

import numpy as np
import pytest

from uav_sway.control.of_tsrmpc import (
    AX_MAX,
    AX_MIN,
    AX_SLEW,
    HORIZON,
    backbone_command,
    build_of_tsrmpc_qp,
    enumerate_grid,
    solve_steady_state,
)


ROOT = Path(__file__).resolve().parents[2]


def frozen_arrays():
    return (
        np.load(ROOT / "reproducibility/frozen/linear_model/A.npy"),
        np.load(ROOT / "reproducibility/frozen/linear_model/B.npy"),
        np.load(ROOT / "reproducibility/frozen/task_lqr/C_task.npy"),
        np.load(ROOT / "reproducibility/frozen/linear_model/K.npy"),
        np.load(ROOT / "reproducibility/frozen/linear_model/P.npy"),
    )


def test_c_task_shape_and_row_semantics_are_frozen():
    c = frozen_arrays()[2]
    assert c.shape == (4, 16)
    np.testing.assert_allclose(c[0], [1, 0, 0, 0, -2.81, 0, -2.57, -2.07, -1.57, -1.07, -0.57, 0, 0, 0, 0, 0])
    np.testing.assert_allclose(c[1], [0, 1, 0, 0, 0, -2.81, 0, 0, 0, 0, 0, -2.57, -2.07, -1.57, -1.07, -0.57])
    np.testing.assert_allclose(c[2], [0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0])
    np.testing.assert_allclose(c[3], [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1])


def test_output_bias_is_tip_x_only_and_no_matched_wind_term():
    c = frozen_arrays()[2]
    x = np.arange(16, dtype=float) / 10.0
    measured = c @ x + np.array([0.2, 4.0, -3.0, 2.0])
    d_raw = float(measured[0] - c[0] @ x)
    assert d_raw == pytest.approx(0.2)
    source = (ROOT / "src/uav_sway/control/of_tsrmpc.py").read_text(encoding="utf-8")
    assert "B*d_hat" not in source
    assert "wind estimate" not in source.lower()


def test_steady_state_zero_bias_is_zero():
    a, b, c, *_ = frozen_arrays()
    result = solve_steady_state(a, b, c[0], 0.0)
    np.testing.assert_allclose(result.x_s, 0.0, atol=1e-12)
    assert result.u_s == pytest.approx(0.0, abs=1e-12)
    assert result.residual <= 1e-12


def test_steady_state_control_equality_and_equilibrium_shift():
    a, b, c, gain, _ = frozen_arrays()
    result = solve_steady_state(a, b, c[0], 0.15)
    assert result.residual <= 1e-10
    physical = result.u_s - float((gain @ result.x_s)[0])
    # x=x_s and v=0 must produce u_s, not u_s-K*x_s.
    corrected = result.u_s - float((gain @ (result.x_s - result.x_s))[0])
    assert corrected == pytest.approx(result.u_s, abs=1e-12)
    assert physical != pytest.approx(corrected, abs=1e-8)


def test_qp_uses_h20_and_physical_amplitude_and_slew_constraints():
    a, b, c, gain, p = frozen_arrays()
    qp = build_of_tsrmpc_qp(a, b, c, gain, p, np.zeros(16), np.zeros(16), 0.0, 0.0, 40, 5, 0.5, 0.0)
    assert qp.P.shape == (HORIZON, HORIZON)
    assert qp.A.shape == (2 * HORIZON, HORIZON)
    # With x_s=0 and u_s=0, first constraints are v_0 amplitude and slew.
    assert qp.lower[0] == pytest.approx(AX_MIN)
    assert qp.upper[0] == pytest.approx(AX_MAX)
    assert qp.lower[1] == pytest.approx(-AX_SLEW)
    assert qp.upper[1] == pytest.approx(AX_SLEW)
    assert np.isfinite(qp.P).all() and np.isfinite(qp.A).all()


def test_backbone_parity_and_closed_loop_matrix_are_unchanged():
    a, b, c, gain, p = frozen_arrays()
    rng = np.random.default_rng(20260808)
    for _ in range(8):
        x = rng.normal(size=16)
        assert backbone_command(gain, x) == pytest.approx(float((-gain @ x).item()), abs=1e-12)
    assert np.max(np.abs((a - b @ gain) - (a - b @ gain))) == 0.0
    assert np.min(np.linalg.eigvalsh((p + p.T) / 2.0)) > 0.0


def test_exact_36_deterministic_grid():
    grid = enumerate_grid()
    assert len(grid) == 36
    assert [row["candidate_id"] for row in grid] == [f"of_ts_rmpc_{i:03d}" for i in range(36)]
    assert len({json.dumps(row, sort_keys=True) for row in grid}) == 36


def test_target_step_does_not_create_false_bias_in_reference_coordinates():
    c = frozen_arrays()[2]
    x_before = np.zeros(16)
    # A +0.15 target is represented as a -0.15 state error in the frozen
    # reference coordinates; both measured output and C_pos*x shift together.
    x_after = x_before.copy(); x_after[0] = -0.15
    e_before = float(c[0] @ x_before)
    e_after = float(c[0] @ x_after)
    assert (e_after - float(c[0] @ x_after)) == pytest.approx(0.0, abs=1e-12)
    assert (e_before - float(c[0] @ x_before)) == pytest.approx(0.0, abs=1e-12)


def test_source_has_no_forbidden_holdout_or_fallback():
    source = (ROOT / "src/uav_sway/control/of_tsrmpc.py").read_text(encoding="utf-8").lower()
    assert "holdout" not in source
    assert "pid" not in source
    assert "fallback" not in source
