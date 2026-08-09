"""Contract tests for the V3 full-3D DR-TSRMPC implementation."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from uav_sway.v3.dr_tsrmpc import CausalDynamicResidual, SteadyCompensator, build_qp, controllability_basis
from uav_sway.v3.metrics import load_r0_linear_matrices
from uav_sway.v3.observation import V3Reference


ROOT = Path(__file__).resolve().parents[2]


def arrays():
    a, b = load_r0_linear_matrices(ROOT)
    metric = json.loads((ROOT / "reproducibility/v3/r1/task_metric_alignment_audit.json").read_text(encoding="utf-8"))
    c = np.vstack([metric[name] for name in ("C_pos", "C_vel", "C_dir", "C_omega_perp")])
    primary = json.loads((ROOT / "reproducibility/v3/r1r1/primary_traditional_baseline.json").read_text(encoding="utf-8"))
    k = np.asarray(primary["selected_metrics"]["parameters"]["K"])
    return a, b, c, k


def reference(position=(0.0, 0.0, 3.2)) -> V3Reference:
    position = np.asarray(position, dtype=float)
    return V3Reference(position, np.zeros(3), position + np.asarray([0.225, 0.0, -2.81]), 0.0)


def test_controllability_projection_is_frozen_20d_three_input() -> None:
    a, b, *_ = arrays()
    basis = controllability_basis(a, b)
    assert basis.shape == (20, 18)
    assert np.max(np.abs(basis.T @ basis - np.eye(18))) <= 1.0e-10


def test_residual_is_causal_uses_actual_previous_command_and_removes_target_jump() -> None:
    a, b, *_ = arrays()
    estimator = CausalDynamicResidual(a, b, beta=0.15, component_limit=0.2)
    estimator.update(np.zeros(20), reference())
    actual = np.asarray([0.2, -0.1, 0.05])
    estimator.accept(actual)
    delta = np.asarray([0.15, -0.08, 0.04])
    shifted = a @ np.zeros(20) + b @ actual
    shifted[[0, 2, 4]] -= delta
    raw, filtered, projected = estimator.update(shifted, reference(np.asarray([0.0, 0.0, 3.2]) + delta))
    assert np.linalg.norm(raw) <= 1.0e-12
    assert np.linalg.norm(filtered) <= 1.0e-12
    assert np.linalg.norm(projected) <= 1.0e-12


def test_steady_compensation_preserves_external_zero_task_target() -> None:
    a, b, c, _ = arrays()
    solver = SteadyCompensator(a, b, c)
    residual = np.linspace(-0.01, 0.01, 20)
    result = solver.solve(residual)
    assert result.state.shape == (20,)
    assert result.command.shape == (3,)
    assert result.equality_residual <= 1.0e-9
    assert np.max(np.abs(result.command)) <= 1.9 + 1.0e-12


def test_qp_has_three_axis_physical_amplitude_and_slew_constraints() -> None:
    a, b, c, k = arrays()
    steady = SteadyCompensator(a, b, c).solve(np.zeros(20))
    previous = np.asarray([0.2, -0.1, 0.05])
    qp = build_qp(a, b, c, k, np.zeros(20), steady, previous, 8, 40.0, 2.0, 1.0, 0.2, 0.2, 1.0)
    assert qp.P.shape == (24, 24)
    assert qp.A.shape == (48, 24)
    # At zero state/steady command the first physical map is identity.
    assert qp.A[0, :3] == pytest.approx([1.0, 0.0, 0.0])
    assert qp.lower[1] == pytest.approx(previous[0] - 0.25)
    assert qp.upper[1] == pytest.approx(previous[0] + 0.25)


def test_source_has_no_future_wind_holdout_or_fallback_path() -> None:
    source = (ROOT / "src/uav_sway/v3/dr_tsrmpc.py").read_text(encoding="utf-8").lower()
    assert "future_wind" not in source
    assert "holdout" not in source
    assert "fallback" not in source
