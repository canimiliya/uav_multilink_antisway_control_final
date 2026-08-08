"""Pre-performance contract tests for the LV2026 five-link adaptation."""

from __future__ import annotations

import numpy as np
import pytest

from uav_sway.control.lv2026_cascade import (
    PAPER_K_BETA,
    PAPER_KP_BETA,
    equivalent_swing_from_vectors,
    enumerate_grid,
    paper_middle_loop_beta_acceleration,
    paper_suspension_acceleration_correction,
    synthetic_equivalent_swing_audit,
)


def test_grid_is_exactly_nine_and_only_paper_gain_multipliers():
    grid = enumerate_grid()
    assert len(grid) == 9
    assert [row["candidate_id"] for row in grid] == [f"lv2026_{i:03d}" for i in range(9)]
    assert {row["k_beta_multiplier"] for row in grid} == {0.5, 1.0, 2.0}
    assert {row["k_p_beta_multiplier"] for row in grid} == {0.5, 1.0, 2.0}
    assert all(row["k_beta"] == pytest.approx(PAPER_K_BETA * row["k_beta_multiplier"]) for row in grid)
    assert all(row["k_p_beta"] == pytest.approx(PAPER_KP_BETA * row["k_p_beta_multiplier"]) for row in grid)


def test_equivalent_beta_sign_uses_mujo_co_z_up_convention():
    equilibrium = equivalent_swing_from_vectors([0.0, 0.0, -2.5], [0.0, 0.0, 0.0])
    positive = equivalent_swing_from_vectors([0.1, 0.0, -2.5], [0.0, 0.0, 0.0])
    negative = equivalent_swing_from_vectors([-0.1, 0.0, -2.5], [0.0, 0.0, 0.0])
    assert equilibrium.beta_eq_rad == pytest.approx(0.0)
    assert positive.beta_eq_rad > 0.0
    assert negative.beta_eq_rad < 0.0


def test_beta_rate_is_analytic_and_causal():
    state = equivalent_swing_from_vectors([0.1, 0.0, -2.5], [0.2, 0.0, 0.0])
    expected = 2.5 * 0.2 / (0.1**2 + 2.5**2)
    assert state.beta_dot_eq_rad_s == pytest.approx(expected)


def test_eq18_shape_has_stable_small_angle_sign():
    beta_accel, beta_error, shaped_rate = paper_middle_loop_beta_acceleration(0.05, 0.0, 3.2, 3.2)
    assert beta_error < 0.0
    assert shaped_rate < 0.0
    assert beta_accel < 0.0


def test_pendulum_bridge_positive_x_offset_has_positive_recovery_acceleration():
    correction, beta_accel, _, _ = paper_suspension_acceleration_correction(0.01, 0.0, 2.5, 3.2, 3.2)
    assert beta_accel < 0.0
    assert correction > 0.0


def test_synthetic_audit_passes_without_performance_data():
    audit = synthetic_equivalent_swing_audit()
    assert audit["causal"] is True
    assert audit["positive_x_sign_pass"] is True

