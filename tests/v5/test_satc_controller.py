import json
from pathlib import Path

import numpy as np

from uav_sway.v3.metrics import load_r0_linear_matrices
from uav_sway.v5.satc_ofmpc import SATCOFMPC


ROOT = Path(__file__).resolve().parents[2]


def parameters():
    value = json.loads((ROOT / "reproducibility/v4/r1/near_miss.json").read_text(encoding="utf-8"))["parameters"]
    value.update({
        "shock_reference_gain": 8.0, "shock_innovation_scale": 0.10, "shock_decay": 0.88,
        "conflict_beta": 0.30, "shock_base_weight": 0.6, "conflict_gain": 0.8,
        "cancellation_gain": 0.4, "cancellation_norm_scale": 0.5,
        "robust_blend_max": 1.0, "offset_disengage_rate": 0.25, "offset_engage_rate": 0.05,
        "robust_gain_scale": 1.0, "amplitude_reserve_fraction": 0.0, "slew_reserve_fraction": 0.0,
    })
    return value


def make_controller():
    a, b = load_r0_linear_matrices(ROOT)
    metric = json.loads((ROOT / "reproducibility/v3/r1/task_metric_alignment_audit.json").read_text(encoding="utf-8"))
    c = np.vstack([metric[name] for name in ("C_pos", "C_vel", "C_dir", "C_omega_perp")])
    task = np.asarray(json.loads((ROOT / "reproducibility/v3/r1/task_lqr_freeze.json").read_text(encoding="utf-8"))["parameters"]["K"])
    full = np.asarray(json.loads((ROOT / "reproducibility/v3/r1/full_lqr_freeze.json").read_text(encoding="utf-8"))["parameters"]["K"])
    return SATCOFMPC(a, b, c, task, full, parameters())


def test_conflict_index_is_geometric_not_x_specific():
    assert SATCOFMPC._opposition(np.array([1.0, 0.0, 0.0]), np.array([-1.0, 0.0, 0.0])) == 1.0
    assert SATCOFMPC._opposition(np.array([0.0, 1.0, 0.0]), np.array([0.0, -1.0, 0.0])) == 1.0
    assert SATCOFMPC._opposition(np.array([1.0, 0.0, 0.0]), np.array([0.0, 1.0, 0.0])) == 0.0


def test_reset_preserves_frozen_authority_and_clears_transient_state():
    controller = make_controller()
    controller.shock_score = 1.0
    controller.offset_engagement = 0.0
    controller.reset()
    assert controller.shock_score == 0.0
    assert controller.offset_engagement == 1.0
    assert controller.limiter.absolute_limit_m_s2 == 2.0
    assert controller.limiter.slew_limit_m_s2_per_update == 0.25


def test_parameter_set_is_causal_and_finite():
    controller = make_controller()
    assert np.isfinite(controller.robust_gain).all()
    assert not {"wind_speed", "true_wind", "future_wind"} & set(controller.parameters)
