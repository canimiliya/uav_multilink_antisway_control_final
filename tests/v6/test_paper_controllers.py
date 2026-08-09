import json
from pathlib import Path

import numpy as np

from uav_sway.task_space.state import CutterTaskState
from uav_sway.v3.metrics import load_r0_linear_matrices
from uav_sway.v3.observation import V3Observation, V3Reference
from uav_sway.v6.paper_controllers import SEP2026PassivityNMPC, Yu2026FiniteTimeCFO, signed_power


ROOT = Path(__file__).resolve().parents[2]


def protocol():
    return json.loads((ROOT / "reproducibility/v6/papers/development_protocol.json").read_text(encoding="utf-8"))


def observation(error=(0.1, -0.05, 0.03), velocity=(0.02, -0.01, 0.0)):
    task = CutterTaskState(np.asarray(error), np.asarray(velocity), np.zeros(3), np.asarray([1.0, 0.0, 0.0]), np.eye(3))
    state = np.zeros(20)
    state[[0, 2, 4]] = error
    state[[1, 3, 5]] = velocity
    return V3Observation(state, np.zeros(3), np.zeros(3), task)


def reference():
    return V3Reference(np.zeros(3), np.zeros(3), np.zeros(3), 0.0)


def task_matrix():
    data = json.loads((ROOT / "reproducibility/v3/r1/task_metric_alignment_audit.json").read_text(encoding="utf-8"))
    return np.vstack([data[name] for name in ("C_pos", "C_vel", "C_dir", "C_omega_perp")])


def test_signed_power_is_odd_and_finite():
    value = signed_power(np.asarray([-4.0, 0.0, 4.0]), 0.5)
    assert np.allclose(value, [-2.0, 0.0, 2.0])


def test_yu_controller_is_causal_and_respects_common_limiter():
    parameters = protocol()["papers"]["YU2026"]["stage_a"][0]
    controller = Yu2026FiniteTimeCFO(parameters)
    assert not {"wind", "future_wind", "true_wind"} & set(parameters)
    previous = np.zeros(3)
    for _ in range(8):
        command = controller.command(observation(), reference())
        assert np.max(np.abs(command)) <= 2.0 + 1e-12
        assert np.max(np.abs(command - previous)) <= 0.25 + 1e-12
        previous = command
    assert np.isfinite(controller.diagnostics.disturbance_hat).all()


def test_sep_controller_solves_and_applies_passivity_projection():
    parameters = protocol()["papers"]["SEP2026"]["stage_a"][0]
    a, b = load_r0_linear_matrices(ROOT)
    controller = SEP2026PassivityNMPC(a, b, task_matrix(), parameters)
    command = controller.command(observation(), reference())
    assert np.isfinite(command).all()
    assert np.max(np.abs(command)) <= 0.25 + 1e-12
    assert controller.diagnostics.qp_status in {"solved", "solved inaccurate"}
    assert 0.0 <= controller.diagnostics.passivity_projection_fraction <= 1.0


def test_reset_clears_both_controller_states():
    data = protocol()
    yu = Yu2026FiniteTimeCFO(data["papers"]["YU2026"]["stage_a"][0])
    yu.command(observation(), reference())
    yu.reset()
    assert np.allclose(yu.z0, 0.0)
    a, b = load_r0_linear_matrices(ROOT)
    sep = SEP2026PassivityNMPC(a, b, task_matrix(), data["papers"]["SEP2026"]["stage_a"][0])
    sep.command(observation(), reference())
    sep.reset()
    assert np.allclose(sep.limiter.previous, 0.0)
