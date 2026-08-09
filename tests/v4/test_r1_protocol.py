import json
from pathlib import Path

import numpy as np

from uav_sway.v3.metrics import load_r0_linear_matrices
from uav_sway.v4.cart_ofmpc import ConstraintFeasibleSteadySolver, _box_qp


ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/v4/r0"
R1 = ROOT / "reproducibility/v4/r1"


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def test_protocol_budget_and_subset_are_frozen() -> None:
    protocol = read_json(R1 / "development_protocol.json")
    manifest = read_json(R0 / "development_manifest.json")
    assert len(protocol["search"]["stage_a"]["candidates"]) == 8
    assert len(protocol["search"]["stage_a"]["sample_ids"]) == 16
    assert len(protocol["search"]["stage_b"]["candidates"]) == 18
    assert protocol["search"]["stage_c"]["candidate_max"] == 6
    assert len({row["candidate_id"] for row in protocol["search"]["stage_a"]["candidates"] + protocol["search"]["stage_b"]["candidates"]}) == 26
    assert set(protocol["search"]["stage_a"]["sample_ids"]) <= {row["sample_id"] for row in manifest["samples"]}


def test_protocol_has_no_performance_or_holdout_execution() -> None:
    protocol = read_json(R1 / "development_protocol.json")
    assert protocol["written_before_first_new_self_development_performance"] is True
    assert protocol["new_self_performance_executed_at_freeze"] is False
    assert protocol["holdout_executed"] is False
    assert all(row["execution_allowed"] is False for row in read_json(R0 / "holdout_manifest.json")["samples"])


def test_box_qp_respects_asymmetric_reachable_bounds() -> None:
    h = np.diag([1.0, 2.0, 3.0])
    result = _box_qp(h, np.asarray([-10.0, 10.0, -1.0]), np.asarray([-0.2, -0.4, -0.6]), np.asarray([0.3, 0.5, 0.7]))
    assert np.all(result >= np.asarray([-0.2, -0.4, -0.6]) - 1.0e-12)
    assert np.all(result <= np.asarray([0.3, 0.5, 0.7]) + 1.0e-12)


def test_steady_solver_never_silently_scales_requested_input() -> None:
    a, b = load_r0_linear_matrices(ROOT)
    metric = read_json(ROOT / "reproducibility/v3/r1/task_metric_alignment_audit.json")
    c_task = np.vstack([metric[name] for name in ("C_pos", "C_vel", "C_dir", "C_omega_perp")])
    parameters = read_json(R1 / "development_protocol.json")["search"]["stage_a"]["candidates"][0]
    solver = ConstraintFeasibleSteadySolver(a, b, c_task, parameters)
    residual = np.zeros(20); residual[9] = 2.0; residual[15] = 2.0
    result = solver.solve(residual, np.asarray([1.8, -1.8, 0.0]))
    assert np.all(np.abs(result.command) <= 2.0 + 1.0e-12)
    assert np.all(np.abs(result.command - np.asarray([1.8, -1.8, 0.0])) <= 2.0 + 1.0e-12)
    assert np.linalg.norm(result.unrepresented) >= 0.0
    assert not np.shares_memory(result.command, result.requested_command)
