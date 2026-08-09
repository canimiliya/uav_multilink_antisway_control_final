import json
from pathlib import Path

import numpy as np

from uav_sway.v5.jirousek2025_incremental_mpc import build_incremental_qp


ROOT = Path(__file__).resolve().parents[2]
PAPER = ROOT / "reproducibility/v5/paper"


def read(name): return json.loads((PAPER / name).read_text(encoding="utf-8"))


def test_unique_primary_source_and_protocol_are_frozen():
    selection = read("paper_selection.json"); protocol = read("adaptation_protocol.json")
    assert selection["doi"] == "10.5220/0013789200003982"
    assert selection["primary_source_complete"] is True
    assert selection["performance_seen_before_selection"] is False
    assert protocol["written_before_paper_performance"] is True
    assert protocol["core_mechanism_preserved"] is True
    assert protocol["search"]["maximum_unique_configurations"] == 32
    assert len(protocol["search"]["stage_a"]["candidates"]) == 8
    assert len(protocol["search"]["stage_b"]["candidates"]) == 24
    assert protocol["holdout_executed"] is False


def test_incremental_qp_enforces_every_input_and_delta_move():
    rng = np.random.default_rng(3); a = 0.98 * np.eye(20); b = rng.normal(scale=.01, size=(20, 3)); c = rng.normal(size=(12, 20))
    p = {"horizon_updates": 6, "position_weight": 30., "velocity_weight": 4., "orientation_weight": 1., "angular_weight": .2, "input_weight": .1, "delta_weight": 2.}
    qp = build_incremental_qp(a, b, c, np.zeros(20), np.zeros(3), p)
    assert qp.P.shape == (18, 18)
    assert qp.A.shape == (36, 18)
    assert np.isfinite(qp.P).all() and np.isfinite(qp.q).all()
    assert np.max(qp.upper[::2]) == .25 and np.min(qp.lower[::2]) == -.25
