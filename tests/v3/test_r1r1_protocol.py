"""Pre-corrective-performance checks for V3-R1R1."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from uav_sway.task_space.state import CutterTaskState
from uav_sway.v3.controllers import V3TaskPID
from uav_sway.v3.observation import V3Observation, V3Reference


ROOT = Path(__file__).resolve().parents[2]
R1R1 = ROOT / "reproducibility/v3/r1r1"


def _observation(error: np.ndarray) -> tuple[V3Observation, V3Reference]:
    zero = np.zeros(3)
    task = CutterTaskState(np.asarray(error, dtype=float), zero, zero, [1, 0, 0], np.eye(3))
    return V3Observation(np.zeros(20), zero, zero, task), V3Reference(zero, zero, zero, 0.0)


def test_old_pid_positive_and_negative_amplitude_antiwindup_bug_is_preserved() -> None:
    for sign in (-1.0, 1.0):
        pid = V3TaskPID([10, 0, 0], [0, 0, 0], [1, 0, 0])
        observation, reference = _observation(np.asarray([sign, 0, 0]))
        pid.command(observation, reference, 0.05)
        assert pid.diagnostics.saturated[0]
        assert pid.integral[0] == sign * 0.05


def test_old_pid_positive_and_negative_slew_antiwindup_bug_is_preserved() -> None:
    for sign in (-1.0, 1.0):
        pid = V3TaskPID([1, 0, 0], [0, 0, 0], [1, 0, 0])
        observation, reference = _observation(np.asarray([sign, 0, 0]))
        pid.command(observation, reference, 0.05)
        assert pid.diagnostics.slew_limited[0]
        assert pid.integral[0] == sign * 0.05


def test_protocol_freezes_scope_before_corrective_performance() -> None:
    protocol = json.loads((R1R1 / "pid_corrective_protocol.json").read_text(encoding="utf-8"))
    assert protocol["start_head"] == "4c1bdda538ceee970bff3f28162b40bcbd846081"
    assert protocol["benchmark_frozen"] is True
    assert protocol["holdout_execution_allowed"] is False
    assert protocol["read_only_traditional"] == ["full_lqr_048", "task_lqr_009"]
    assert protocol["rounds"][0]["candidate_budget"] == 64


def test_r1_evidence_tree_is_still_byte_addressed() -> None:
    gate = ROOT / "reproducibility/v3/r1/gate.json"
    assert hashlib.sha256(gate.read_bytes()).hexdigest() == "09783df06d4378f439fceee211a032014ca041a530fdc74d34f4fef4aa151782"
