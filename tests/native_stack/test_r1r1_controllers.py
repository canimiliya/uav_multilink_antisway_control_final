from __future__ import annotations

import numpy as np

from uav_sway.native_stack.r1r1_controllers import (
    NativeFullLQR, NativeGains, NativePID, candidate_registries, lqr_audit, physical_hover_model,
)


def test_physical_hover_model_has_four_wrench_inputs_and_is_controllable() -> None:
    a, b = physical_hover_model()
    assert a.shape == (12, 12)
    assert b.shape == (12, 4)
    assert np.count_nonzero(b[:, 0]) == 1
    audit = lqr_audit(.04, 3.0, 30.0, 8.0, 50.0)
    assert audit["rank_B"] == 4
    assert audit["controllability_rank"] == 12
    assert audit["controllable"] and audit["stabilizable"]
    assert max(x[0] for x in audit["closed_loop_poles"]) < 0.0


def test_candidate_budgets_and_physical_commands() -> None:
    registries = candidate_registries()
    assert {key: len(value) for key, value in registries.items()} == {
        "native_pid": 12, "native_full_lqr": 8, "native_task_lqr": 8, "satc_native": 16,
    }
    pid = NativePID(NativeGains((.1,)*3, (.5,)*3), "pid")
    assert pid.physical_command().as_array().shape == (4,)
    lqr = NativeFullLQR(registries["native_full_lqr"][0]["gains"], "lqr", registries["native_full_lqr"][0]["q"])
    assert lqr.diagnostics()["linear_model_audit"]["physical_input_dimension"] == 4
