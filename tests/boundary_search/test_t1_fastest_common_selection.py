from uav_sway.demo.boundary_runner import T1_DURATIONS, _t1_boundary


def _result(duration, controller, stable):
    return {"job": {"task": "T1", "move_duration_s": duration, "controller": controller}, "metrics": {"STABLE_RECOVERED": stable}}


def test_t1_common_selection_uses_first_grid_case_both_pass():
    results = []
    for d in T1_DURATIONS:
        results += [_result(d, "full_lqr_048", d >= 5.0), _result(d, "satc_b_027", d >= 4.5)]
    boundary = _t1_boundary(results)
    assert boundary["LQR_fastest_stable"] == 5.0
    assert boundary["SATC_fastest_stable"] == 4.5
    assert boundary["COMMON_fastest_stable"] == 5.0
    assert boundary["SATC_boundary_bracket"] == {"last_failed_duration": 4.0, "first_stable_duration": 4.5}
