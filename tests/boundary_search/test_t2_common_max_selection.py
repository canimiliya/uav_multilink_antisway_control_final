from uav_sway.demo.boundary_runner import _t2_reselect


def _result(wind, controller, stable):
    return {"job": {"task": "T2", "speed_mps": wind, "controller": controller}, "metrics": {"STABLE_RECOVERED": stable}}


def test_t2_common_selection_is_highest_evaluated_common_pass():
    results = []
    for wind in (0.0, 1.0, 2.0, 2.5, 3.0):
        results += [_result(wind, "full_lqr_048", wind <= 2.5), _result(wind, "satc_b_027", wind <= 3.0)]
    t2 = {"integer_winds_tested": [0, 1, 2, 3, 4, 5], "midpoint_winds_tested": [2.5], "move_duration_s": 5.0}
    assert _t2_reselect(results, t2)["COMMON_max_stable"] == 2.5
