from uav_sway.demo.boundary_runner import _t2_boundary


def _result(wind, controller, stable):
    return {"job": {"task": "T2", "speed_mps": wind, "controller": controller}, "metrics": {"STABLE_RECOVERED": stable}}


def test_midpoint_rule_pairs_controllers_for_each_controller_bracket():
    results = []
    for wind in range(6):
        results += [_result(wind, "full_lqr_048", wind <= 2), _result(wind, "satc_b_027", wind <= 3)]
    boundary, midpoint = _t2_boundary(results, 5.0)
    assert boundary["midpoint_winds_tested"] == [2.5, 3.5]
    assert len(midpoint) == 4
    assert {j["controller"] for j in midpoint} == {"full_lqr_048", "satc_b_027"}
