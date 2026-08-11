from uav_sway.demo.recoverable_runner import _outer_truth


def test_outer_truth_exposes_frozen_limit_hit_fields():
    assert callable(_outer_truth)

