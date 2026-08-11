import numpy as np

from uav_sway.demo.meeting_runner import _reference


def test_task1_reference_endpoints_and_rest():
    p0 = np.array([0.2, 0.0, 0.4]); target = p0 + np.array([.3, .2, .15])
    before = _reference(p0, target, 0.0, 1.0, 5.0)
    end = _reference(p0, target, 5.0, 1.0, 5.0)
    assert np.allclose(before.position_world, p0)
    assert np.allclose(end.position_world, target)
    assert np.allclose(end.velocity_world, 0.0)
