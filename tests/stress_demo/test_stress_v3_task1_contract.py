import numpy as np
from uav_sway.demo.extreme_runner import INITIAL_ANGLES_DEG, TARGET_DELTA, aggressive_reference


def test_task1_extreme_contract():
    assert np.array_equal(INITIAL_ANGLES_DEG, [20, -16, 12, -8, 4])
    assert np.array_equal(TARGET_DELTA, [2.0, 1.7, 4.5])
    ref = aggressive_reference(np.zeros(3), TARGET_DELTA, 5.0)
    assert np.allclose(ref.position_world, TARGET_DELTA)
