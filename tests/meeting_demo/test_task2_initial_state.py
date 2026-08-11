import numpy as np


def test_task2_initial_joint_rms_definition():
    angles = np.deg2rad([5.0, -4.0, 3.0, -2.0, 1.0])
    expected = float(np.sqrt(np.mean(angles**2)))
    assert np.isclose(expected, 0.0578860226449684)
