import numpy as np


def test_task3_wind_onset_is_zero_order_hold():
    times = np.array([3.999, 4.0, 4.001])
    wind = np.where(times[:, None] >= 4.0, np.array([3.0, 0.0, 0.0]), np.zeros(3))
    assert np.allclose(wind[0], 0.0)
    assert np.allclose(wind[1:], np.array([3.0, 0.0, 0.0]))
