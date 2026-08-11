import numpy as np
from uav_sway.demo.extreme_runner import WIND_VECTORS_10


def test_extreme_wind_vectors():
    assert np.allclose(WIND_VECTORS_10["X"], [10, 0, 0])
    assert np.allclose(WIND_VECTORS_10["Y"], [0, 10, 0])
    assert np.allclose(WIND_VECTORS_10["XY30"], [8.660254037844386, 5, 0])
