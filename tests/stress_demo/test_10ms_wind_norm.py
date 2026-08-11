import numpy as np
from uav_sway.demo.extreme_runner import WIND_VECTORS_10


def test_extreme_wind_norm_is_ten():
    for vector in WIND_VECTORS_10.values():
        assert abs(float(np.linalg.norm(vector)) - 10.0) < 1e-12
