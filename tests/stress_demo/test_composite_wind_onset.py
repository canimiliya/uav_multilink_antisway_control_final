import numpy as np
from uav_sway.demo.extreme_runner import WIND_VECTORS_5, WIND_VECTORS_10, wind_profile


def test_composite_wind_onsets_and_ramps():
    assert np.allclose(wind_profile("XY30", 2.5, 5.0), 0.0)
    assert np.allclose(wind_profile("XY30", 3.5, 5.0), WIND_VECTORS_5["XY30"])
    assert np.allclose(wind_profile("XY30", 8.0, 10.0), 0.0)
    assert np.allclose(wind_profile("XY30", 9.0, 10.0), WIND_VECTORS_10["XY30"])
