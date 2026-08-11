import numpy as np
from uav_sway.demo.extreme_runner import WIND_VECTORS_5, wind_profile


def test_task2_composite_wind_contract():
    assert np.allclose(WIND_VECTORS_5["XY30"], [4.330127018922193, 2.5, 0.0])
    assert np.isclose(np.linalg.norm(WIND_VECTORS_5["XY30"]), 5.0)
    assert np.allclose(wind_profile("XY30", 2.5, 5.0), 0.0)
    assert np.allclose(wind_profile("XY30", 3.5, 5.0), WIND_VECTORS_5["XY30"])
