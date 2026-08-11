import numpy as np
from uav_sway.demo.stress_runner import WIND_VECTORS

def test_wind_vectors():
    assert np.allclose(WIND_VECTORS["X"],[5,0,0]); assert np.allclose(WIND_VECTORS["Y"],[0,5,0])

