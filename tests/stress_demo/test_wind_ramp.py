import numpy as np
from uav_sway.demo.stress_runner import wind_profile

def test_wind_ramp(): assert np.allclose(wind_profile("X",8),0); assert np.allclose(wind_profile("X",10),[5,0,0])

