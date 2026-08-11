import numpy as np
from uav_sway.demo.stress_runner import WIND_VECTORS

def test_xy30_norm(): assert abs(np.linalg.norm(WIND_VECTORS["XY30"])-5)<1e-12

