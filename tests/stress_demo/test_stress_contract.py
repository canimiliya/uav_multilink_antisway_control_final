import numpy as np
from uav_sway.demo.stress_runner import WIND_VECTORS, jobs, quintic, wind_profile

def test_task1_reference_quintic_endpoints():
    assert quintic(0)[0] == 0 and quintic(1)[0] == 1
    assert np.allclose(quintic(.5)[:3], (0.5, 1.875, 0.0))

def test_fixed_angles_and_wind_vectors():
    assert np.allclose(np.degrees(np.deg2rad([20,-16,12,-8,4])), [20,-16,12,-8,4])
    assert abs(np.linalg.norm(WIND_VECTORS["XY30"]) - 5.0) < 1e-12
    assert np.allclose(wind_profile("X", 8), 0) and np.allclose(wind_profile("X", 10), [5,0,0])

def test_parallel_job_registry():
    assert len(jobs()) == 10
    assert sum(j["task"] == "T3" for j in jobs()) == 6
