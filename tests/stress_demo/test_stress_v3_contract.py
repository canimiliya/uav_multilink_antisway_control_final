from pathlib import Path
import numpy as np
from uav_sway.demo.extreme_runner import (
    INITIAL_ANGLES_DEG, TARGET_DELTA, WIND_VECTORS_10, WIND_VECTORS_5,
    aggressive_reference, jobs, wind_profile,
)


def test_v3_has_exactly_ten_jobs_and_frozen_inputs():
    assert len(jobs()) == 10
    assert np.allclose(INITIAL_ANGLES_DEG, [20, -16, 12, -8, 4])
    assert np.allclose(TARGET_DELTA, [2.0, 1.7, 4.5])


def test_v3_reference_is_four_second_quintic():
    p0 = np.zeros(3); target = np.array([2.0, 1.7, 4.5])
    assert np.allclose(aggressive_reference(p0, target, 1.0).position_world, p0)
    assert np.allclose(aggressive_reference(p0, target, 5.0).position_world, target)
    assert np.allclose(aggressive_reference(p0, target, 1.0).velocity_world, 0)


def test_t2_xy30_ramp_and_onset():
    assert np.allclose(wind_profile("XY30", 2.5, 5.0), 0)
    assert np.allclose(wind_profile("XY30", 3.5, 5.0), WIND_VECTORS_5["XY30"])


def test_10ms_vectors_have_exact_norm():
    for value in WIND_VECTORS_10.values():
        assert abs(float(np.linalg.norm(value)) - 10.0) < 1e-12
