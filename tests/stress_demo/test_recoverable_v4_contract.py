import numpy as np

from uav_sway.demo.recoverable_runner import CONTROLLERS, TARGET_DELTA, WIND_SPEEDS, jobs, six_second_reference, wind_profile


def test_parallel_20_jobs_and_speed_registry():
    registry = jobs()
    assert len(registry) == 20
    assert sum(j["task"] == "T1" for j in registry) == 2
    assert sum(j["task"] == "T2" for j in registry) == 2
    assert sum(j["task"] == "T3" for j in registry) == 16
    assert {j["controller"] for j in registry} == set(CONTROLLERS)
    assert sorted({j.get("speed_mps") for j in registry if j["task"] == "T3"}) == list(WIND_SPEEDS)


def test_t1_six_second_reference_contract():
    p0 = np.zeros(3); target = TARGET_DELTA.copy()
    assert np.allclose(six_second_reference(p0, target, 0.0).position_world, p0)
    mid = six_second_reference(p0, target, 4.0)
    assert np.all(mid.position_world > p0)
    assert np.allclose(six_second_reference(p0, target, 7.0).position_world, target)


def test_t2_xwind_contract():
    assert np.allclose(wind_profile("T2", 2.9), [0, 0, 0])
    assert np.allclose(wind_profile("T2", 4.0), [5, 0, 0])


def test_wind_speed_registry_and_recoverable_definition_inputs():
    assert WIND_SPEEDS == (3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0)
    assert np.isclose(np.linalg.norm(wind_profile("T3", 9.0, 5.0)), 5.0)


def test_outer_hit_tolerance_is_frozen_in_source_contract():
    source = __import__("inspect").getsource(__import__("uav_sway.demo.recoverable_runner", fromlist=["_outer_truth"])._outer_truth)
    assert "1e-6" in source
    assert "slew_limit_m_s2_per_update" in source
