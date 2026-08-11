from uav_sway.demo.boundary_runner import T2_WINDS, t2_integer_jobs


def test_t2_integer_wind_registry_is_exact_fixed_grid():
    jobs = t2_integer_jobs(5.0)
    assert tuple(T2_WINDS) == (0.0, 1.0, 2.0, 3.0, 4.0, 5.0)
    assert len(jobs) == 12
    assert sorted({j["speed_mps"] for j in jobs}) == list(T2_WINDS)
    assert all(j["wind_axis"] if "wind_axis" in j else True for j in jobs)
