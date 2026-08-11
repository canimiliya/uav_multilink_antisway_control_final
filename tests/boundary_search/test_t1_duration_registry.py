from uav_sway.demo.boundary_runner import CONTROLLERS, T1_DURATIONS, t1_jobs


def test_t1_duration_registry_is_exact_fixed_grid():
    jobs = t1_jobs()
    assert tuple(T1_DURATIONS) == (4.0, 4.5, 5.0, 5.5, 6.0)
    assert len(jobs) == 10
    assert {j["controller"] for j in jobs} == set(CONTROLLERS)
    assert sorted({j["move_duration_s"] for j in jobs}) == list(T1_DURATIONS)
