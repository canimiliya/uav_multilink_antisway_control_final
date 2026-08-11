from uav_sway.demo.boundary_runner import t2_integer_jobs


def test_zero_wind_job_is_explicitly_zero_not_legacy_default_wind():
    zero = [j for j in t2_integer_jobs(6.0) if j["speed_mps"] == 0.0]
    assert len(zero) == 2
    assert all(j["zero_wind"] for j in zero)
