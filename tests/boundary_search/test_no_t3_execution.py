from uav_sway.demo.boundary_runner import jobs


def test_r1f_registry_contains_no_t3_jobs():
    assert all(job["task"] in {"T1", "T2"} for job in jobs())
