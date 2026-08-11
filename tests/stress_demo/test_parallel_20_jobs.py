from uav_sway.demo.recoverable_runner import jobs


def test_job_count_is_twenty():
    assert len(jobs()) == 20

