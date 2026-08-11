from uav_sway.demo.recoverable_runner import jobs


def test_recoverable_jobs_have_both_frozen_controllers():
    assert {j["controller"] for j in jobs()} == {"full_lqr_048", "satc_b_027"}

