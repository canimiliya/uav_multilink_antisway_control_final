from uav_sway.demo.stress_runner import jobs

def test_parallel_registry(): assert len(jobs())==10

