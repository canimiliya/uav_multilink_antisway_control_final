from uav_sway.demo.meeting_runner import _reference, run_all


def test_demo_import():
    assert callable(run_all)
    assert callable(_reference)
