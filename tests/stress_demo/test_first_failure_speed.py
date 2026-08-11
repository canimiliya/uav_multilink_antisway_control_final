from uav_sway.demo.recoverable_runner import WIND_SPEEDS


def test_first_failure_scan_has_successor_speeds():
    assert len(WIND_SPEEDS) == 8

