from uav_sway.demo.recoverable_runner import WIND_SPEEDS


def test_max_speed_scan_is_ordered():
    assert list(WIND_SPEEDS) == sorted(WIND_SPEEDS)

