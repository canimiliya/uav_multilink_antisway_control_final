from uav_sway.demo.recoverable_runner import WIND_SPEEDS


def test_registry_is_fixed():
    assert WIND_SPEEDS == (3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0)
