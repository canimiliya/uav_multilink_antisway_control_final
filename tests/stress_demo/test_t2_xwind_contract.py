from uav_sway.demo.recoverable_runner import wind_profile


def test_xwind_is_world_x():
    assert tuple(wind_profile("T2", 4.0, 5.0)) == (5.0, 0.0, 0.0)

