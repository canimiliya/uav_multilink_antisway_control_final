from uav_sway.demo.recoverable_runner import six_second_reference


def test_reference_callable():
    assert callable(six_second_reference)
