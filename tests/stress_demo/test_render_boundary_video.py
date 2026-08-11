from uav_sway.demo.recoverable_runner import _render, _side_by_side


def test_native_render_helpers_exist():
    assert callable(_render)
    assert callable(_side_by_side)
