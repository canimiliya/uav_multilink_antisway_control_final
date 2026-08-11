from uav_sway.demo.boundary_runner import _render_final


def test_render_final_is_the_only_render_selection_entrypoint():
    import inspect
    source = inspect.getsource(_render_final)
    assert "{task}_FINAL_LQR.mp4" in source
    assert "{task}_FINAL_SATC.mp4" in source
    assert "COMMON STABLE" in source
