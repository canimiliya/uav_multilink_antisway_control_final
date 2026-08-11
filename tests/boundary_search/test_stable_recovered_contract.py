def test_stable_contract_thresholds_are_frozen_in_runner_source():
    import inspect
    from uav_sway.demo import recoverable_runner
    source = inspect.getsource(recoverable_runner._run_job)
    assert "STABLE_RECOVERED" in source
    assert "<= 0.15" in source
    assert "<= 0.20" in source
    assert "<= 1.0" in source
