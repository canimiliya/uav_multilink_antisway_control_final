from uav_sway.demo.boundary_runner import DOC, OUT


def test_final_contract_targets_are_r1f_paths():
    assert DOC.name == "clean_release"
    assert OUT.name == "meeting_demo_boundary_v5"
    assert "FINAL_THREE_SCENARIO_CONTRACT.md".endswith(".md")
