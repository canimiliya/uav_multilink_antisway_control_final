from pathlib import Path

def test_native_render_frame():
    assert Path("outputs/meeting_demo_stress_v2/T1/full_lqr_048/01_native_model_start.png").exists() or Path("outputs/meeting_demo_extreme_v3/T1/full_lqr_048/01_native_model_start.png").exists()
