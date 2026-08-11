from pathlib import Path


def test_native_render_smoke_if_run_exists():
    root = Path(__file__).resolve().parents[2] / "outputs/meeting_demo_extreme_v3"
    videos = list(root.glob("**/T1_AGGRESSIVE_LQR.mp4"))
    if not videos:
        return
    assert videos[0].stat().st_size > 0
