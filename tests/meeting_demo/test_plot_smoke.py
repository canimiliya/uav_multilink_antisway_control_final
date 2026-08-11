from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_meeting_summary_plot_exists_after_demo():
    assert (ROOT / "outputs/meeting_demo/meeting_summary.png").exists()
