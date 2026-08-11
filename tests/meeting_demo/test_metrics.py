import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_meeting_metrics_has_both_controllers_after_demo():
    path = ROOT / "outputs/meeting_demo/MEETING_METRICS.md"
    assert path.exists()
    text = path.read_text(encoding="utf-8")
    assert "full_lqr_048" in text and "satc_b_027" in text
