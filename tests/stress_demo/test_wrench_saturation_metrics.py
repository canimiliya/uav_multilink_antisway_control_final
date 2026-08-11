import json
from pathlib import Path


def test_wrench_saturation_metrics_are_recorded_if_run_exists():
    root = Path(__file__).resolve().parents[2] / "outputs/meeting_demo_extreme_v3"
    metrics = list(root.glob("**/metrics.json"))
    if not metrics:
        return
    payload = json.loads(metrics[0].read_text(encoding="utf-8"))
    for key in ("thrust_saturation_rate", "torque_saturation_rate", "any_wrench_saturation_rate"):
        assert key in payload
        assert 0.0 <= payload[key] <= 1.0
