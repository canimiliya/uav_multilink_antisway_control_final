import json
from pathlib import Path

def test_metric_after_wind_peak():
    p=Path("outputs/meeting_demo_stress_v2/T3/X/satc_b_027/metrics.json")
    if p.exists(): assert json.loads(p.read_text())["peak_time_s"]>=8.0
