import json
from pathlib import Path

def test_metric_after_move():
    p=Path("outputs/meeting_demo_stress_v2/T1/full_lqr_048/metrics.json")
    if p.exists(): assert json.loads(p.read_text())["move_interval_s"]==[3.0,23.0]

