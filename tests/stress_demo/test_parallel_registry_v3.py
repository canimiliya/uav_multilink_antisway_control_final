import json
from pathlib import Path


def test_parallel_registry_v3_if_run_exists():
    path = Path(__file__).resolve().parents[2] / "artifacts/meeting_demo_extreme_v3/job_registry.json"
    if not path.exists():
        return
    assert len(json.loads(path.read_text(encoding="utf-8"))) == 10
