"""Run the fixed P3-R1D extreme composite demo."""
from pathlib import Path
import json
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src")); sys.path.insert(0, str(ROOT / "third_party/udaan"))
from uav_sway.demo.extreme_runner import run_all

if __name__ == "__main__":
    print(json.dumps(run_all(), indent=2, ensure_ascii=False, default=str))
