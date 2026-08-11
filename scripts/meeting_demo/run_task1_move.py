"""Run the single T1 meeting task."""
import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "third_party/udaan"))
from uav_sway.demo.meeting_runner import run_single

if __name__ == "__main__":
    p=argparse.ArgumentParser(); p.add_argument("--controller", choices=("lqr","satc"), required=True); a=p.parse_args()
    print(json.dumps(run_single("task1_move", a.controller), indent=2))
