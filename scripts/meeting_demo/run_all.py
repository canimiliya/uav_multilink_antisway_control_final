"""One-command runner for the three-task meeting package."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "third_party/udaan"))
from uav_sway.demo.meeting_runner import run_all


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plots-only", action="store_true")
    args = parser.parse_args()
    print(json.dumps(run_all(args.plots_only), indent=2, ensure_ascii=False, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
