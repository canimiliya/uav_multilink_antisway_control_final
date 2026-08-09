"""Record immutable implementation evidence before V6 performance."""

from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v6/papers/implementation_freeze.json"
SUITE_HEAD = "104991c2f0b5bdcdaed2ba20d11485f7bfea9ebc"


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def main() -> int:
    head = subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip()
    if head != SUITE_HEAD:
        raise RuntimeError("implementation freeze must be prepared directly after suite freeze")
    files = [
        "src/uav_sway/v6/paper_controllers.py", "scripts/run_v6_development.py",
        "reproducibility/v6/papers/yu2026_adaptation_contract.json",
        "reproducibility/v6/papers/sep2026_adaptation_contract.json",
        "reproducibility/v6/papers/development_protocol.json",
    ]
    payload = {
        "status": "V6_PAPER_IMPLEMENTATIONS_FROZEN_BEFORE_PERFORMANCE",
        "suite_freeze_head": SUITE_HEAD,
        "implementations": {
            "YU2026": "src/uav_sway/v6/paper_controllers.py::Yu2026FiniteTimeCFO",
            "SEP2026": "src/uav_sway/v6/paper_controllers.py::SEP2026PassivityNMPC",
        },
        "sha256": {name: sha(ROOT / name) for name in files},
        "traditional_mutated": False, "satc_mutated": False,
        "paper_performance_executed": False, "holdout_accessed": False,
        "parameters_and_implementation_mutable_after_freeze": False,
    }
    OUT.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
