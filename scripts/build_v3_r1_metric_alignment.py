"""Build the V3-R1 task metric alignment audit without performance runs."""

from __future__ import annotations

import json
from pathlib import Path

from uav_sway.v3.metrics import build_task_metric_alignment


ROOT = Path(__file__).resolve().parents[1]
R1 = ROOT / "reproducibility" / "v3" / "r1"


def main() -> int:
    audit = build_task_metric_alignment(ROOT)
    payload = {
        "task": "V3-R1-task-metric-alignment",
        "source": "frozen MuJoCo model and equilibrium finite differences",
        "R0_C_task_v3_unchanged": True,
        "outputs": ["C_pos", "C_vel", "C_dir", "C_omega_perp"],
        **audit,
        "pass": bool(audit["finite"]),
    }
    R1.mkdir(parents=True, exist_ok=True)
    (R1 / "task_metric_alignment_audit.json").write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"result": "V3_R1_TASK_METRIC_ALIGNMENT_READY", "pass": payload["pass"], "ranks": payload["ranks"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
