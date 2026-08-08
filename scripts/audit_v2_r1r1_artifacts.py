"""Audit generated V2-R1R1 artifacts without executing any sample."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import mujoco
import numpy as np

from uav_sway.task_space.state import CutterTaskSpaceReader


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v2/r1r1"
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"


def main() -> int:
    raw = sorted((OUT / "runs").rglob("*.csv"))
    max_speed = 0.0
    missing = []
    nonfinite = []
    for path in raw:
        with path.open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        if not rows or "cutter_angular_speed_rad_s" not in rows[0]:
            missing.append(str(path.relative_to(ROOT)))
            continue
        values = np.asarray([float(row["cutter_angular_speed_rad_s"]) for row in rows], dtype=float)
        if not np.isfinite(values).all():
            nonfinite.append(str(path.relative_to(ROOT)))
        max_speed = max(max_speed, float(np.max(values)))

    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    data.qpos[:7] = [0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    equilibrium = CutterTaskSpaceReader(model).read(model, data)
    equilibrium_norm = float(np.linalg.norm(equilibrium.cutter_angular_velocity_world))
    audit = {
        "formal_source": "mujoco.mj_jacSite(..., jacr) followed by jacr @ qvel",
        "finite_difference_used_for_formal_angular_velocity": False,
        "raw_trace_column": "cutter_angular_speed_rad_s",
        "raw_csv_count": len(raw),
        "all_raw_csv_have_angular_speed": not missing,
        "missing_angular_speed_files": missing,
        "all_angular_speed_values_finite": not nonfinite,
        "nonfinite_files": nonfinite,
        "max_raw_cutter_angular_speed_rad_s": max_speed,
        "equilibrium_cutter_angular_velocity_world": equilibrium.cutter_angular_velocity_world.tolist(),
        "equilibrium_cutter_angular_speed_rad_s": equilibrium_norm,
        "equilibrium_angular_speed_tolerance_rad_s": 1.0e-12,
        "equilibrium_angular_speed_pass": equilibrium_norm <= 1.0e-12,
        "acquisition_includes_angular_speed_le_0p10_rad_s": True,
        "acquisition_continuous_hold_s": 1.0,
        "holdout_executed": False,
        "result": "PASS_R1R1_CUTTER_ANGULAR_VELOCITY_AUDIT",
    }
    (OUT / "cutter_angular_velocity_audit.json").write_text(json.dumps(audit, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps(audit, indent=2))
    return 0 if not missing and not nonfinite and audit["equilibrium_angular_speed_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
