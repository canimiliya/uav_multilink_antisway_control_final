"""Execute the frozen P2-R0G diagnostic oracle on 200 Development cases."""

from __future__ import annotations

import csv
import json
import os
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np

from uav_sway.native_stack.governance_oracle import ORACLE_AUTHORITY, evaluate_oracle_case


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/native_stack/governance"
NOMINAL = {"calm", "moderate", "stochastic"}
CHALLENGE = {"strong_sustained", "strong_transient", "ramp"}


def summary(rows):
    def block(subset):
        return {
            "cases": len(subset),
            "safety_rate": float(np.mean([row["safe"] for row in subset])),
            "success_rate": float(np.mean([row["success"] for row in subset])),
            "position_rmse_mean_m": float(np.mean([row["position_rmse_m"] for row in subset])),
            "endpoint_position_error_mean_m": float(np.mean([row["endpoint_position_error_m"] for row in subset])),
            "endpoint_velocity_error_mean_m_s": float(np.mean([row["endpoint_velocity_error_m_s"] for row in subset])),
        }
    by_wind = {wind: block([row for row in rows if row["wind_kind"] == wind]) for wind in sorted(NOMINAL | CHALLENGE)}
    by_task = {task: block([row for row in rows if row["task_family"] == task]) for task in ("setpoint", "smooth_trajectory")}
    nominal = [row for row in rows if row["wind_kind"] in NOMINAL]
    challenge = [row for row in rows if row["wind_kind"] in CHALLENGE]
    max_torque = np.max(np.asarray([row["max_requested_or_applied_torque_Nm"] for row in rows]), axis=0)
    return {
        "version": "p2-r0g-oracle-diagnostic-v1",
        "executed": True,
        "development_cases": len(rows),
        "execution_authority": ORACLE_AUTHORITY,
        "selection_authority": "NONE",
        "controller_baseline": False,
        "all": block(rows),
        "nominal": block(nominal),
        "challenge": block(challenge),
        "by_wind": by_wind,
        "by_task_family": by_task,
        "required_wrench": {
            "max_thrust_N": max(row["max_requested_or_applied_thrust_N"] for row in rows),
            "max_abs_torque_Nm": max_torque.tolist(),
            "max_thrust_limit_N": 285.74568,
            "max_abs_torque_limits_Nm": [25.0, 25.0, 12.0],
            "max_thrust_saturation_rate": max(row["thrust_saturation_rate"] for row in rows),
            "max_torque_saturation_rate": max(row["torque_saturation_rate"] for row in rows),
        },
        "max_true_external_force_N": max(row["max_true_external_force_N"] for row in rows),
        "failure_mechanisms": dict(Counter(row["failure_mechanism"] for row in rows)),
        "MISSION_ENVELOPE_STRUCTURAL_PROBLEM": float(np.mean([row["success"] for row in rows])) < 0.80,
        "interpretation_rule": "<80% oracle endpoint success indicates a structural mission concern; >=80% supports physical reachability but does not validate any controller",
        "holdout_executed": False,
    }


def main():
    protocol = json.loads((OUT / "oracle_protocol.json").read_text(encoding="utf-8"))
    if not protocol["frozen_before_oracle_execution"] or protocol["search_or_tuning_allowed"]:
        raise AssertionError("oracle protocol is not frozen")
    manifest = json.loads((ROOT / "reproducibility/native_stack/r0/native_development_manifest.json").read_text(encoding="utf-8"))
    identities = manifest["cases"]
    if len(identities) != 200 or any(case["split"] != "development" for case in identities):
        raise AssertionError("unexpected Development manifest")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    with ProcessPoolExecutor(max_workers=4) as pool:
        rows = list(pool.map(evaluate_oracle_case, identities, chunksize=1))
    rows.sort(key=lambda row: row["sample_id"])
    fieldnames = list(rows[0])
    with (OUT / "oracle_dynamic_attempt_results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            serialized = dict(row)
            serialized["safety_reasons"] = json.dumps(serialized["safety_reasons"], separators=(",", ":"))
            serialized["max_requested_or_applied_torque_Nm"] = json.dumps(serialized["max_requested_or_applied_torque_Nm"], separators=(",", ":"))
            writer.writerow(serialized)
    (OUT / "oracle_dynamic_attempt.json").write_text(json.dumps(summary(rows), indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
