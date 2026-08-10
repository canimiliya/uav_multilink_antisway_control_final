"""Compute the post-freeze, no-authority Governance v2 retrospective."""

from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/native_stack/governance"
NOMINAL = {"calm", "moderate", "stochastic"}
METHODS = {
    "native_pid_001": ROOT / "reproducibility/native_stack/r1r1/traditional/pid/development_results.csv",
    "native_full_lqr_006": ROOT / "reproducibility/native_stack/r1r1/traditional/full_lqr/development_results.csv",
    "native_task_lqr_004": ROOT / "reproducibility/native_stack/r1r1/traditional/task_lqr/development_results.csv",
}


def boolean(row, key):
    return row[key].lower() == "true"


def mean(rows, key):
    return float(np.mean([float(row[key]) for row in rows]))


def evaluate(method_id, path, thresholds):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream) if row["method_id"] == method_id]
    nominal = [row for row in rows if row["wind_kind"] in NOMINAL]
    nominal_setpoint = [row for row in nominal if row["task_family"] == "setpoint"]
    nominal_trajectory = [row for row in nominal if row["task_family"] == "smooth_trajectory"]
    values = {
        "all_case_safety_rate": float(np.mean([boolean(row, "safe") for row in rows])),
        "all_case_catastrophic_count": sum(boolean(row, "catastrophic") for row in rows),
        "deadline_miss_rate": mean(rows, "deadline_miss_rate"),
        "nominal_success_rate": float(np.mean([boolean(row, "success") for row in nominal])),
        "nominal_each_wind_success_rate": {
            wind: float(np.mean([boolean(row, "success") for row in nominal if row["wind_kind"] == wind]))
            for wind in ("calm", "moderate", "stochastic")
        },
        "nominal_each_task_family_success_rate": {
            task: float(np.mean([boolean(row, "success") for row in nominal if row["task_family"] == task]))
            for task in ("setpoint", "smooth_trajectory")
        },
        "nominal_setpoint_rmse_m": mean(nominal_setpoint, "position_rmse_m"),
        "nominal_trajectory_rmse_m": mean(nominal_trajectory, "position_rmse_m"),
    }
    checks = {
        "all_case_safety": values["all_case_safety_rate"] >= thresholds["all_case_safety_rate_min"],
        "all_case_catastrophic": values["all_case_catastrophic_count"] <= thresholds["all_case_catastrophic_count_max"],
        "deadline": values["deadline_miss_rate"] <= thresholds["deadline_miss_rate_max"],
        "nominal_success": values["nominal_success_rate"] >= thresholds["nominal_success_rate_min"],
        "nominal_each_wind": all(value >= thresholds["nominal_each_wind_success_rate_min"] for value in values["nominal_each_wind_success_rate"].values()),
        "nominal_each_task_family": all(value >= thresholds["nominal_each_task_family_success_rate_min"] for value in values["nominal_each_task_family_success_rate"].values()),
        "nominal_setpoint_rmse": values["nominal_setpoint_rmse_m"] <= thresholds["nominal_setpoint_rmse_max_m"],
        "nominal_trajectory_rmse": values["nominal_trajectory_rmse_m"] <= thresholds["nominal_trajectory_rmse_max_m"],
    }
    return {
        "method_id": method_id,
        "source": path.relative_to(ROOT).as_posix(),
        "values": values,
        "checks": checks,
        "WOULD_EXISTING_METHOD_PASS": all(checks.values()),
        "qualification_authority": "NONE_RETROSPECTIVE_ONLY",
    }


def main():
    contract = json.loads((OUT / "competence_governance_v2.json").read_text(encoding="utf-8"))
    if not contract["frozen_before_retrospective_existing_method_diagnostic"]:
        raise AssertionError("Governance v2 was not frozen before retrospective")
    thresholds = contract["traditional_eligibility"]
    results = {method: evaluate(method, path, thresholds) for method, path in METHODS.items()}
    output = {
        "version": "p2-r0g-v2-retrospective-v1",
        "governance_freeze_commit": "29b26646a4e97e6962cbaa1d2fda84ad9756d560",
        "computed_after_rule_freeze": True,
        "thresholds_changed_after_computation": False,
        "selection_authority": "NONE",
        "retroactive_pass_allowed": False,
        "methods": results,
        "existing_method_pass_count": sum(result["WOULD_EXISTING_METHOD_PASS"] for result in results.values()),
        "interpretation": "Governance v2 corrects research-role coupling but does not rescue an existing method. native_pid_001 remains the strongest evidence and fails nominal reliability, especially moderate wind, despite all-case safety and nominal tracking RMSE.",
    }
    (OUT / "retrospective_existing_method_diagnostic.json").write_text(json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
