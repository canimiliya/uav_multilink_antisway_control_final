"""Build the immutable R1R2 failure-mechanism audit from R1R1 evidence only."""

from __future__ import annotations

import csv
import hashlib
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
R1R1 = ROOT / "reproducibility/native_stack/r1r1"
OUT = ROOT / "reproducibility/native_stack/r1r2/failure_audit"
DOC = ROOT / "docs/native_stack/r1r2/TRADITIONAL_FAILURE_AUDIT.md"
FAMILIES = {
    "native_pid": "traditional/pid",
    "native_full_lqr": "traditional/full_lqr",
    "native_task_lqr": "traditional/task_lqr",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def target_bin(case: dict[str, Any]) -> tuple[str, str]:
    start = case["target"]["initial_cutter_target_world_m"]
    end = case["target"]["final_cutter_target_world_m"]
    horizontal = ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5
    vertical = abs(end[2] - start[2])
    h = "near_<=1m" if horizontal <= 1.0 else ("mid_1_to_2m" if horizontal <= 2.0 else "far_>2m")
    v = "low_<=0.5m" if vertical <= 0.5 else ("mid_0.5_to_1m" if vertical <= 1.0 else "high_>1m")
    return h, v


def failure_class(row: dict[str, Any]) -> str:
    pos = float(row["endpoint_position_error_m"])
    speed = float(row["endpoint_velocity_error_m_s"])
    if row["safe"].lower() != "true":
        return "E_safety_catastrophic"
    if float(row["deadline_miss_rate"]) > 0.01:
        return "F_deadline_issue"
    if pos <= 0.5 and speed > 0.5:
        return "A_position_pass_speed_fail"
    if 0.5 < pos <= 0.75:
        return "B_narrow_position_fail"
    if 0.75 < pos <= 1.5:
        return "C_moderate_position_fail"
    if pos > 1.5:
        return "D_large_tracking_failure"
    return "PASS"


def error_bin(value: float) -> str:
    if value <= 0.5:
        return "<=0.5"
    if value <= 0.75:
        return "0.5_to_0.75"
    if value <= 1.0:
        return "0.75_to_1.0"
    if value <= 1.5:
        return "1.0_to_1.5"
    return ">1.5"


def counter(rows: list[dict[str, Any]], key: str) -> dict[str, int]:
    return dict(sorted(Counter(r[key] for r in rows).items()))


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    n = len(rows)
    pos_pass_speed_fail = sum(r["position_pass"] and not r["speed_pass"] for r in rows)
    position_fail_speed_pass = sum(not r["position_pass"] and r["speed_pass"] for r in rows)
    both_fail = sum(not r["position_pass"] and not r["speed_pass"] for r in rows)
    task_failure: dict[str, dict[str, int]] = {}
    for task in sorted({r["task_family"] for r in rows}):
        task_failure[task] = counter([r for r in rows if r["task_family"] == task], "failure_class")
    target_matrix: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    for row in rows:
        target_matrix[row["horizontal_distance_bin"]][row["vertical_displacement_bin"]] += 1
    return {
        "case_count": n,
        "success_count": sum(r["success"] for r in rows),
        "success_rate": sum(r["success"] for r in rows) / n,
        "safe_count": sum(r["safe"] for r in rows),
        "safety_rate": sum(r["safe"] for r in rows) / n,
        "endpoint_gate_decomposition": {
            "position_pass_speed_fail": pos_pass_speed_fail,
            "position_pass_speed_fail_fraction": pos_pass_speed_fail / n,
            "position_fail_speed_pass": position_fail_speed_pass,
            "position_fail_speed_pass_fraction": position_fail_speed_pass / n,
            "both_fail": both_fail,
            "both_fail_fraction": both_fail / n,
        },
        "final_position_error_distribution": counter(rows, "final_error_bin"),
        "failure_class_distribution": counter(rows, "failure_class"),
        "by_task_family": task_failure,
        "by_wind_kind": counter(rows, "wind_kind"),
        "failure_by_wind_kind": {
            wind: counter([r for r in rows if r["wind_kind"] == wind], "failure_class")
            for wind in sorted({r["wind_kind"] for r in rows})
        },
        "failure_by_direction": {
            direction: counter([r for r in rows if r["wind_direction"] == direction], "failure_class")
            for direction in sorted({r["wind_direction"] for r in rows})
        },
        "target_bin_case_counts": {h: dict(sorted(v.items())) for h, v in sorted(target_matrix.items())},
        "failure_by_horizontal_distance": {
            value: counter([r for r in rows if r["horizontal_distance_bin"] == value], "failure_class")
            for value in sorted({r["horizontal_distance_bin"] for r in rows})
        },
        "failure_by_vertical_displacement": {
            value: counter([r for r in rows if r["vertical_displacement_bin"] == value], "failure_class")
            for value in sorted({r["vertical_displacement_bin"] for r in rows})
        },
    }


def main() -> None:
    manifest_path = ROOT / "reproducibility/native_stack/r0s/resolved_development_manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    cases = {case["sample_id"]: case for case in manifest["cases"]}
    payload: dict[str, Any] = {
        "task": "P2-R1R2 failure mechanism audit",
        "source": "P2-R1R1 frozen Development evidence only",
        "new_controller_executions": 0,
        "development_identity_hash": "0f03df8fe11310f6357197a9d03b605476831c76db58f64d04e92535e6df9473",
        "development_resolved_hash": "0034d481a1abe04a46505410a557dc24815932c8a07cf96b08156c374d1aefe9",
        "resolved_manifest_sha256": sha256(manifest_path),
        "families": {},
    }
    enriched_rows: list[dict[str, Any]] = []
    for family, rel in FAMILIES.items():
        freeze_path = R1R1 / rel / "freeze.json"
        results_path = R1R1 / rel / "development_results.csv"
        freeze = json.loads(freeze_path.read_text(encoding="utf-8"))
        selected = freeze["selected"]
        with results_path.open(encoding="utf-8", newline="") as handle:
            selected_rows = [r for r in csv.DictReader(handle) if r["method_id"] == selected]
        if len(selected_rows) != 200:
            raise AssertionError(f"{family}: expected 200 selected rows, got {len(selected_rows)}")
        family_rows = []
        for row in selected_rows:
            case = cases[row["sample_id"]]
            hbin, vbin = target_bin(case)
            pos = float(row["endpoint_position_error_m"])
            speed = float(row["endpoint_velocity_error_m_s"])
            clean = {
                "family": family,
                "method_id": selected,
                "sample_id": row["sample_id"],
                "task_family": row["task_family"],
                "wind_kind": row["wind_kind"],
                "wind_direction": row["wind_direction"],
                "horizontal_distance_bin": hbin,
                "vertical_displacement_bin": vbin,
                "safe": row["safe"].lower() == "true",
                "success": row["success"].lower() == "true",
                "position_pass": pos <= 0.5,
                "speed_pass": speed <= 0.5,
                "endpoint_position_error_m": pos,
                "endpoint_velocity_error_m_s": speed,
                "final_error_bin": error_bin(pos),
                "deadline_miss_rate": float(row["deadline_miss_rate"]),
            }
            clean["failure_class"] = failure_class(row)
            family_rows.append(clean)
        summary = summarize(family_rows)
        summary.update({
            "selected_method": selected,
            "r1r1_freeze_sha256": sha256(freeze_path),
            "r1r1_results_sha256": sha256(results_path),
        })
        payload["families"][family] = summary
        enriched_rows.extend(family_rows)

    payload["mechanism_conclusions"] = {
        "native_pid_primary_failure": "endpoint regulation, dominated by position failure rather than safety or deadline failure",
        "native_full_lqr_primary_failure": "servo/reference architecture: hover stabilization is valid but static LQR tracking lacks integral task regulation",
        "native_task_lqr_primary_failure": "task endpoint regulation and trajectory feedforward: native safety improved while tracking authority/servo structure remained insufficient",
        "setpoint": "steady-state endpoint regulation, trim allocation, and integral action dominate",
        "smooth_trajectory": "reference velocity/acceleration feedforward and causal task tracking dominate",
        "architecture_action": "expand causal PID servo/feedforward and LQI/LQT augmentation; do not change plant, benchmark, metrics, or competence gates",
    }
    OUT.mkdir(parents=True, exist_ok=True)
    audit_path = OUT / "traditional_failure_mechanism_audit.json"
    audit_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    with (OUT / "case_failure_classification.csv").open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(enriched_rows[0]))
        writer.writeheader(); writer.writerows(enriched_rows)

    lines = [
        "# P2-R1R2 Traditional failure audit", "",
        "This audit uses only frozen P2-R1R1 Development results; it executes no new controller cases.", "",
        "| Family | Selected | Safety | Success | Position-pass/speed-fail | Position-fail/speed-pass | Both fail |",
        "|---|---|---:|---:|---:|---:|---:|",
    ]
    for family, summary in payload["families"].items():
        gate = summary["endpoint_gate_decomposition"]
        lines.append(
            f"| {family} | `{summary['selected_method']}` | {summary['safety_rate']:.1%} | {summary['success_rate']:.1%} | "
            f"{gate['position_pass_speed_fail_fraction']:.1%} | {gate['position_fail_speed_pass_fraction']:.1%} | {gate['both_fail_fraction']:.1%} |"
        )
    lines += ["", "## Mechanism conclusion", "",
              "Native PID is already safe and meets the aggregate RMSE gates, but most non-successes are endpoint-regulation failures. "
              "The recovery therefore needs stronger causal steady-state/servo action and reference feedforward, not a relaxed success definition.", "",
              "The physical Full-LQR model is controllable and stabilizable; its loss is a reference-servo problem, not an inability of LQR to stabilize. "
              "Full-LQR recovery must add equilibrium shifting, feedforward, and integral augmentation while retaining linear-quadratic identity.", "",
              "Native Task-LQR traded the historical adapter's unsafe tracking behavior for safety, but lacks sufficient task endpoint regulation and trajectory feedforward. "
              "Task-LQR recovery must preserve the native safety gain while adding LQT/LQI task tracking.", "",
              "Detailed task, wind, direction, target-bin, and failure-class counts are frozen in `traditional_failure_mechanism_audit.json`.", ""]
    DOC.parent.mkdir(parents=True, exist_ok=True)
    DOC.write_text("\n".join(lines), encoding="utf-8")


if __name__ == "__main__":
    main()
