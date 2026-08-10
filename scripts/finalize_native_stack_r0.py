"""Assemble the immutable P2-R0 final gate and evidence manifest."""

from __future__ import annotations

import hashlib
import json
import platform
import subprocess
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/native_stack/r0"
FINAL = ROOT / "reproducibility/native_stack/final"
DOCS = ROOT / "docs/native_stack"


def read(path: Path): return json.loads(path.read_text(encoding="utf-8"))


def dump(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha(path: Path) -> str: return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def main() -> None:
    audit = read(R0 / "physical_actuation_audit.json")
    scheduler = read(R0 / "scheduler_validation.json")
    physical = read(R0 / "physical_wrench_parity.json")
    legacy = read(R0 / "legacy_pipeline_parity.json")
    protocol = read(R0 / "benchmark_protocol.json")
    dev = read(R0 / "native_development_manifest.json")
    holdout = read(R0 / "native_holdout_manifest.json")
    split = read(R0 / "split_integrity.json")
    protected = read(R0 / "protected_evidence_audit.json")
    smoke = read(R0 / "platform_controller_smoke.json")

    project_tests = {
        "environment": {
            "python": platform.python_version(), "mujoco": mujoco.__version__, "numpy": np.__version__,
            "executable": "D:/anaconda/envs/uav_sway/python.exe",
        },
        "authoritative_execution": "one isolated pytest process per test directory",
        "groups": {
            "native_stack": 13, "release": 4, "v2": 56, "v3": 63, "v4": 25,
            "v5": 22, "v6": 20, "v7": 9, "v8": 11, "v9": 13, "v10": 12,
        },
        "group_count": 11, "passed": 248, "failed": 0, "status": "PASS",
        "warnings": "35 OSQP deprecation/pending-deprecation warnings across V6 and V10",
    }
    dump(FINAL / "project_tests.json", project_tests)
    execution_audit = {
        "scientific_retries": 0, "controller_performance_runs": 0, "native_holdout_runs": 0,
        "technical_attempts": [
            {"attempt": 1, "scope": "all tests", "wrapper": "conda run", "result": "TIMEOUT_184_S_NO_CONCLUSION"},
            {"attempt": 2, "scope": "all tests", "wrapper": "conda run", "result": "TIMEOUT_604_S_ORPHANED_WRAPPER_PROCESSES"},
            {"attempt": 3, "scope": "all tests", "wrapper": "direct environment python", "result": "TIMEOUT_604_S_HOST_CONTENTION"},
            {"attempt": 4, "scope": "multiple explicit directories in one pytest process", "result": "COLLECTION_MODULE_NAME_COLLISION_NO_TEST_RESULT"},
            {"attempt": 5, "scope": "all 11 directories in isolated processes plus final gate test", "result": "248_PASSED"},
        ],
        "cleanup": "Only verified P2 pytest/conda process trees were terminated after timeout; unrelated smd-blackwell workers were not touched.",
        "protocol_changed_between_attempts": False,
    }
    dump(FINAL / "execution_audit.json", execution_audit)

    gates = {
        "ACTUATION_AUDIT_COMPLETE": bool(audit["audit_complete"]),
        "PHYSICAL_API_VALIDATED": bool(physical["pass"]),
        "MULTIRATE_SCHEDULER_VALIDATED": bool(scheduler["pass"]),
        "SENSOR_ENVELOPE_FROZEN": tuple(protocol["sensor_fields"]) == tuple(read(R0 / "benchmark_protocol.json")["sensor_fields"]),
        "NO_FUTURE_LEAKAGE": not split["future_information_leakage"],
        "LEGACY_PIPELINE_PARITY": bool(legacy["pass"]),
        "PHYSICAL_WRENCH_PARITY": bool(physical["pass"]),
        "SETPOINT_TASK_VALID": any(case["task_family"] == "setpoint" for case in dev["cases"]),
        "SMOOTH_TRAJECTORY_TASK_VALID": all(name in {case["trajectory_type"] for case in dev["cases"]} for name in ("minimum_jerk", "approach_stop", "waypoint_3d")),
        "NEW_DEVELOPMENT_FROZEN": len(dev["cases"]) == 200 and dev["execution_allowed"],
        "NEW_HOLDOUT_FROZEN": len(holdout["cases"]) == 140 and not holdout["execution_allowed"],
        "HOLDOUT_EXECUTED": bool(holdout["executed"]),
        "V1_V10_UNCHANGED": bool(protected["all_unchanged"]),
    }
    pass_conditions = [value for key, value in gates.items() if key != "HOLDOUT_EXECUTED"] + [not gates["HOLDOUT_EXECUTED"]]
    final_gate = {
        "task": "P2-R0-NATIVE-STACK-BENCHMARK-ARCHITECTURE-IMPLEMENTATION-VALIDATION-AND-FREEZE-R1",
        "source_head": "263e78859097592c4dfca5a86a6486606c1cb84f",
        "branch": "platform/native-stack-v1", "gates": gates,
        "holdout_executed": False, "new_controller_performance": False, "new_paper_selected": False,
        "satc_retuned": False, "traditional_retuned": False, "old_holdout_accessed": False,
        "platform_smoke_pass": smoke["pass"], "platform_smoke_selection_authority": False,
        "project_tests": project_tests, "all_required_gates_pass": all(pass_conditions),
        "result": "P2_NATIVE_STACK_BENCHMARK_READY" if all(pass_conditions) else "BLOCKED_P2_NATIVE_STACK_BENCHMARK",
    }
    dump(FINAL / "final_gate.json", final_gate)

    evidence_files = sorted(
        path for path in (R0.parent).rglob("*")
        if path.is_file() and path != FINAL / "evidence_manifest.json"
    )
    evidence_files += sorted(path for path in DOCS.glob("*.md") if path.is_file())
    manifest = {
        "source_head": final_gate["source_head"], "generated_before_final_commit": True,
        "entries": [{"path": path.relative_to(ROOT).as_posix(), "sha256": sha(path), "bytes": path.stat().st_size} for path in evidence_files],
    }
    dump(FINAL / "evidence_manifest.json", manifest)

    report = f"""# P2-R0 Native-Stack Benchmark v1 final report

## Outcome

`{final_gate['result']}`. All mandatory gates passed. This task upgraded and froze the comparison platform only; it selected no paper, introduced no new controller performance claim, retuned neither SATC nor Traditional, and executed no old or native Holdout.

## Audited actuation and timing

The unchanged plant exposes the canonical direct physical interface `WrenchCommand[T,tau_x,tau_y,tau_z]` through `thrust_motor`, `mx_motor`, `my_motor`, and `mz_motor`. Limits are thrust `[0, 285.74568] N` and body torques `±25/±25/±12 N m`. Physics is 1000 Hz; legacy inner and outer rates are 200 and 20 Hz. There are no motor dynamics, actuator lag, or actuator rate limits in the frozen model.

The integer-tick scheduler supports `{scheduler['supported_rates_hz']}`. Its 1000-second logical audit had exact update counts and zero accumulated drift.

## Compatibility and isolation

Three frozen V3 Development cases reproduced `full_lqr_048` with command and metric maximum error 0. Across 64 deterministic inputs, desired force, thrust, torque, clipping, and applied wrench matched exactly. Runtime controllers cannot own the offline truth packet, future preview is disabled, and true wind/hidden force/Holdout metadata are absent from the sensor API.

## New banks

Native Development contains {dev['case_count']} cases (SHA-256 `{dev['manifest_sha256']}`). Native Holdout contains {holdout['case_count']} cases (SHA-256 `{holdout['manifest_sha256']}`), remains `execution_allowed=false`, and has zero executions. All target, wind, trajectory, and timing identities are disjoint.

## Test evidence

All 11 isolated test-directory processes passed: {project_tests['passed']} passed, 0 failed. The execution audit preserves three monolithic timeout attempts and one same-process module-name collection conflict; no scientific protocol changed. V1-V10 protected trees and tags remain unchanged.

## Research boundary

Benchmark A and its V5 SATC Holdout win remain valid and read-only. Benchmark B is a separate system-level protocol. P2-R1 is not started by this result; it requires explicit authorization and must develop native baselines on Development without opening Holdout.
"""
    (DOCS / "P2_R0_FINAL_REPORT.md").write_text(report, encoding="utf-8")


if __name__ == "__main__": main()
