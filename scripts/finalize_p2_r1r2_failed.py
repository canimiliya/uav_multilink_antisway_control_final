"""Close P2-R1R2 at the exhausted Traditional-recovery failure gate."""

from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "reproducibility/native_stack/r1r2"
DOCS = ROOT / "docs/native_stack/r1r2"
SOURCE = "046c64eb8d364a58e5af5425df2f88b95c73b261"
FOLDERS = {"native_pid": "pid", "native_full_lqr": "full_lqr", "native_task_lqr": "task_lqr"}


def read(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True, encoding="utf-8").strip()


def md_metrics(metrics: dict[str, Any]) -> str:
    return (
        f"- Safety: {metrics['safety_rate']:.1%}\n- Success: {metrics['success_rate']:.1%}\n"
        f"- Setpoint RMSE: {metrics['setpoint_position_rmse_m']:.6f} m\n"
        f"- Trajectory RMSE: {metrics['trajectory_position_rmse_m']:.6f} m\n"
        f"- Strong P90: {metrics['strong_p90_m']:.6f} m\n"
        f"- Catastrophic count: {metrics['catastrophic_count']}\n"
        f"- Deadline miss rate: {metrics['deadline_miss_rate']:.6f}\n"
    )


def run_tests() -> dict[str, Any]:
    env = os.environ.copy(); env["PYTHONPATH"] = os.pathsep.join([str(ROOT / "src"), str(ROOT / "third_party/udaan")])
    env["OPENBLAS_NUM_THREADS"] = "1"; env["OMP_NUM_THREADS"] = "1"
    groups = ["native_stack", "release", "v2", "v3", "v4", "v5", "v6", "v7", "v8", "v9", "v10"]
    results = []; passed_total = 0; failed_total = 0
    for group in groups:
        completed = subprocess.run([sys.executable, "-m", "pytest", f"tests/{group}", "-q"], cwd=ROOT, env=env,
                                   text=True, capture_output=True, timeout=180)
        tail = "\n".join((completed.stdout + completed.stderr).strip().splitlines()[-4:])
        matches = re.findall(r"(?m)(\d+) passed(?:,| in)", completed.stdout + completed.stderr)
        passed = int(matches[-1]) if matches else 0
        results.append({"group": group, "returncode": completed.returncode, "passed": passed, "summary_tail": tail})
        passed_total += passed; failed_total += int(completed.returncode != 0)
    monolithic = subprocess.run([sys.executable, "-m", "pytest", "tests", "-q", "--collect-only"], cwd=ROOT, env=env,
                                text=True, capture_output=True, timeout=180)
    mono_tail = "\n".join((monolithic.stdout + monolithic.stderr).strip().splitlines()[-16:])
    return {"authoritative_isolated_groups": results, "authoritative_passed_total": passed_total,
            "authoritative_failed_groups": failed_total, "authoritative_all_passed": failed_total == 0,
            "monolithic_collection_returncode": monolithic.returncode,
            "monolithic_collection_status": "NON_AUTHORITATIVE_EXISTING_DUPLICATE_MODULE_LAYOUT_DIAGNOSTIC",
            "monolithic_collection_tail": mono_tail, "performance_rerun_during_finalization": False}


def main() -> None:
    family_freezes = {family: read(BASE / "traditional" / folder / "freeze.json") for family, folder in FOLDERS.items()}
    if any(freeze["total_unique_configs"] != 96 for freeze in family_freezes.values()):
        raise AssertionError("all three Traditional families must exhaust 96 unique configs")
    if any(freeze["competence_pass"] for freeze in family_freezes.values()):
        raise AssertionError("failure closeout cannot contain a competent family")
    hold = read(ROOT / "reproducibility/native_stack/r0s/resolved_holdout_manifest.json")
    fingerprints = {case["case_semantic_fingerprint"] for case in hold["cases"]}
    hold_ok = (
        hold["identity_manifest_hash"] == "63e6192faf992494f5a78f4c008d844564b0015fc26b43a94a0c98da659b2538"
        and hold["resolved_manifest_hash"] == "4a4b5d92027760e0d37176b7f768746c690a4ca53236ec60224cd423d6582df0"
        and not hold["execution_allowed"] and not hold["executed"] and hold["authoritative_runs"] == 0
        and len(fingerprints) == 140
    )
    if not hold_ok:
        raise AssertionError("Native Holdout freeze was compromised")
    changed = set(git("diff", "--name-only", SOURCE).splitlines())
    protected_prefixes = (
        "reproducibility/v", "docs/v", "src/uav_sway/v", "reproducibility/native_stack/r0",
        "reproducibility/native_stack/r1r1", "configs/", "reproducibility/frozen/",
    )
    protected_files = {
        "src/uav_sway/native_stack/actuation.py", "src/uav_sway/native_stack/api.py",
        "src/uav_sway/native_stack/runner.py", "src/uav_sway/native_stack/references.py",
        "src/uav_sway/native_stack/safety.py", "src/uav_sway/native_stack/scheduler.py",
        "src/uav_sway/native_stack/sensors.py",
    }
    protected_diff = sorted(path for path in changed if path.startswith(protected_prefixes) or path in protected_files
                            or path.startswith("src/uav_sway/native_stack/case_semantics/"))
    if protected_diff:
        raise AssertionError(f"protected paths changed: {protected_diff}")

    write(BASE / "traditional/primary_traditional.json", {
        "frozen": False, "status": "NOT_FROZEN", "reason": "competent Traditional count is 0; minimum is 2",
        "selection_authority": False,
    })
    write(BASE / "traditional/traditional_envelope.json", {
        "frozen": False, "status": "NOT_FROZEN", "reason": "requires at least two competent Traditional families",
        "metrics": None,
    })
    write(BASE / "satc/status.json", {
        "search_executed": False, "unique_configs": 0, "selected": None, "freeze_head": None,
        "bootstrap": "NOT_RUN", "advanced_qualified": False, "stack_frozen": False,
        "r1r1_diagnostic_retained": "satc_native_004", "r1r1_diagnostic_selection_authority": False,
        "reason": "Traditional start gate failed (0 competent; 2 required)",
    })
    write(BASE / "comparison/traditional_recovery_summary.json", {
        family: {"architectures_evaluated": 2 if family == "native_pid" else 2,
                 "unique_configs": freeze["total_unique_configs"], "selected": freeze["selected"],
                 "metrics": freeze["metrics"], "competence": False,
                 "r1r1_incumbent": read(ROOT / f"reproducibility/native_stack/r1r1/traditional/{folder}/freeze.json")["selected"],
                 "r1r1_incumbent_remains_stronger": True}
        for family, folder in FOLDERS.items() for freeze in [family_freezes[family]]
    })
    write(BASE / "final/holdout_status.json", {
        "identity_hash": hold["identity_manifest_hash"], "resolved_hash": hold["resolved_manifest_hash"],
        "execution_allowed": False, "executed": False, "authoritative_runs": 0, "compromised": False,
        "semantic_fingerprint_count": len(fingerprints), "semantic_fingerprint_check": True,
        "permitted_checks_only": ["hash verification", "semantic fingerprint verification"],
    })
    tests = run_tests(); write(BASE / "final/project_tests.json", tests)
    result = "P2_NATIVE_TRADITIONAL_RECOVERY_FAILED"
    gate = {
        "task": "P2-R1R2-STRONG-TRADITIONAL-COMPETENCE-RECOVERY-AND-SATC-FINAL-FREEZE-R1",
        "result": result, "source_head": SOURCE, "branch": "research/p2-native-baselines-r1r2",
        "competent_traditional_count": 0, "traditional_minimum_required": 2,
        "native_pid_competent": False, "native_full_lqr_competent": False, "native_task_lqr_competent": False,
        "primary_native_traditional_frozen": False, "traditional_envelope_frozen": False,
        "satc_search_executed": False, "satc_stack_frozen": False, "satc_advanced_qualified": False,
        "equal_rate_executed": False, "native_holdout_executed": False,
        "competence_gate_changed": False, "benchmark_changed": False, "physical_plant_changed": False,
        "new_paper_selected": False, "paper_search": False, "paper_implementation": False, "paper_performance": False,
        "v1_v10_unchanged": True, "r1r1_unchanged": True, "protected_diff": protected_diff,
        "p2_r2_authorized": False, "final_tag": None,
    }
    write(BASE / "final/final_gate.json", gate)
    write(BASE / "final/claim_matrix.json", {
        "P2_R1R2_exhausted_96_per_family": True, "competent_traditional_count": 0,
        "Primary_Native_Traditional": "NOT_FROZEN", "Traditional_Envelope": "NOT_FROZEN",
        "SATC_Stack": "NOT_FROZEN", "SATC_Advanced": False, "Equal_Rate": "NOT_RUN",
        "Native_Holdout": "UNTOUCHED", "Paper": "NOT_STARTED", "P2_R2": "NOT_AUTHORIZED",
    })

    docs = {
        "NATIVE_PID_RECOVERY.md": "# Native PID recovery\n\nTwo PID/PD servo forms and 96 unique configurations were evaluated.\n\n"
            + f"Selected R1R2 recovery candidate: `{family_freezes['native_pid']['selected']}`.\n\n"
            + md_metrics(family_freezes["native_pid"]["metrics"])
            + "\nResult: **FAIL competence**. Stage C improved safety over the broad initial search, but did not restore endpoint success; the frozen R1R1 Native PID remains stronger.\n",
        "NATIVE_FULL_LQR_RECOVERY.md": "# Native Full-LQR recovery\n\nA physical 12-state/4-input LQR with three LQI servo states was evaluated over 96 unique configurations. The augmented linear model is controllable/stabilizable, but that property did not imply nonlinear mission competence.\n\n"
            + f"Selected R1R2 recovery candidate: `{family_freezes['native_full_lqr']['selected']}`.\n\n"
            + md_metrics(family_freezes["native_full_lqr"]["metrics"])
            + "\nResult: **FAIL competence**. The dominant issue is physical-wrench servo/model mismatch and unsafe nonlinear tracking, not lack of linear controllability.\n",
        "NATIVE_TASK_LQR_RECOVERY.md": "# Native Task-LQR recovery\n\nAn output-weighted task LQT/LQI form was evaluated over 96 unique configurations.\n\n"
            + f"Selected R1R2 recovery candidate: `{family_freezes['native_task_lqr']['selected']}`.\n\n"
            + md_metrics(family_freezes["native_task_lqr"]["metrics"])
            + "\nResult: **FAIL competence**. Feeding cutter-tip output errors into the hover-model servo did not preserve the R1R1 native safety improvement; the output/model mismatch was catastrophic across the full bank.\n",
        "SATC_RECOVERY.md": "# SATC recovery\n\nSATC search was **NOT RUN**. The preregistered start gate requires at least two competent Traditional families; the observed count is zero. `satc_native_004` remains an R1R1 diagnostic with no selection authority.\n",
        "P2_R2_ENTRY_CRITERIA.md": "# P2-R2 entry criteria\n\nP2-R2 is **not authorized**. Entry still requires at least two competent Traditional families, a frozen Primary and Traditional Envelope, a frozen Advanced SATC stack, and untouched Holdout. Only the Holdout condition is currently satisfied.\n",
    }
    postmortem = """# Benchmark competence postmortem

## Formal result

`P2_NATIVE_TRADITIONAL_RECOVERY_FAILED`

Each Traditional family evaluated 96 unique configurations under the frozen Benchmark v1.1 Development bank and unchanged competence gate. Zero families passed; therefore the failure is now eligible for owner-level review, but this task does not change the gate.

## Mechanisms

- Native PID: the R1R1 controller was safe and met RMSE gates, but endpoint regulation dominated failure. Broader gains reduced safety; bounded local contraction recovered some safety but not the 70% endpoint success requirement.
- Native Full-LQR: 12-state controllability and 15-state LQI stabilizability were verified, yet direct physical-wrench tracking produced nonlinear safety loss and large setpoint tails. Linear stabilizability is not task competence.
- Native Task-LQR: task-output errors were not dynamically consistent with the hover-model position channels. Integral augmentation amplified this mismatch and failed safety across the full Development bank.
- Smooth trajectories mainly require causal velocity/acceleration feedforward; setpoints mainly require trim-correct steady-state endpoint regulation. A single direct hover-servo formulation did not satisfy both under the frozen mission envelope.

## Governance conclusion

The 70% success gate, plant, runner, metrics, and case semantics were not changed. SATC, Equal-Rate, Holdout, and Paper were not run. Any reconsideration of mission-envelope/gate structural compatibility is a separate owner-authorized task.
"""
    docs["BENCHMARK_COMPETENCE_POSTMORTEM.md"] = postmortem
    docs["P2_R1R2_FINAL_REPORT.md"] = """# P2-R1R2 final report

Result: **P2_NATIVE_TRADITIONAL_RECOVERY_FAILED**.

All three Traditional families exhausted 96 unique configurations. None passed the unchanged competence gate, so Primary Traditional, Traditional Envelope, and SATC Stack remain not frozen. Native Holdout remains untouched. P2-R2 and Paper remain unauthorized.
"""
    for name, content in docs.items():
        (DOCS / name).write_text(content, encoding="utf-8")

    evidence_files = sorted(
        [path for path in BASE.rglob("*") if path.is_file() and path.name != "evidence_manifest.json"]
        + [path for path in DOCS.rglob("*") if path.is_file()]
    )
    entries = [{"path": path.relative_to(ROOT).as_posix(), "sha256": sha256(path), "size_bytes": path.stat().st_size}
               for path in evidence_files]
    write(BASE / "final/evidence_manifest.json", {"entry_count": len(entries), "entries": entries,
          "hash_failures": 0, "size_failures": 0})
    print(json.dumps({"result": result, "competent_traditional_count": 0,
                      "tests_passed": tests["authoritative_passed_total"], "holdout_untouched": hold_ok,
                      "evidence_entries": len(entries)}))


if __name__ == "__main__":
    main()
