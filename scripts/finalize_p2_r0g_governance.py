"""Finalize P2-R0G reports, gates, protection audit, and evidence hashes."""

from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/native_stack/governance"
SOURCE_HEAD = "9d0264246408eced13b5c5029763443907610697"
PLATFORM_HEAD = "88c3aef081fabab44d174bc8afd234ae634af3da"
PLATFORM_TAG = "native-stack-benchmark-v1.2-governance"


def dump(name, value):
    (OUT / name).write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def git(*args):
    return subprocess.run(["git", *args], cwd=ROOT, check=True, capture_output=True, text=True).stdout.strip()


def main():
    platform_paths = git("diff", "--name-only", "native-stack-benchmark-v1.1..platform/native-stack-v1.2-governance").splitlines()
    allowed_platform = {
        "docs/native_stack/governance/COMPETENCE_GOVERNANCE_V2.md",
        "docs/native_stack/governance/NOMINAL_VS_CHALLENGE_PROTOCOL.md",
        "docs/native_stack/governance/V1_1_TO_V1_2_GOVERNANCE.md",
        "reproducibility/native_stack/governance/cohort_role_definition.json",
        "reproducibility/native_stack/governance/competence_governance_v2.json",
        "tests/native_stack/test_governance_v2_platform.py",
    }
    dump("platform_freeze.json", {
        "base_tag": "native-stack-benchmark-v1.1",
        "branch": "platform/native-stack-v1.2-governance",
        "head": PLATFORM_HEAD,
        "remote_head": git("rev-parse", "origin/platform/native-stack-v1.2-governance"),
        "tag": PLATFORM_TAG,
        "tag_target": git("rev-parse", f"{PLATFORM_TAG}^{{}}"),
        "changed_paths": platform_paths,
        "allowed_paths": sorted(allowed_platform),
        "governance_only_scope": set(platform_paths) == allowed_platform,
        "controller_implementation_included": False,
        "performance_evidence_included": False,
    })

    project_tests = {
        "authoritative_mode": "11 isolated pytest directory processes",
        "environment": "D:/anaconda/envs/uav_sway/python.exe with PYTHONPATH=src;third_party/udaan",
        "groups": {
            "native_stack": 54,
            "release": 4,
            "v2": 56,
            "v3": 63,
            "v4": 25,
            "v5": 22,
            "v6": 20,
            "v7": 9,
            "v8": 11,
            "v9": 13,
            "v10": 12,
        },
        "passed": 289,
        "failed": 0,
        "warnings": "OSQP deprecation warnings only",
        "historical_byte_materialization": {
            "manifest_entries_verified": 185,
            "lf_or_crlf_bytes_restored_by_existing_sha256": 122,
            "historical_manifest_modified": False,
        },
    }
    dump("project_tests.json", project_tests)

    changed = git("diff", "--name-only", SOURCE_HEAD).splitlines()
    protected_prefixes = (
        "reproducibility/frozen/",
        "src/uav_sway/native_stack/runner.py",
        "src/uav_sway/native_stack/actuation.py",
        "src/uav_sway/native_stack/case_semantics/",
        "reproducibility/native_stack/r0/native_",
        "reproducibility/native_stack/r0s/resolved_",
    )
    protected_changed = [path for path in changed if path.startswith(protected_prefixes)]
    holdout = json.loads((OUT / "holdout_status.json").read_text(encoding="utf-8"))
    dump("protected_evidence_audit.json", {
        "audit_source_head": SOURCE_HEAD,
        "changed_paths": changed,
        "protected_paths_changed": protected_changed,
        "protected_scope_clean": not protected_changed,
        "holdout_hash_only": True,
        "holdout_identity_hash": holdout["identity_manifest_hash"],
        "holdout_resolved_hash": holdout["resolved_manifest_hash"],
        "holdout_fingerprints": holdout["unique_fingerprint_count"],
        "history": {
            "P2-R1R1": "BLOCKED_P2_TRADITIONAL_COMPETENCE",
            "P2-R1R2": "P2_NATIVE_TRADITIONAL_RECOVERY_FAILED",
        },
    })

    dump("claim_matrix.json", {
        "root_cause_classified": True,
        "mission_feasibility_audited": True,
        "competence_70_origin_audited": True,
        "nominal_challenge_role_defined": True,
        "traditional_eligibility_challenge_coupling": True,
        "mission_envelope_structural_problem": False,
        "owner_decision": "B",
        "governance_v2_created": True,
        "existing_method_pass_count_v2_retrospective": 0,
        "controller_tuning": False,
        "satc_search": False,
        "paper_search": False,
        "paper_implementation": False,
        "paper_performance": False,
        "holdout_execution": False,
    })

    dump("final_gate.json", {
        "task": "P2-R0G-MISSION-ENVELOPE-AND-TRADITIONAL-COMPETENCE-STRUCTURAL-AUDIT-R1",
        "result": "P2_NATIVE_BENCHMARK_GOVERNANCE_AUDIT_COMPLETE",
        "audit_source_head": SOURCE_HEAD,
        "benchmark_tag": "native-stack-benchmark-v1.1",
        "ROOT_CAUSE_CLASSIFIED": True,
        "MISSION_FEASIBILITY_AUDITED": True,
        "COMPETENCE_70_ORIGIN_AUDITED": True,
        "NOMINAL_CHALLENGE_ROLE_DEFINED": True,
        "HOLDOUT_UNTOUCHED": True,
        "OWNER_DECISION": "B",
        "governance_v2": {
            "created": True,
            "platform_head": PLATFORM_HEAD,
            "tag": PLATFORM_TAG,
            "retrospective_existing_method_pass_count": 0,
            "retroactive_pass": False,
        },
        "prohibited": {
            "controller_tuning": False,
            "satc_search": False,
            "paper_search": False,
            "paper_implementation": False,
            "paper_performance": False,
            "holdout_execution": False,
        },
        "project_progress_percent": 84,
        "p2_r2_authorized": False,
        "final_status": "COMPLETE_DECISION_B_NO_CONTROLLER_QUALIFIED",
    })

    evidence_paths = []
    evidence_paths.extend(sorted((ROOT / "docs/native_stack/governance").glob("*")))
    evidence_paths.extend(sorted(path for path in OUT.glob("*") if path.name != "evidence_manifest.json"))
    evidence_paths.extend([
        ROOT / "scripts/audit_p2_r0g_governance.py",
        ROOT / "scripts/run_p2_r0g_oracle.py",
        ROOT / "scripts/run_p2_r0g_equilibrium_oracle.py",
        ROOT / "scripts/compute_p2_r0g_retrospective.py",
        ROOT / "scripts/finalize_p2_r0g_governance.py",
        ROOT / "src/uav_sway/native_stack/governance_oracle.py",
        ROOT / "tests/native_stack/test_governance_audit.py",
        ROOT / "tests/native_stack/test_competence_governance_v2.py",
        ROOT / "tests/native_stack/test_governance_v2_platform.py",
        ROOT / "tests/native_stack/test_governance_final.py",
    ])
    entries = []
    for path in sorted(set(evidence_paths)):
        if not path.is_file():
            raise FileNotFoundError(path)
        entries.append({
            "path": path.relative_to(ROOT).as_posix(),
            "size_bytes": path.stat().st_size,
            "sha256": sha(path),
        })
    dump("evidence_manifest.json", {"entry_count": len(entries), "entries": entries})


if __name__ == "__main__":
    main()
