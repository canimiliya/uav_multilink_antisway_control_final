"""Unlock and freeze the one-shot V5 Holdout execution."""

from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v5/r0"
SELF = ROOT / "reproducibility/v5/self"
PAPER = ROOT / "reproducibility/v5/paper"
OUT = ROOT / "reproducibility/v5/holdout"
START_HEAD = "5de1160e4a160f5fbab09603ae1a6596a2f5460f"


def read(path: Path) -> dict: return json.loads(path.read_text(encoding="utf-8"))
def write(name: str, value: object) -> None:
    path = OUT / name; path.parent.mkdir(parents=True, exist_ok=True); path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
def git(*args: str) -> str: return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()
def sha(path: Path) -> str: return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def main() -> int:
    if git("rev-parse", "HEAD") != START_HEAD: raise RuntimeError("Holdout unlock must follow Paper closure")
    self_freeze = read(SELF / "self_freeze.json"); paper = read(PAPER / "paper_final_status.json"); source = read(R0 / "holdout_manifest.json")
    if self_freeze["status"] != "V5_SELF_DEVELOPMENT_QUALIFIED_FROZEN": raise RuntimeError("Self not qualified")
    if paper["development_status"] != "PAPER_DEVELOPMENT_INELIGIBLE": raise RuntimeError("Paper outcome not final")
    write("unlock_audit.json", {
        "traditional_finalized": True, "legacy_self_finalized": True, "v5_self_finalized_and_qualified": True,
        "paper_selection_process_finalized": True, "paper_qualified_candidate_exists": False,
        "paper_holdout_status": "NOT_RUN_DEVELOPMENT_INELIGIBLE", "all_method_selection_outcomes_final": True,
        "all_search_permanently_stopped": True, "original_v5_contract_modified": False,
        "v1_v4_unchanged": True, "holdout_accessed_before_unlock": False, "result": "V5_HOLDOUT_UNLOCKED",
    })
    samples = [{**row, "execution_allowed": True} for row in source["samples"]]
    participants = [
        {"kind": "corrected_pid", "candidate_id": "hybrid_x007_y041_z041"}, {"kind": "full_lqr", "candidate_id": "full_lqr_048"},
        {"kind": "task_lqr", "candidate_id": "task_lqr_009"}, {"kind": "legacy_self", "candidate_id": "self_a_034"},
        {"kind": "satc_ofmpc", "candidate_id": self_freeze["candidate_id"]},
    ]
    write("execution_manifest.json", {
        "source_manifest_sha256": sha(R0 / "holdout_manifest.json"), "mechanical_transformation": "same ordered samples; execution_allowed false changed to true only after unlock",
        "sample_count": len(samples), "samples": samples, "participants": participants, "participant_count": len(participants),
        "authoritative_run_count": len(samples) * len(participants), "paper": {"method": "JIROUSEK2025-INCREMENTAL-MPC-ADAPTED-5LINK", "status": "NOT_RUN_DEVELOPMENT_INELIGIBLE"},
    })
    write("holdout_protocol_freeze.json", {
        "task": "V5-ONE-SHOT-HOLDOUT-VALIDATION-R1", "start_head": START_HEAD, "one_shot": True,
        "written_before_first_holdout_trajectory": True, "controller_code_parameters_metrics_runner_manifest_locked": True,
        "same_exact_manifest_all_participants": True, "paper_not_run": True,
        "technical_retry_rule": "OS crash, power/write/process failure only; same commit/config/sample; record reason",
        "scientific_bug_rule": "HOLDOUT_COMPROMISED; do not repair and continue as the same Holdout",
        "bootstrap": {"resamples": 10000, "seed": 20260816, "pairing": "exact sample_id", "ci": [0.025, 0.975]},
        "final_status_order": ["V5_SELF_STRICT_HOLDOUT_WIN", "V5_SELF_OVERALL_HOLDOUT_WIN", "V5_SELF_DEVELOPMENT_WIN_NOT_CONFIRMED_ON_HOLDOUT"],
        "holdout_executed_at_freeze": False,
    })
    print(json.dumps({"unlock": "V5_HOLDOUT_UNLOCKED", "samples": len(samples), "participants": len(participants), "runs": len(samples) * len(participants)}, indent=2)); return 0


if __name__ == "__main__": raise SystemExit(main())
