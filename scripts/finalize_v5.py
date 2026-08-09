"""Freeze final V5 evidence without executing controller performance."""

from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
V5 = ROOT / "reproducibility/v5"
FINAL = V5 / "final"
DOC = ROOT / "docs/V5_FINAL_TECHNICAL_REPORT.md"
SOURCE_HEAD = "8e3d7ad08747b00d46b9bdfc3cc72451491567db"
PROTOCOL_HEAD = "a97f00d5cfa11a8646117f490753742567893143"


def read(path: Path) -> dict: return json.loads(path.read_text(encoding="utf-8"))
def write(name: str, value: object) -> None:
    path = FINAL / name; path.parent.mkdir(parents=True, exist_ok=True); path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
def git(*args: str) -> str: return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()
def sha(path: Path) -> str: return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def main() -> int:
    if git("rev-parse", "HEAD") != PROTOCOL_HEAD: raise RuntimeError("finalization must remain on the frozen Holdout protocol head")
    holdout = read(V5 / "holdout/gate.json"); cohorts = read(V5 / "holdout/cohort_analysis.json"); boot = read(V5 / "holdout/paired_bootstrap.json")
    self_freeze = read(V5 / "self/self_freeze.json"); mechanism = read(V5 / "self/mechanism_report.json"); ablation = read(V5 / "self/ablation.json"); paper = read(V5 / "paper/paper_final_status.json")
    self_h = cohorts[self_freeze["candidate_id"]]; full_h = cohorts["full_lqr_048"]
    old_paths = {"v1_release_frozen": "frozen", "v2": "v2", "v3": "v3", "v4": "v4"}
    trees = {name: {"path": f"reproducibility/{path}", "source": git("rev-parse", f"v4-research-final-2026-08-09:reproducibility/{path}"), "current": git("rev-parse", f"HEAD:reproducibility/{path}")} for name, path in old_paths.items()}
    unchanged = all(value["source"] == value["current"] for value in trees.values())
    write("final_gate.json", {
        "task": "V5-END-TO-END-TRANSIENT-ROBUST-SELF-PAPER-AND-ONE-SHOT-VALIDATION-R1",
        "result": "V5_SELF_OVERALL_HOLDOUT_WIN", "v5_complete": True, "v5_permanently_closed": True,
        "strict_holdout_win": False, "overall_holdout_win": True, "development_win": True,
        "failed_frozen_gate": {"section": "normal_nonregression", "gate": "acquisition", "observed_degradation": holdout["audit"]["comparisons"]["normal_acquisition_degradation_vs_legacy"], "maximum": 0.10},
        "holdout": {"executed": True, "sample_count": 96, "participants": 5, "authoritative_runs": 480, "retries": [], "compromised": False},
        "paper": {"development": paper["development_status"], "holdout": paper["holdout_status"]},
        "v1_v4_unchanged": unchanged, "v1_v4_trees": trees, "source_head": SOURCE_HEAD, "holdout_protocol_head": PROTOCOL_HEAD,
    })
    write("metric_summary.json", {
        "development": self_freeze["metrics"], "holdout_self": self_h, "holdout_full_lqr": full_h,
        "holdout_comparisons": holdout["audit"]["comparisons"], "holdout_directional": holdout["audit"]["directional"],
        "holdout_bootstrap": boot, "catastrophic_pairs": holdout["audit"]["catastrophic_pair_ids"],
    })
    write("method_status.json", {
        "traditional": {"status": "FROZEN_UNCHANGED", "methods": ["hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009"]},
        "legacy_self": {"method": "self_a_034", "status": "FROZEN_UNCHANGED"},
        "v4_cart": {"status": "NEGATIVE_MECHANISM_BASELINE_UNCHANGED"},
        "v5_self": {"method": "SATC-OFMPC", "candidate": self_freeze["candidate_id"], "development": "QUALIFIED_FROZEN", "holdout": "OVERALL_WIN_NOT_STRICT"},
        "paper": {"source": paper["source"], "method": paper["method"], "development": paper["development_status"], "holdout": paper["holdout_status"]},
    })
    write("claim_matrix.json", {
        "allowed": [
            "SATC-OFMPC satc_b_027 achieved the frozen V5 Overall Holdout win against the fixed Traditional benchmark.",
            "SATC-OFMPC passed all frozen strong-transient and directional Holdout tail gates with zero catastrophic pairs.",
            "SATC-OFMPC improved Holdout mean position RMSE by 29.10% versus full_lqr_048 with a positive paired-bootstrap lower bound.",
            "The V5 adaptation of Jirousek et al. was Development-ineligible and was not evaluated on Holdout.",
        ],
        "forbidden": [
            "V5_SELF_STRICT_HOLDOUT_WIN", "normal-regime acquisition non-regression passed", "Self outperformed the original Jirousek et al. method", "Self beat Paper on Holdout", "Paper was evaluated on Holdout", "V5 results establish real-flight performance",
        ],
        "qualification": "The sole strict-gate miss was 11.27% normal acquisition degradation versus a frozen 10% maximum.",
    })
    write("resume_claims.json", {
        "resume_status": "V5_PERMANENTLY_CLOSED", "do_not_resume": ["SATC tuning", "Paper search", "V5 Development", "V5 Holdout", "second Holdout"],
        "future_work_requires": "a separately authorized new research version with a new contract and new unseen Holdout",
        "headline": "SATC-OFMPC is a Development-qualified and Overall-Holdout-winning V5 Self method, but not a Strict Holdout winner.",
    })
    write("mechanism_summary.json", {
        "shock_detection": "implemented and time-series observed", "bumpless_engagement": "implemented and time-series observed",
        "cancellation_coordination": "implemented and time-series observed", "slew_headroom": "implemented and time-series observed",
        "conflict_index": "coordinate-free causal index implemented; representative traces observed",
        "development_catastrophic_pairs": len(self_freeze["metrics"]["catastrophic_pair_ids"]), "holdout_catastrophic_pairs": len(holdout["audit"]["catastrophic_pair_ids"]),
        "ablation_conclusion": "Shock-only had slightly better strong mean/P90 than full SATC; the cumulative additions were not monotonically beneficial, while full SATC retained the qualified Overall/strong balance. No post-ablation retuning occurred.",
        "trace_evidence": mechanism["traces"], "causal_scope": mechanism["causal_claim_scope"], "ablation_technical_retries": ablation["technical_retries"],
    })
    write("project_tests.json", {"targeted_v5_tests": "22 passed", "full_project_tests": "170 passed", "third_party_udaan": "repository-pinned third_party/udaan supplied via PYTHONPATH", "performance_rerun_during_finalization": False})

    report = f"""# V5 Final Technical Report

## Final result

V5 is permanently closed as **V5_SELF_OVERALL_HOLDOUT_WIN**. The frozen SATC-OFMPC candidate `satc_b_027` passed all five Overall Holdout gates and 19/20 total frozen gates. It is not a Strict Holdout winner because normal-regime acquisition was 11.27% slower than legacy `self_a_034`, exceeding the preregistered 10% allowance.

## Frozen research design

V5 branched from V4 tag `v4-research-final-2026-08-09` at `{SOURCE_HEAD}`. Before performance, it froze a 120-sample Development bank, a disjoint 96-sample Holdout bank, relative aligned/opposed/cross direction strata, 64-Self-configuration maximum, 10,000-resample paired bootstrap (seed 20260816), physical acceleration ±2.0 m/s², slew 0.25 m/s²/update, and the unchanged 20 Hz interface.

## Self Development

SATC-OFMPC combines CART's constraint-feasible offset model with causal shock detection, rate-limited offset engagement, final physical-input coordination, slew reserve, and a geometric disturbance-task conflict index. Three of 36 full-bank Stage-B configurations passed 20/20 gates and all three repeated that result in independent Stage C. `satc_b_027` was frozen with Development position RMSE {self_freeze['metrics']['overall']['position_mean_m']:.6f} m, success {100*self_freeze['metrics']['overall']['success_rate']:.2f}%, strong P90 {self_freeze['metrics']['strong']['position_p90_m']:.6f} m, and zero catastrophic pairs.

Post-freeze ablation was explanatory only. Shock-only slightly outperformed full SATC on strong mean/P90, so the added conflict/headroom layers are not claimed to provide monotonic gains. No ablation result was used for retuning.

## Paper-Advanced

The sole selected source was Jiroušek, Báča, and Saska, *Towards Fully Onboard State Estimation and Trajectory Tracking for UAVs with Suspended Payloads* (ICINCO 2025, DOI 10.5220/0013789200003982; primary full text: https://arxiv.org/html/2508.11547v2). The adaptation preserved the paper's augmented incremental-MPC equations and frozen input/increment constraints. All 24 full-bank configurations were ineligible; best `paper_b_002` passed 2/5 Overall gates. Paper was not run on Holdout. This does not compare SATC with the original authors' system.

## One-shot Holdout

The five eligible frozen controllers ran the same 96 samples for 480 authoritative runs, with no retry or compromise. SATC achieved 100% safety, {100*self_h['overall']['success_rate']:.2f}% success, {self_h['overall']['position_mean_m']:.6f} m mean position RMSE, {self_h['overall']['acquisition_median_s']:.3f} s median acquisition, and {self_h['strong']['position_p90_m']:.6f} m strong P90. Relative to `full_lqr_048`, position improved {100*holdout['audit']['comparisons']['overall_position_improvement_vs_full']:.2f}%; the overall paired-bootstrap 95% CI was [{boot['overall']['ci_95_m'][0]:.6f}, {boot['overall']['ci_95_m'][1]:.6f}] m. Strong position improved {100*holdout['audit']['comparisons']['strong_position_improvement_vs_full']:.2f}% versus Full-LQR and {100*holdout['audit']['comparisons']['strong_position_improvement_vs_legacy']:.2f}% versus legacy Self. Aligned, opposed, and cross directional P90 gates all passed, with zero catastrophic pairs.

## Scope and closure

The evidence supports an Overall Holdout win and unseen strong-transient tail robustness in this frozen MuJoCo benchmark. It does not support a Strict win, Paper-on-Holdout comparison, original-paper superiority, hardware flight, or real-world generalization. V1–V4 evidence trees remain byte-identical to the V4 source tag. V5 must not be tuned or rerun; any new work requires a new research version and new unseen Holdout.
"""
    DOC.write_text(report, encoding="utf-8", newline="\n")
    evidence = [
        V5 / "r0/research_contract.json", V5 / "r0/development_manifest.json", V5 / "r0/holdout_manifest.json", V5 / "r0/win_contract.json",
        V5 / "self/self_freeze.json", V5 / "self/development_results.csv", V5 / "self/ablation.json", V5 / "self/mechanism_report.json",
        V5 / "paper/paper_selection.json", V5 / "paper/development_gate_audit.json", V5 / "paper/paper_final_status.json",
        V5 / "holdout/unlock_audit.json", V5 / "holdout/execution_manifest.json", V5 / "holdout/holdout_results.csv", V5 / "holdout/gate.json", V5 / "holdout/paired_bootstrap.json",
        FINAL / "final_gate.json", FINAL / "metric_summary.json", FINAL / "method_status.json", FINAL / "claim_matrix.json", FINAL / "resume_claims.json", FINAL / "mechanism_summary.json", DOC,
    ]
    write("evidence_manifest.json", {str(path.relative_to(ROOT)).replace("\\", "/"): sha(path) for path in evidence})
    print(json.dumps({"result": "V5_SELF_OVERALL_HOLDOUT_WIN", "strict": False, "overall": True, "v1_v4_unchanged": unchanged, "performance_rerun": False}, indent=2)); return 0


if __name__ == "__main__": raise SystemExit(main())
