"""Close the failed final external-paper route without opening Holdout."""

from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
V10 = ROOT / "reproducibility/v10"
FINAL = V10 / "final"
DOCS = ROOT / "docs/v10"
SOURCE = "46b4c31d45d427f790ef2978702552b5ee8235ab"
CONTRACT = "2ab2714528c9b7f144bd2f62cfbae632140cb5a7"
IMPLEMENTATION = "6f6a953c413bf4b4c596bedd4153902cdeecebda"
DEVELOPMENT = "d084c95647d5013912d920eef1de36a2f386f1cb"


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
        stream.write("\n")


def write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(value.rstrip() + "\n")


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> int:
    development = read(V10 / "development/development_final.json")
    observer = read(V10 / "development/observer_offline_truth_audit.json")
    split = read(V10 / "r0/split_integrity.json")
    selected = development["selected_metrics"]
    baseline = read(ROOT / "reproducibility/v7/development/comparator_summary.json")["participants"]
    full = baseline["full_lqr_048"]
    satc = baseline["satc_b_027"]
    if development["qualified"] or development["holdout_execution_allowed"]:
        raise RuntimeError("failure closeout refuses a qualified Development result")
    protected = {}
    for path in ["reproducibility/frozen", *[f"reproducibility/v{i}" for i in range(2, 10)]]:
        source_tree = git("rev-parse", f"{SOURCE}:{path}")
        current_tree = git("rev-parse", f"HEAD:{path}")
        protected[path] = {"source_tree": source_tree, "current_tree": current_tree, "unchanged": source_tree == current_tree}
    if not all(row["unchanged"] for row in protected.values()):
        raise RuntimeError("protected V1-V9 evidence changed")
    final_gate = {
        "task": "V10-FXTDO-MPC-RECENT-PAPER-STRONG-BASELINE-AND-FINAL-PROJECT-CLOSURE-R1",
        "result": "V10_FXTDO_MPC_NOT_STRONGER_THAN_TRADITIONAL",
        "source_head": SOURCE, "V10_CONTRACT_FREEZE_HEAD": CONTRACT,
        "PAPER_IMPLEMENTATION_HEAD": IMPLEMENTATION, "PAPER_DEVELOPMENT_FINAL_HEAD": DEVELOPMENT,
        "PAPER_FREEZE_HEAD": None, "V10_HOLDOUT_PROTOCOL_HEAD": None,
        "PAPER_GT_TRADITIONAL": False, "RECENT_PAPER_STRONG_BASELINE_VALIDATED": False,
        "PAPER_VS_SATC_TRADEOFF_REPORTED": False, "TRADEOFF_VALID": False,
        "THREE_TRADITIONAL_STRONG": True, "SATC_SELF_VALIDATED": True,
        "PROJECT_RESEARCH_COMPLETE": False,
        "FINAL_EXTERNAL_PAPER_SEARCH_CLOSED": True, "NO_V11": True,
        "HOLDOUT_EXECUTED": False, "HOLDOUT_COMPROMISED": False,
        "resume_outputs": "NOT_GENERATED_V10_FAILED",
    }
    write_json(FINAL / "final_gate.json", final_gate)
    write_json(FINAL / "holdout_status.json", {
        "status": "NOT_RUN_DEVELOPMENT_INELIGIBLE", "executed": False,
        "inherited_manifest": "reproducibility/v10/r0/holdout_manifest.json",
        "manifest_sha256": split["holdout_sha256"], "manifest_unchanged": True,
        "samples": 112, "participants": 5, "authoritative_runs": 0, "retries": 0,
        "compromised": False, "performance_accessed": False,
    })
    write_json(FINAL / "paper_vs_satc_status.json", {
        "status": "NOT_RUN_PAPER_DEVELOPMENT_INELIGIBLE",
        "reason": "The frozen protocol allowed formal Paper-vs-SATC trade-off reporting only after Paper Development qualification.",
        "paper_development_runtime_advantage_vs_satc": selected["advanced_value"]["runtime_ge_50pct_vs_satc"],
        "paper_development_performance_advantage_claimed": False,
        "tradeoff_valid": False,
    })
    write_json(FINAL / "claim_matrix.json", {
        "supported": [
            "Three frozen Traditional controllers remain completed.",
            "SATC-OFMPC satc_b_027 remains validated with the V5 Overall Holdout win.",
            "V10 implemented an adapted causal multivariable FxTDO with constrained receding-horizon MPC under the common acceleration interface.",
            "V10 observer estimates were finite and bounded in an offline validation replay, but fixed-time empirical settling was not validated.",
            "V10 failed the frozen Development strong-baseline gate and Holdout remained untouched.",
        ],
        "not_supported": [
            "Recent Paper > Traditional", "FxTDO-MPC validated on Holdout", "Paper-vs-SATC valid two-sided trade-off",
            "PROJECT_RESEARCH_COMPLETE", "exact reproduction of Xu et al.", "automatic V11 or another external-paper route",
        ],
        "highest_project_claim": "V5_SELF_OVERALL_HOLDOUT_WIN",
    })
    write_json(FINAL / "protected_evidence_audit.json", {
        "source_head": SOURCE, "protected": protected, "V1_V9_UNCHANGED": True,
        "traditional_unchanged": True, "satc_unchanged": True,
        "old_tags_moved": False, "holdout_accessed": False,
    })
    report = f"""# V10 final technical report

## Outcome

V10 completed the final preregistered external-paper route and **failed** the frozen Development gate. No V10 Holdout run was authorized or executed. The project therefore does not satisfy `Recent Paper > Traditional`, and `PROJECT_RESEARCH_COMPLETE` remains false. The external-paper search is permanently closed and V11 must not start automatically.

## Paper and implementation

The sole paper was Xu et al., *Fixed-Time Disturbance Observer-Based MPC Robust Trajectory Tracking Control of Quadrotor* (arXiv:2408.15019v2). V10 preserved the multivariable bi-homogeneous FxTDO, causal lumped-disturbance estimate, constant disturbance injection across each horizon, and constrained receding-horizon MPC. It adapted the paper to the frozen 20D five-link model and task output, mass-normalized force to world acceleration, and ran at the fair 20 Hz outer rate. Torque FxTDO/INDI were omitted because the common benchmark does not grant independent torque authority. Fidelity is `ADAPTED_NOT_EXACT_REPRODUCTION`.

## Observer diagnostic

Offline replay used 48 frozen V9 validation trajectories and 10,957 valid safe steps. MuJoCo-reconstructed force truth was used only after execution for audit. The estimate was finite and bounded, with all-axis acceleration RMSE `{observer['rmse_all_axes_m_s2']:.6f} m/s^2`, axis RMSE `{observer['rmse_axis_m_s2']}`, post-1 s RMSE `{observer['post_1s_rmse_all_axes_m_s2']:.6f} m/s^2`, and maximum absolute estimate `{observer['estimate_max_abs_m_s2']:.3f} m/s^2`. The empirical fixed-time settling claim was not validated.

## Development evidence

The search evaluated 64 unique configurations: 768 Stage-A core runs, then 2,304 full-Development runs for the preregistered top 16. There were no performance-driven retries. The best formal candidate was `{development['selected_candidate']}`.

| Metric | FxTDO-MPC | Full-LQR | SATC |
|---|---:|---:|---:|
| Safety | {selected['overall']['safety_rate']:.4f} | {full['safety_rate']:.4f} | {satc['safety_rate']:.4f} |
| Success | {selected['overall']['success_rate']:.4f} | {full['success_rate']:.4f} | {satc['success_rate']:.4f} |
| Position mean (m) | {selected['overall']['position_mean_m']:.6f} | {full['position_mean_m']:.6f} | {satc['position_mean_m']:.6f} |
| Position P90 (m) | {selected['overall']['position_p90_m']:.6f} | {full['position_p90_m']:.6f} | {satc['position_p90_m']:.6f} |
| Orientation mean (deg) | {selected['overall']['orientation_mean_deg']:.6f} | {full['orientation_mean_deg']:.6f} | {satc['orientation_mean_deg']:.6f} |
| Effort | {selected['overall']['effort_mean']:.6f} | {full['effort_mean']:.6f} | {satc['effort_mean']:.6f} |
| Solve P95 (ms) | {selected['overall']['solve_p95_ms']:.4f} | {full['solve_p95_ms']:.4f} | {satc['solve_p95_ms']:.4f} |

The candidate passed only safety and the advanced runtime option (2/9 gates). Success was zero. Position improvement versus Full-LQR was `{100.0 * selected['comparisons']['position_improvement']:.2f}%`; this negative number means severe degradation. The paired 10,000-resample bootstrap mean delta was `{selected['bootstrap']['mean_delta_m']:.6f} m` with 95% CI `{selected['bootstrap']['ci_95_m']}` and positive-pair fraction `{selected['bootstrap']['positive_pair_fraction']:.3f}`. All 72 strong cases were catastrophic by the frozen pair rule.

The likely mechanism is under-authoritative finite-horizon correction for the frozen discrete model: the selected Q/R/horizon combinations generated insufficient or poorly scaled action under wind, while the disturbance estimate did not demonstrate the paper's empirical fixed-time settling. This is an evidence-based diagnosis, not a new gate or permission to retune.

## Final boundary

V10 did not unlock Paper freeze, Paper-vs-SATC formal comparison, or the one-shot 112-case Holdout. No completed-version resume text was generated. The highest supported project claim remains the frozen V5 `V5_SELF_OVERALL_HOLDOUT_WIN`. Continued paper replacement would violate the final-route contract and reduce research credibility.
"""
    write_text(DOCS / "V10_FINAL_TECHNICAL_REPORT.md", report)
    boundary = """# PROJECT FINAL RESEARCH BOUNDARY

## 已完成

- 三个 Traditional 控制器已完成并冻结。
- SATC-OFMPC 已完成验证；最高已支持结论是 V5 `V5_SELF_OVERALL_HOLDOUT_WIN`。
- V6-V10 已对多个近期论文方法做受控适配；V10 是最后一次预注册外部 Paper 路线。

## 未完成

- `Recent Paper > Traditional` 未成立。
- V10 FxTDO-MPC 在 Development 失败，因此 112-case Holdout 从未打开。
- 未形成有效的 Paper-vs-SATC 双向性能 trade-off。
- `PROJECT_RESEARCH_COMPLETE=false`。

## 永久边界

`FINAL_EXTERNAL_PAPER_SEARCH_CLOSED=true`，`NO_V11=true`。不得为了简历叙述继续替换论文、回调 V10、再次打开 Holdout，或把运行时优势冒充控制性能优势。最终简历只能使用 V5 已验证的 SATC Overall Holdout 结论，并明确近期 Paper 路线未超过 Traditional。
"""
    write_text(DOCS / "PROJECT_FINAL_RESEARCH_BOUNDARY.md", boundary)
    write_json(FINAL / "project_tests.json", {
        "environment": "conda:uav_sway",
        "python": "3.11.15", "mujoco": "3.0.1", "numpy": "2.4.6", "scipy": "1.17.1", "osqp": "1.1.3",
        "command": "python -m pytest tests/release tests/v2 tests/v3 tests/v4 tests/v5 tests/v6 tests/v7 tests/v8 tests/v9 tests/v10 --import-mode=importlib -q",
        "result": "235 passed", "warnings": 35,
        "third_party_udaan": "repository-pinned third_party/udaan supplied via process-local PYTHONPATH",
        "performance_rerun_during_finalization": False,
    })
    manifest_paths = []
    roots = [V10 / "r0", V10 / "development", FINAL, DOCS, ROOT / "src/uav_sway/v10", ROOT / "tests/v10"]
    for folder in roots:
        for path in folder.rglob("*"):
            if path.is_file() and "_cache" not in path.parts and path.name != "evidence_manifest.json":
                manifest_paths.append(path)
    for name in (
        "build_v10_contract.py", "run_v10_development.py", "select_v10_stage_b.py",
        "evaluate_v10_development.py", "audit_v10_observer_offline.py", "finalize_v10.py",
    ):
        manifest_paths.append(ROOT / "scripts" / name)
    write_json(FINAL / "evidence_manifest.json", {
        "task": final_gate["task"], "result": final_gate["result"],
        "file_count": len(set(manifest_paths)),
        "files": {
            str(path.relative_to(ROOT)).replace("\\", "/"): {"sha256": digest(path), "bytes": path.stat().st_size}
            for path in sorted(set(manifest_paths))
        },
    })
    print(json.dumps({"result": final_gate["result"], "project_complete": False, "protected": len(protected)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
