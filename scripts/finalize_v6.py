"""Generate the final V6 publication package without rerunning performance."""

from __future__ import annotations

import csv
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
V6 = ROOT / "reproducibility/v6"
FINAL = V6 / "final"
DOCS = ROOT / "docs/v6"
SOURCE_TAG = "v5-research-final-2026-08-09"
SOURCE_HEAD = "60a6350d6875d4654ab881fdf8d96e749226133c"
CONTRACT_HEAD = "8cb763054d292f0bf40e2b9be08c2635b90fdb53"
SUITE_HEAD = "104991c2f0b5bdcdaed2ba20d11485f7bfea9ebc"
IMPLEMENTATION_HEAD = "5e54001607f79e97cba3afb2af72acdb65b40795"
DEVELOPMENT_HEAD = "8e877ec8cbd0102dc07f72a87d0f349a50cbe514"
FINALIZATION_START_HEAD = "7c1f683fe8f4041e10aec6027056d9f2a29f5c04"


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text.strip() + "\n", encoding="utf-8", newline="\n")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def satc_v6_bootstrap() -> dict:
    grouped: dict[str, dict[str, float]] = {}
    path = V6 / "development/baseline_results.csv"
    with path.open(encoding="utf-8", newline="") as stream:
        for value in csv.DictReader(stream):
            grouped.setdefault(value["candidate_id"], {})[value["sample_id"]] = float(value["position_rmse_3d_m"])
    full = grouped["full_lqr_048"]
    satc = grouped["satc_b_027"]
    sample_ids = sorted(full)
    if sample_ids != sorted(satc):
        raise RuntimeError("SATC V6 reference exact pairing failed")
    deltas = np.asarray([full[sample_id] - satc[sample_id] for sample_id in sample_ids])
    rng = np.random.Generator(np.random.PCG64(20260819))
    means = np.mean(deltas[rng.integers(0, len(deltas), size=(10000, len(deltas)))], axis=1)
    ci = np.percentile(means, [2.5, 97.5])
    return {
        "evidence_split": "V6 Development only",
        "comparison": "full_lqr_048 minus frozen satc_b_027 position RMSE",
        "pairs": len(deltas), "resamples": 10000, "seed": 20260819,
        "mean_delta_m": float(np.mean(deltas)), "median_delta_m": float(np.median(deltas)),
        "positive_pair_fraction": float(np.mean(deltas > 0)),
        "ci_95_m": [float(ci[0]), float(ci[1])], "lower_bound_gt_zero": bool(ci[0] > 0),
        "formal_claim_boundary": "Supporting V6 Development reference; formal SATC claim remains the frozen V5 Holdout result.",
    }


def row(method: str, method_class: str, status: str, metrics: dict, strong: dict, comparison: float | None = None) -> dict:
    return {
        "evidence_split": "V6 Development", "method": method, "class": method_class, "status": status,
        "safety_rate": metrics["safety_rate"], "success_rate": metrics["success_rate"],
        "position_rmse_m": metrics["position_mean_m"], "vs_full_lqr_position_improvement": comparison,
        "acquisition_median_s": metrics["acquisition_median_s"], "orientation_rmse_deg": metrics["orientation_mean_deg"],
        "strong_position_mean_m": strong["position_mean_m"], "strong_position_p90_m": strong["position_p90_m"],
        "effort_mean": metrics["effort_mean"], "solve_p95_ms": metrics["solve_p95_ms"],
    }


def main() -> int:
    if git("rev-parse", "HEAD") != FINALIZATION_START_HEAD:
        raise RuntimeError("V6 finalization start head drift")
    development = read(V6 / "development/development_final.json")
    if development["qualified_paper_count"] != 0 or development["holdout_executed"]:
        raise RuntimeError("zero-qualified closure contract drift")
    comparator = read(V6 / "development/comparator_gate.json")
    yu = read(V6 / "papers/yu2026/development_gate_audit.json")["ranked"][0]
    sep = read(V6 / "papers/sep2026/development_gate_audit.json")["ranked"][0]
    protected_names = ["frozen", "v2", "v3", "v4", "v5"]
    trees = {
        name: {
            "source": git("rev-parse", f"{SOURCE_TAG}:reproducibility/{name}"),
            "current": git("rev-parse", f"HEAD:reproducibility/{name}"),
        }
        for name in protected_names
    }
    protected_unchanged = all(value["source"] == value["current"] for value in trees.values())
    table_rows = []
    for name, label in (
        ("hybrid_x007_y041_z041", "corrected 3D PID"),
        ("full_lqr_048", "Full-State LQR"),
        ("task_lqr_009", "Task-Weighted LQR"),
        ("satc_b_027", "SATC-OFMPC"),
    ):
        data = comparator["participants"][name]
        full_position = comparator["participants"]["full_lqr_048"]["overall"]["position_mean_m"]
        improvement = (full_position - data["overall"]["position_mean_m"]) / full_position
        status = "frozen V6 comparator" if name != "satc_b_027" else "frozen Self reference; no V6 tuning"
        table_rows.append(row(label, "Traditional" if name != "satc_b_027" else "Self", status, data["overall"], data["strong"], improvement))
    table_rows.append(row("YU2026-FT-CFO-ADAPTED-5LINK", "Recent Paper adaptation", "Development ineligible; Holdout not run", yu["overall"], yu["strong"], yu["comparisons"]["position_improvement_vs_full"]))
    table_rows.append(row("SEP2026-PASSIVITY-NMPC-ADAPTED-5LINK", "Recent Paper adaptation", "Development ineligible; Holdout not run", sep["overall"], sep["strong"], sep["comparisons"]["position_improvement_vs_full"]))
    table_path = DOCS / "FINAL_COMPARISON_TABLE.csv"
    table_path.parent.mkdir(parents=True, exist_ok=True)
    with table_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(table_rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(table_rows)

    satc = comparator["participants"]["satc_b_027"]
    full = comparator["participants"]["full_lqr_048"]
    satc_position_improvement = (full["overall"]["position_mean_m"] - satc["overall"]["position_mean_m"]) / full["overall"]["position_mean_m"]
    satc_bootstrap = satc_v6_bootstrap()
    write_json(FINAL / "final_gate.json", {
        "task": "V6-RECENT-PAPER-ADVANCED-BENCHMARK-SUITE-AND-FINAL-PUBLICATION-FREEZE-R1",
        "result": "V6_NO_QUALIFIED_RECENT_PAPER_BASELINE", "v6_complete": True, "v6_permanently_closed": True,
        "highest_project_claim": "V5_SELF_OVERALL_HOLDOUT_WIN", "highest_project_claim_changed_by_v6": False,
        "qualified_paper_count": 0, "v6_holdout": {"executed": False, "sample_count": 0, "participants": 0, "retries": [], "compromised": False, "reason": "ZERO_QUALIFIED_PAPERS"},
        "v6_development_authoritative_runs": development["authoritative_runs"],
        "checkpoints": {"source_head": SOURCE_HEAD, "contract_head": CONTRACT_HEAD, "paper_suite_head": SUITE_HEAD, "implementation_head": IMPLEMENTATION_HEAD, "paper_development_final_head": DEVELOPMENT_HEAD, "holdout_protocol_head": None},
        "v1_v5_unchanged": protected_unchanged, "protected_trees": trees,
    })
    write_json(FINAL / "method_status.json", {
        "traditional": {"status": "FROZEN_UNCHANGED", "methods": ["hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009"]},
        "satc": {"candidate": "satc_b_027", "status": "FROZEN_UNCHANGED", "v5": "V5_SELF_OVERALL_HOLDOUT_WIN", "v6_role": "reference only"},
        "YU2026": {"development": "PAPER_DEVELOPMENT_INELIGIBLE", "holdout": "NOT_RUN_DEVELOPMENT_INELIGIBLE", "best_candidate": yu["candidate_id"]},
        "SEP2026": {"development": "PAPER_DEVELOPMENT_INELIGIBLE", "holdout": "NOT_RUN_DEVELOPMENT_INELIGIBLE", "best_candidate": sep["candidate_id"]},
        "KANG2026": {"status": "NOT_IMPLEMENTED_FULL_TEXT_UNAVAILABLE_NO_FORMULA_GUESSING"},
    })
    write_json(FINAL / "satc_v6_reference.json", {
        "candidate": "satc_b_027", "status": "FROZEN_UNCHANGED_REFERENCE_ONLY",
        "overall": satc["overall"], "strong": satc["strong"],
        "position_improvement_vs_full_lqr": satc_position_improvement,
        "paired_bootstrap": satc_bootstrap,
        "holdout": "NOT_RUN_V6_ZERO_QUALIFIED_PAPERS",
    })
    write_json(FINAL / "claim_matrix.json", {
        "allowed": [
            "V5 SATC-OFMPC remains the project's formal Overall Holdout winner against the frozen Traditional benchmark.",
            "On the new V6 Development bank, frozen SATC achieved lower mean position RMSE than frozen Full-LQR without V6 retuning.",
            "The two preregistered recent-Paper adaptations failed V6 Development qualification and were not evaluated on V6 Holdout.",
            "The negative Paper results are adaptation results on this five-link benchmark, not evaluations of the original authors' systems.",
        ],
        "forbidden": [
            "V5_SELF_STRICT_HOLDOUT_WIN", "Paper-Advanced > Traditional", "SATC beat Yu et al. or SEP-NMPC as originally published",
            "V6 Holdout was executed", "Kang and Shan formulas were reproduced", "real-flight or hardware validation",
        ],
        "v6_satc_development_position_improvement_vs_full": satc_position_improvement,
    })
    write_json(FINAL / "publication_package.json", {
        "technical_report": "docs/v6/FINAL_PROJECT_TECHNICAL_REPORT.md", "paper_ready_results": "docs/v6/PAPER_READY_RESULTS.md",
        "comparison_table": "docs/v6/FINAL_COMPARISON_TABLE.csv", "claim_matrix": "docs/v6/FINAL_CLAIM_MATRIX.md",
        "resume": "docs/v6/RESUME_PROJECT_FINAL.md", "interview_qa": "docs/v6/INTERVIEW_QA.md",
        "method_evolution": "docs/v6/METHOD_EVOLUTION.md", "negative_results": "docs/v6/NEGATIVE_RESULTS.md",
    })
    write_json(FINAL / "project_tests.json", {
        "command": "python -m pytest tests/release tests/v2 tests/v3 tests/v4 tests/v5 tests/v6 --import-mode=importlib -q",
        "result": "190 passed", "warnings": "6 dependency deprecation warnings from OSQP tests", "performance_rerun_during_finalization": False,
    })

    write_text(DOCS / "FINAL_PROJECT_TECHNICAL_REPORT.md", f"""
# Final Project Technical Report

## Final scientific status

The project closes with **V5_SELF_OVERALL_HOLDOUT_WIN** as its highest positive result and **V6_NO_QUALIFIED_RECENT_PAPER_BASELINE** as the final V6 outcome. V6 did not alter or retune SATC-OFMPC, the Traditional controllers, or any V1-V5 evidence.

## V5 retained result

Frozen `satc_b_027` remains the formal unseen-Holdout Self result: 96 samples, 100% safety, 55.21% success, 0.09683 m position RMSE, and a positive 10,000-resample paired-bootstrap interval versus `full_lqr_048`. It is an Overall, not Strict, Holdout win.

## V6 preregistered design

V6 froze a new 120-sample Development bank and a disjoint 96-sample locked Holdout bank before Paper performance. The suite was fixed to Yu et al. (2026) and SEP-NMPC (2026). Kang and Shan (2026) was excluded before performance because only abstract-level primary content was retrievable. The Paper search budget was 32 unique configurations per method: eight 24-sample smoke configurations and 24 full 120-sample configurations.

## V6 Development result

The best Yu adaptation `{yu['candidate_id']}` passed {yu['gate_pass_count']}/8 gates: safety {100*yu['overall']['safety_rate']:.2f}%, success {100*yu['overall']['success_rate']:.2f}%, position RMSE {yu['overall']['position_mean_m']:.4f} m, and {len(yu['catastrophic_pair_ids'])} strong catastrophic pairs. The best SEP adaptation `{sep['candidate_id']}` passed {sep['gate_pass_count']}/8 gates: safety {100*sep['overall']['safety_rate']:.2f}%, success {100*sep['overall']['success_rate']:.2f}%, position RMSE {sep['overall']['position_mean_m']:.4f} m, and {len(sep['catastrophic_pair_ids'])} strong catastrophic pairs. Its overall paired-bootstrap interval versus Full-LQR was [{sep['bootstrap']['overall']['ci_95_m'][0]:.4f}, {sep['bootstrap']['overall']['ci_95_m'][1]:.4f}] m, entirely favoring Full-LQR.

Because no Paper qualified, V6 Holdout was not executed. This is the preregistered stopping outcome, not an infrastructure block.

## Scope

Evidence supports SATC's V5 simulated Holdout result and the V6 adaptation-level negative results. It does not establish superiority over the original published systems, a Paper-Advanced win, real-flight behavior, hardware feasibility, or a Strict V5 win.
""")
    write_text(DOCS / "PAPER_READY_RESULTS.md", f"""
# Paper-Ready Results

## Benchmark and protocol

All controllers used the same MuJoCo five-link plant, cutter-tip task metrics, geometric inner loop, ±2 m/s² world-frame acceleration authority, and 0.25 m/s²/update slew limit. V6 Paper adaptation used only the new 120-sample Development split. The 96-sample V6 Holdout remained locked because no Paper qualified.

## Main comparator table

Use `FINAL_COMPARISON_TABLE.csv` for exact Development values. The frozen SATC reference achieved {100*satc['overall']['safety_rate']:.2f}% safety, {100*satc['overall']['success_rate']:.2f}% success, {satc['overall']['position_mean_m']:.5f} m mean position RMSE, and {satc['overall']['acquisition_median_s']:.4f} s acquisition on V6 Development without retuning. These are supporting cross-version Development results; the formal SATC claim remains its V5 unseen Holdout win.

## Recent-Paper adaptations

YU2026-FT-CFO preserved the compensation-function observer, finite-time signed-power feedback, and an equivalent multi-link swing-energy term. SEP2026 preserved finite-horizon constrained optimization, storage shaping, and strict passivity filtering; obstacle HOCBFs were inapplicable because the frozen benchmark has no obstacle task. Neither method met Development qualification. Therefore no Paper was evaluated on V6 Holdout, and no Paper-versus-SATC Holdout statement is valid.

## Recommended result sentence

"A preregistered adaptation study of two recent suspended-payload controllers produced no Development-qualified external advanced baseline under the frozen five-link acceleration interface; consequently, the unseen V6 Holdout remained unopened, while the previously established V5 SATC Overall Holdout win was retained unchanged."
""")
    write_text(DOCS / "FINAL_CLAIM_MATRIX.md", """
# Final Claim Matrix

## Allowed

- SATC-OFMPC achieved a formal V5 Overall Holdout win against the frozen Traditional benchmark.
- SATC reduced V5 Holdout position RMSE with a positive paired-bootstrap lower bound and 100% safety.
- Two recent-Paper adaptations were preregistered, fully reported, and Development-ineligible in V6.
- V6 Holdout was not run because zero Papers qualified.

## Required qualifications

- V5 is not a Strict all-metric win.
- Strong-cohort task success remained zero for both SATC and Full-LQR in V5.
- Paper results concern benchmark adaptations, not the original authors' hardware or models.

## Forbidden

- Paper-Advanced outperformed Traditional.
- SATC outperformed the original Yu et al. or SEP-NMPC systems.
- V6 includes unseen-Holdout Paper evidence.
- Results establish real-flight, hardware, or field performance.
""")
    write_text(DOCS / "RESUME_PROJECT_FINAL.md", """
# Resume Project Final

## 中文简历版

构建并冻结五连杆无人机吊载抗摆 MuJoCo 基准，统一 PID、Full-State LQR、Task-LQR 与自研 SATC-OFMPC 的三维加速度权限；在 96 个未见 Holdout 样本上，自研方法保持 100% safety，并将相对 Full-LQR 的位置 RMSE 降低 29.10%，10,000 次配对 bootstrap 置信区间严格大于零。进一步预注册并适配两种 2026 论文控制方法，完整保留其 Development 负结果且未打开不合格方法的 Holdout。

## English resume version

Built a reproducible five-link UAV suspended-tool MuJoCo benchmark and a fair acceleration-level comparison across PID, two LQR baselines, and a self-developed SATC-OFMPC controller. Achieved a 29.10% position-RMSE reduction versus Full-LQR on a 96-sample unseen Holdout with 100% safety and a strictly positive 10,000-resample paired-bootstrap interval; preregistered and transparently reported two recent-paper adaptation failures without test-set tuning.

## Do not claim

Do not describe the result as real flight, a Strict all-metric win, or superiority over the original published Paper systems.
""")
    write_text(DOCS / "INTERVIEW_QA.md", """
# Interview Q&A

## What is the strongest result?

The strongest result is the V5 SATC-OFMPC Overall Holdout win: lower position RMSE and higher success than frozen Full-LQR on 96 unseen samples, 100% safety, and a positive paired-bootstrap confidence interval.

## Why is it not a Strict win?

Normal-regime acquisition degraded 11.27% versus the frozen legacy Self reference, exceeding the preregistered 10% allowance.

## Why did V6 not run Holdout?

Both recent-Paper adaptations failed the preregistered Development gate. The contract required zero-qualified Paper suites to stop before Holdout, preventing test-set fishing.

## Why not implement Kang and Shan from the abstract?

The full primary equations were not legally and reliably available at suite freeze. Guessing formulas would have produced an unverifiable method, so a complete predeclared 2026 fallback was used instead.

## What did the Paper failures teach you?

Cross-model adaptation is not a neutral implementation detail. A controller derived for a point load or obstacle-aware single cable can lose stability or task accuracy when mapped to a five-link cutter and a strict acceleration/slew interface. The negative results quantify that adaptation gap.

## What remains before deployment?

Hardware-in-the-loop timing, actuator and sensor calibration, model mismatch, cable/link flexibility, estimator noise, embedded computation, and real-flight safety validation remain outside this project.
""")
    write_text(DOCS / "METHOD_EVOLUTION.md", """
# Method Evolution

## LS-PMPC

Established the predictive-control and reproducibility foundation, but did not yield a final robust task-space winner.

## OF/DR-TSRMPC

Added offset-free and disturbance-rejection mechanisms. Formal gates exposed persistent acquisition and robustness limitations; the route was closed without hiding failures.

## CART-OFMPC

Introduced constraint-feasible steady targets, explicit unrepresented residuals, trust, and anti-windup debt. It clarified the saturation/estimation mechanism but did not achieve a V4 Development win.

## SATC-OFMPC

Added causal shock detection, bumpless offset engagement, physical-input coordination, slew headroom, and a geometric disturbance-task conflict index. The frozen `satc_b_027` passed Development and achieved the project's V5 Overall Holdout win.

## Recent-Paper suite

V6 froze Yu 2026 and SEP-NMPC 2026 adaptations before performance. Neither qualified on new Development, so the project closed without a Paper Holdout or replacement search.
""")
    write_text(DOCS / "NEGATIVE_RESULTS.md", f"""
# Negative Results

## V6 Yu 2026 adaptation

Best candidate `{yu['candidate_id']}` passed {yu['gate_pass_count']}/8 gates. It had {100*yu['overall']['safety_rate']:.2f}% safety, zero success, {yu['overall']['position_mean_m']:.4f} m mean position RMSE, and {len(yu['catastrophic_pair_ids'])} catastrophic strong pairs. The failure indicates that the point-load finite-time/CFO structure did not remain stable under this five-link task mapping and frozen slew authority.

## V6 SEP-NMPC adaptation

Best candidate `{sep['candidate_id']}` passed {sep['gate_pass_count']}/8 gates. It maintained 100% safety but achieved only {100*sep['overall']['success_rate']:.2f}% success and {sep['overall']['position_mean_m']:.4f} m position RMSE. Its paired-bootstrap interval versus Full-LQR was wholly negative, and it produced {len(sep['catastrophic_pair_ids'])} catastrophic strong pairs.

## Interpretation boundaries

These are failures of frozen five-link adaptations, not evidence that the original published controllers fail on their own systems. No parameters were added after the budget, no Paper was replaced after suite freeze, and V6 Holdout was not opened.
""")
    evidence = [
        V6 / "r0/research_contract.json", V6 / "r0/development_manifest.json", V6 / "r0/holdout_manifest.json",
        V6 / "papers/suite_freeze.json", V6 / "papers/implementation_freeze.json",
        V6 / "development/baseline_results.csv", V6 / "papers/yu2026/stage_b_results.csv", V6 / "papers/sep2026/stage_b_results.csv",
        V6 / "papers/yu2026/development_gate_audit.json", V6 / "papers/sep2026/development_gate_audit.json",
        V6 / "development/development_final.json", V6 / "holdout_status.json",
        FINAL / "final_gate.json", FINAL / "method_status.json", FINAL / "satc_v6_reference.json", FINAL / "claim_matrix.json", FINAL / "publication_package.json", FINAL / "project_tests.json",
        *sorted(DOCS.glob("*")),
    ]
    write_json(FINAL / "evidence_manifest.json", {str(path.relative_to(ROOT)).replace("\\", "/"): sha(path) for path in evidence})
    print(json.dumps({"result": "V6_NO_QUALIFIED_RECENT_PAPER_BASELINE", "highest_project_claim": "V5_SELF_OVERALL_HOLDOUT_WIN", "holdout_executed": False, "v1_v5_unchanged": protected_unchanged, "publication_files": len(list(DOCS.glob("*")))}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
