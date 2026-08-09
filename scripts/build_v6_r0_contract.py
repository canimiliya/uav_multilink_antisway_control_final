"""Freeze the V6 recent-Paper benchmark before any V6 performance."""

from __future__ import annotations

import hashlib
import json
import subprocess
from collections import Counter
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v6/r0"
SOURCE_TAG = "v5-research-final-2026-08-09"
SOURCE_HEAD = "60a6350d6875d4654ab881fdf8d96e749226133c"
DEV_SEED = 20260817
HOLDOUT_SEED = 20260818
BOOTSTRAP_SEED = 20260819


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def write(name: str, payload: object) -> None:
    path = OUT / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def directions(seed: int, prefix: str, count_per_stratum: int) -> list[dict]:
    rng = np.random.Generator(np.random.PCG64(seed))
    ranges = {"aligned": (0.55, 0.90), "opposed": (-0.90, -0.55), "cross": (-0.20, 0.20)}
    result = []
    for stratum, (low, high) in ranges.items():
        for index in range(count_per_stratum):
            x = float(rng.uniform(low, high))
            angle = float(rng.uniform(0.0, 2.0 * np.pi))
            radial = float(np.sqrt(1.0 - x * x))
            vector = np.asarray([x, radial * np.cos(angle), radial * np.sin(angle)])
            radius = 0.18 if index % 2 == 0 else 0.23
            result.append({
                "target_id": f"{prefix}_{stratum}_{index:02d}",
                "canonical_stratum": stratum,
                "direction_unit": vector.tolist(),
                "radius_m": radius,
                "delta_tip_m": (radius * vector).tolist(),
                "generator": "PCG64 stratified spherical direction",
                "generator_seed": seed,
            })
    return result


def sample(sample_id: str, cohort: str, target: dict, wind: dict, issue: float | None, allowed: bool) -> dict:
    sign = int(wind.get("direction_sign", 0))
    tx = float(target["direction_unit"][0])
    if sign == 0 or abs(tx) <= 0.25:
        stratum = "cross" if sign else "not_applicable"
    else:
        stratum = "aligned" if sign * tx > 0.0 else "opposed"
    return {
        "sample_id": sample_id,
        "cohort": cohort,
        "directional_stratum": stratum,
        "target": target,
        "target_issue_time_s": issue,
        "wind": wind,
        "duration_s": 12.0,
        "execution_allowed": allowed,
    }


def mirrored(target: dict, sign: int) -> dict:
    if sign > 0:
        return target
    result = dict(target)
    vector = -np.asarray(target["direction_unit"], dtype=float)
    result["direction_unit"] = vector.tolist()
    result["delta_tip_m"] = (float(target["radius_m"]) * vector).tolist()
    result["target_id"] = target["target_id"] + "_mirrored"
    return result


def make_bank(seed: int, prefix: str, holdout: bool) -> dict:
    targets = directions(seed, f"{prefix}_dir", 4 if holdout else 6)
    allowed = not holdout
    rows: list[dict] = []
    for index, target in enumerate(targets):
        rows.append(sample(f"{prefix}_calm_{index:02d}", "NORMAL_CALM", target, {"kind": "calm", "direction_sign": 0}, 3.0, allowed))
        sign = 1 if index % 2 == 0 else -1
        rows.append(sample(
            f"{prefix}_moderate_{index:02d}", "NORMAL_CONSTANT", target,
            {"kind": "constant", "speed_m_s": 2.3 if holdout else 2.1, "direction_sign": sign, "onset_time_s": 0.0}, 3.0, allowed,
        ))
    groups = {name: [t for t in targets if t["canonical_stratum"] == name] for name in ("aligned", "opposed", "cross")}
    specs = [
        ("preexisting", "STRONG_PREEXISTING", 0.0, 3.0),
        ("early", "STRONG_NEAR_SIMULTANEOUS", 2.65, 3.0),
        ("late", "STRONG_NEAR_SIMULTANEOUS", 3.35, 3.0),
    ]
    if holdout:
        specs = [
            ("delayed_target", "STRONG_PREEXISTING", 0.35, 3.25),
            ("near_before", "STRONG_NEAR_SIMULTANEOUS", 2.72, 3.0),
            ("near_after", "STRONG_NEAR_SIMULTANEOUS", 3.28, 3.0),
        ]
    for label, cohort, onset, issue in specs:
        for stratum, group in groups.items():
            for local, target in enumerate(group):
                sign = 1 if local % 2 == 0 else -1
                used = mirrored(target, sign)
                rows.append(sample(
                    f"{prefix}_strong_{label}_{stratum}_{local:02d}", cohort, used,
                    {"kind": "constant", "speed_m_s": 3.6 if holdout else 3.5, "direction_sign": sign, "onset_time_s": onset}, issue, allowed,
                ))
    stochastic_start = 7000 if holdout else 6000
    stochastic_count = 24 if holdout else 18
    for offset in range(stochastic_count):
        wind_seed = stochastic_start + offset
        target = targets[(3 * offset + 1) % len(targets)]
        sign = 1 if offset % 2 == 0 else -1
        rows.append(sample(
            f"{prefix}_stochastic_{wind_seed}", "NORMAL_STOCHASTIC", target,
            {"kind": "stochastic", "seed": wind_seed, "sigma_m_s": 0.92 if holdout else 0.88, "clip_m_s": 3.1, "direction_sign": sign}, 3.0, allowed,
        ))
    for stratum, group in groups.items():
        for local, target in enumerate(group[:4]):
            sign = 1 if local % 2 == 0 else -1
            used = mirrored(target, sign)
            rows.append(sample(
                f"{prefix}_ramp_{stratum}_{local:02d}", "RAMP_STRONG", used,
                {"kind": "ramp", "start_speed_m_s": 0.0, "speed_m_s": 3.6 if holdout else 3.5, "direction_sign": sign,
                 "onset_time_s": 2.18 if holdout else 1.85, "duration_s": 5.55 if holdout else 6.15}, 3.0, allowed,
            ))
    return {
        "name": "V6_FROZEN_HOLDOUT_BANK" if holdout else "V6_FROZEN_DEVELOPMENT_BANK",
        "generator_seed": seed,
        "sample_count": len(rows),
        "samples": rows,
        "cohort_counts": dict(sorted(Counter(row["cohort"] for row in rows).items())),
        "directional_counts": dict(sorted(Counter(row["directional_stratum"] for row in rows if row["directional_stratum"] != "not_applicable").items())),
        "performance_accessed_at_freeze": False,
        "execution_allowed": allowed,
    }


def main() -> int:
    if git("branch", "--show-current") != "research-v6":
        raise RuntimeError("V6 contract must be created on research-v6")
    if git("rev-parse", f"{SOURCE_TAG}^{{}}") != SOURCE_HEAD:
        raise RuntimeError("V5 source tag drift")
    if git("merge-base", "HEAD", SOURCE_TAG) != SOURCE_HEAD:
        raise RuntimeError("research-v6 is not rooted at frozen V5")
    development = make_bank(DEV_SEED, "v6d", False)
    holdout = make_bank(HOLDOUT_SEED, "v6h", True)
    if development["sample_count"] != 120 or holdout["sample_count"] != 96:
        raise RuntimeError("V6 sample-count drift")
    write("development_manifest.json", development)
    write("holdout_manifest.json", holdout)
    write("research_contract.json", {
        "task": "V6-RECENT-PAPER-ADVANCED-BENCHMARK-SUITE-AND-FINAL-PUBLICATION-FREEZE-R1",
        "source_tag": SOURCE_TAG, "source_head": SOURCE_HEAD, "branch": "research-v6",
        "scientific_question": "Can recent faithfully adapted published suspended-payload controllers outperform frozen Traditional controllers on the same five-link acceleration-authority benchmark?",
        "frozen": {
            "plant": "MuJoCo five-link plant", "task_metrics_safety": True, "geometric_inner_loop": True,
            "command_frame": "world [ax, ay, az]", "acceleration_limit_m_s2_per_axis": 2.0,
            "slew_limit_m_s2_per_update": 0.25, "outer_rate_hz": 20.0,
            "traditional": ["hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009"],
            "primary_traditional": "full_lqr_048", "self_reference": "satc_b_027",
        },
        "self_search": "FORBIDDEN", "satc_role": "FROZEN_SELF_REFERENCE_ONLY",
        "paper_suite_size_max": 3, "paper_suite_selected_before_performance": True,
        "zero_qualified_action": "V6_NO_QUALIFIED_RECENT_PAPER_BASELINE; Holdout not executed",
        "holdout_policy": "one shot only after all qualified Paper parameters freeze",
        "forbidden": ["V1-V5 mutation", "SATC tuning", "Traditional tuning", "Paper fishing after suite freeze", "Holdout access before protocol push", "authority change"],
        "v6_controller_performance_executed": False, "v6_holdout_executed": False,
    })
    write("paper_selection_contract.json", {
        "search_years": [2025, 2026], "suite_size_min": 2, "suite_size_max": 3,
        "primary_source_required": True, "full_equations_required": True,
        "priority": ["Yu et al. 2026 finite-time anti-swing plus compensation-function observer", "Kang and Shan 2026 fully-actuated-system robust controller"],
        "fallback_predeclared_before_performance": True,
        "ranking": ["primary-source completeness", "problem relevance", "five-link adaptability", "same acceleration authority", "publication quality"],
        "replacement_after_suite_freeze": False, "selection_by_performance": False,
        "adaptation_labels": ["PRESERVED", "ADAPTED", "OMITTED", "UNAVAILABLE"],
        "unfaithful_action": "UNFAITHFUL_ADAPTATION; withdraw before performance",
    })
    write("paper_search_budget.json", {
        "per_paper_unique_configuration_planned": 32, "per_paper_unique_configuration_max": 48,
        "stage_a": {"maximum": 8, "sample_count_each": 24, "selection_authority": False},
        "stage_b": {"maximum": 24, "sample_count_each": 120, "selection_authority": True},
        "selection_rule": "all qualification gates, then gate count, strong P90, overall position, acquisition, success, effort, runtime, candidate id",
        "budget_extension_after_failure": False,
    })
    write("win_contract.json", {
        "primary_traditional": "full_lqr_048", "best_traditional_fields": ["safety", "success"],
        "development_qualification": {
            "safety_no_worse_best_traditional": True, "success_no_worse_best_traditional": True,
            "position_improvement_vs_full_min": 0.05, "acquisition_degradation_vs_full_max": 0.05,
            "strong_p90_degradation_vs_full_max": 0.10, "catastrophic_pair_max": 0,
            "meaningful_extra": "strong mean improvement vs Full-LQR >=5% OR normal acquisition improvement vs Full-LQR >=5%",
            "bootstrap_lower_gt_zero": True,
        },
        "holdout_overall": {
            "safety_no_worse_full": True, "success_no_worse_full": True,
            "position_improvement_vs_full_min": 0.05, "acquisition_degradation_vs_full_max": 0.05,
            "bootstrap_lower_gt_zero": True, "strong_p90_degradation_vs_full_max": 0.10, "catastrophic_pair_max": 0,
        },
        "strict_additional": {"acquisition_improvement_vs_full_min": 0.05},
    })
    write("statistical_protocol.json", {
        "pairing": "exact sample_id", "resamples": 10000, "bootstrap_seed": BOOTSTRAP_SEED,
        "ci_quantiles": [0.025, 0.975], "cohorts": ["overall", "normal", "strong", "aligned", "opposed", "cross"],
        "seed_hunting_forbidden": True,
    })
    old_manifests = [
        "reproducibility/v3/r0/holdout_manifest.json", "reproducibility/v3/r4/holdout_execution_manifest.json",
        "reproducibility/v4/r0/holdout_manifest.json", "reproducibility/v4/final/unused_holdout_manifest.json",
        "reproducibility/v5/r0/holdout_manifest.json", "reproducibility/v5/holdout/execution_manifest.json",
    ]
    old_ids = {}
    for name in old_manifests:
        payload = json.loads((ROOT / name).read_text(encoding="utf-8"))
        rows = payload.get("samples", payload.get("execution_samples", []))
        old_ids[name] = sorted(row.get("sample_id", "") for row in rows)
    new_ids = {row["sample_id"] for row in holdout["samples"]}
    write("split_integrity.json", {
        "development_generator_seed": DEV_SEED, "holdout_generator_seed": HOLDOUT_SEED,
        "development_holdout_ids_disjoint": not ({r["sample_id"] for r in development["samples"]} & new_ids),
        "development_holdout_targets_disjoint": not ({r["target"]["target_id"] for r in development["samples"]} & {r["target"]["target_id"] for r in holdout["samples"]}),
        "development_stochastic_seeds": list(range(6000, 6018)), "holdout_stochastic_seeds": list(range(7000, 7024)),
        "old_holdout_id_overlap": {name: sorted(new_ids & set(ids)) for name, ids in old_ids.items()},
        "old_manifest_sha256": {name: digest(ROOT / name) for name in old_manifests},
        "v6_holdout_execution_allowed": False,
    })
    protected = ["frozen", "v2", "v3", "v4", "v5"]
    write("source_freeze.json", {
        "source_tag_type": git("cat-file", "-t", SOURCE_TAG), "source_head": SOURCE_HEAD,
        "protected_trees": {name: git("rev-parse", f"{SOURCE_TAG}:reproducibility/{name}") for name in protected},
        "traditional_parameter_files": {
            name: digest(ROOT / name) for name in [
                "reproducibility/v3/r1r1/pid_freeze.json", "reproducibility/v3/r1/full_lqr_freeze.json",
                "reproducibility/v3/r1/task_lqr_freeze.json", "reproducibility/v5/self/self_freeze.json",
            ]
        },
        "v1_v5_read_only": True,
    })
    contract_files = sorted(path for path in OUT.glob("*.json") if path.name != "contract_sha256.json")
    write("contract_sha256.json", {path.name: digest(path) for path in contract_files})
    print(json.dumps({"development_samples": 120, "holdout_samples": 96, "bootstrap_seed": BOOTSTRAP_SEED, "holdout_execution_allowed": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
