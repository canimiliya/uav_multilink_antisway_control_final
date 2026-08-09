"""Freeze the V5 transient-robust research contract before performance."""

from __future__ import annotations

import hashlib
import json
import subprocess
from collections import Counter
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v5/r0"
SOURCE_TAG = "v4-research-final-2026-08-09"
SOURCE_HEAD = "8e3d7ad08747b00d46b9bdfc3cc72451491567db"
DEV_SEED = 20260814
HOLDOUT_SEED = 20260815
BOOTSTRAP_SEED = 20260816


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def digest(path: Path) -> str:
    data = path.read_bytes()
    if b"\x00" not in data:
        data = data.replace(b"\r\n", b"\n")
    return hashlib.sha256(data).hexdigest()


def write_json(name: str, payload: object) -> None:
    path = OUT / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def directions(seed: int, prefix: str, count_per_stratum: int) -> list[dict]:
    """Generate deterministic spherical directions with explicit x strata."""
    rng = np.random.Generator(np.random.PCG64(seed))
    result: list[dict] = []
    ranges = {"aligned": (0.55, 0.90), "opposed": (-0.90, -0.55), "cross": (-0.20, 0.20)}
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


def sample(sample_id: str, cohort: str, target: dict, wind: dict, issue: float | None = 3.0, *, allowed: bool) -> dict:
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


def make_bank(seed: int, prefix: str, *, holdout: bool) -> dict:
    targets = directions(seed, f"{prefix}_dir", 4 if holdout else 6)
    allowed = not holdout
    rows: list[dict] = []
    # Every deterministic target is seen in calm and moderate wind.
    for index, target in enumerate(targets):
        rows.append(sample(f"{prefix}_calm_{index:02d}", "NORMAL_CALM", target, {"kind": "calm", "direction_sign": 0}, allowed=allowed))
        sign = 1 if index % 2 == 0 else -1
        rows.append(sample(f"{prefix}_moderate_{index:02d}", "NORMAL_CONSTANT", target, {"kind": "constant", "speed_m_s": 2.0 if not holdout else 2.25, "direction_sign": sign, "onset_time_s": 0.0}, allowed=allowed))
    # Strong cohorts are exactly balanced by relative direction, with both wind signs.
    by_canonical = {name: [t for t in targets if t["canonical_stratum"] == name] for name in ("aligned", "opposed", "cross")}
    strong_specs = [
        ("preexisting", "STRONG_PREEXISTING", 0.0, 3.0),
        ("simultaneous", "STRONG_SIMULTANEOUS", 3.0, 3.0),
        ("post_target", "STRONG_POST_TARGET", 4.0, 1.0),
    ]
    if holdout:
        # Previously unseen timing offsets: wind 0.15 s before and 0.10 s after target.
        strong_specs = [
            ("preexisting", "STRONG_PREEXISTING", 0.0, 3.0),
            ("near_before", "STRONG_NEAR_SIMULTANEOUS", 2.85, 3.0),
            ("near_after", "STRONG_NEAR_SIMULTANEOUS", 3.10, 3.0),
        ]
    for label, cohort, onset, issue in strong_specs:
        for stratum, group in by_canonical.items():
            for local, target in enumerate(group):
                sign = 1 if local % 2 == 0 else -1
                # Flip the target for negative wind so the relative stratum remains fixed.
                target_used = dict(target)
                if sign < 0:
                    vector = -np.asarray(target["direction_unit"], dtype=float)
                    target_used["direction_unit"] = vector.tolist()
                    target_used["delta_tip_m"] = (float(target["radius_m"]) * vector).tolist()
                    target_used["target_id"] = target["target_id"] + "_mirrored"
                rows.append(sample(
                    f"{prefix}_strong_{label}_{stratum}_{local:02d}", cohort, target_used,
                    {"kind": "constant", "speed_m_s": 3.5 if not holdout else 3.6, "direction_sign": sign, "onset_time_s": onset},
                    issue, allowed=allowed,
                ))
    stochastic_start = 5000 if holdout else 4500
    stochastic_count = 24 if holdout else 18
    for offset in range(stochastic_count):
        seed_value = stochastic_start + offset
        target = targets[offset % len(targets)]
        sign = 1 if offset % 2 == 0 else -1
        rows.append(sample(
            f"{prefix}_stochastic_{seed_value}", "NORMAL_STOCHASTIC", target,
            {"kind": "stochastic", "seed": seed_value, "sigma_m_s": 0.85 if not holdout else 0.9, "clip_m_s": 3.1, "direction_sign": sign},
            allowed=allowed,
        ))
    # Balanced strong ramps exercise headroom without sharing Holdout timings.
    for stratum, group in by_canonical.items():
        for local, target in enumerate(group[:4]):
            sign = 1 if local % 2 == 0 else -1
            target_used = dict(target)
            if sign < 0:
                vector = -np.asarray(target["direction_unit"], dtype=float)
                target_used["direction_unit"] = vector.tolist()
                target_used["delta_tip_m"] = (float(target["radius_m"]) * vector).tolist()
                target_used["target_id"] = target["target_id"] + "_mirrored"
            rows.append(sample(
                f"{prefix}_ramp_{stratum}_{local:02d}", "RAMP_STRONG", target_used,
                {"kind": "ramp", "start_speed_m_s": 0.0, "speed_m_s": 3.5 if not holdout else 3.6, "direction_sign": sign, "onset_time_s": 2.0 if not holdout else 2.35, "duration_s": 6.0 if not holdout else 5.7},
                allowed=allowed,
            ))
    return {
        "name": "V5_FROZEN_HOLDOUT_BANK" if holdout else "V5_FROZEN_DEVELOPMENT_BANK",
        "generator_seed": seed,
        "sample_count": len(rows),
        "samples": rows,
        "cohort_counts": dict(sorted(Counter(row["cohort"] for row in rows).items())),
        "directional_counts": dict(sorted(Counter(row["directional_stratum"] for row in rows if row["directional_stratum"] != "not_applicable").items())),
        "performance_accessed_at_freeze": False,
        "execution_allowed": allowed,
    }


def main() -> int:
    if git("branch", "--show-current") != "research-v5":
        raise RuntimeError("V5 contract must be created on research-v5")
    if git("rev-list", "-n", "1", SOURCE_TAG) != SOURCE_HEAD:
        raise RuntimeError("V4 source tag drift")
    if git("merge-base", "HEAD", SOURCE_TAG) != SOURCE_HEAD:
        raise RuntimeError("research-v5 is not rooted at the frozen V4 source")

    development = make_bank(DEV_SEED, "v5d", holdout=False)
    holdout = make_bank(HOLDOUT_SEED, "v5h", holdout=True)
    write_json("development_manifest.json", development)
    write_json("holdout_manifest.json", holdout)

    write_json("research_contract.json", {
        "task": "V5-END-TO-END-TRANSIENT-ROBUST-SELF-PAPER-AND-ONE-SHOT-VALIDATION-R1",
        "source_tag": SOURCE_TAG,
        "source_head": SOURCE_HEAD,
        "branch": "research-v5",
        "scientific_question": "Can a causal predictive controller remain tail-robust when a strong disturbance onset and a 3D target transient occur concurrently, without sacrificing normal-regime performance?",
        "frozen": {
            "plant": "MuJoCo five-link plant", "wind_physics": "unchanged world-x quadratic drag implementation",
            "task_and_metrics": True, "safety": True, "terminal_acquisition": True,
            "inner_loop": "frozen geometric inner loop", "command_frame": "world [ax, ay, az]",
            "acceleration_limit_m_s2_per_axis": 2.0, "slew_limit_m_s2_per_update": 0.25, "outer_rate_hz": 20.0,
            "traditional": ["hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009"],
            "legacy_self": "self_a_034", "v4_cart": "negative mechanism baseline only",
        },
        "forbidden": ["true wind measurement", "future wind", "target-x sign switch", "Holdout access before unlock", "old evidence mutation", "authority increase"],
        "phase_order": ["contract", "Self Development", "Self freeze and ablation", "Paper Development", "selection freeze", "one-shot Holdout", "final freeze"],
        "stop_on_self_failure": "V5_SELF_NO_DEVELOPMENT_QUALIFICATION; Paper and Holdout not executed",
        "new_controller_performance_executed": False,
        "holdout_executed": False,
    })
    write_json("self_method_contract.json", {
        "method": "SATC-OFMPC", "long_name": "Shock-Aware Transient-Coordination Offset-Free Model Predictive Control",
        "family": "CART-OFMPC plus causal transient coordination",
        "required_mechanisms": ["innovation shock detection", "bumpless offset engagement", "direct physical-input coordination", "finite-slew authority reserve", "disturbance-task conflict index"],
        "cart_mechanisms_retained": ["constraint-feasible steady target", "explicit unrepresented residual", "anti-windup debt", "causal residual estimator", "physical acceleration and slew constraints"],
        "conflict_definition": "causal opposition between controllable innovation-equivalent acceleration and desired task displacement; no coordinate-sign branch",
        "representative_trace_strata": ["normal", "strong_aligned", "strong_opposed", "strong_cross"],
        "implementation_started": False, "performance_executed": False,
    })
    write_json("self_search_budget.json", {
        "maximum_unique_configurations": 64,
        "stage_a": {"maximum": 12, "purpose": "structural smoke", "selection_authority": False, "sample_count_each": 24},
        "stage_b": {"maximum": 36, "purpose": "full frozen Development search", "sample_count_each": development["sample_count"]},
        "stage_c": {"maximum": 8, "purpose": "exact full-bank confirmation of existing Stage-B parameters", "creates_new_configuration": False},
        "maximum_unique_planned": 48,
        "selection_rule": "all hard gates first; then directional worst P90, strong mean, overall mean, normal mean, acquisition, effort, runtime, candidate id",
        "no_65th_configuration": True,
    })
    write_json("win_contract.json", {
        "primary_traditional": "full_lqr_048", "bootstrap": {"resamples": 10000, "seed": BOOTSTRAP_SEED, "confidence": 0.95, "required_lower_bound_m": 0.0},
        "catastrophic_definition": "candidate position RMSE > 2 * paired full_lqr_048 position RMSE",
        "overall": {"safety_no_worse_best_traditional": True, "success_no_worse_best_traditional": True, "position_improvement_vs_full_min": 0.05, "acquisition_degradation_vs_full_max": 0.05, "bootstrap_lower_gt_zero": True},
        "strong_transient": {"position_improvement_vs_full_min": 0.05, "position_improvement_vs_legacy_min": 0.30, "p90_degradation_vs_full_max": 0.10, "catastrophic_pair_max": 0, "success_no_worse_best_traditional": True},
        "normal_nonregression_vs_legacy": {"safety_no_worse": True, "success_no_worse": True, "position_degradation_max": 0.05, "acquisition_degradation_max": 0.10},
        "directional_hard_gate_each_aligned_opposed_cross": {"p90_degradation_vs_full_max": 0.10, "catastrophic_pair_max": 0},
        "holdout_uses_same_relative_rules": True, "development_thresholds_are_not_absolute_holdout_targets": True,
    })
    write_json("paper_selection_contract.json", {
        "year_range": [2024, 2026], "topic": "constraint-aware robust transient control for UAV suspended or slung payloads",
        "primary_source_required": True, "selection_count": 1, "unique_configuration_max": 32,
        "ranking": ["research relevance", "mathematical completeness", "five-link adaptability", "same acceleration authority", "available sensing", "mechanism preservation", "publication quality"],
        "selection_by_ease_of_defeat_forbidden": True, "performance_before_protocol_freeze": False,
        "qualification": "at least the frozen V5 Overall gate", "failure_status": "PAPER_DEVELOPMENT_INELIGIBLE",
    })
    write_json("statistical_protocol.json", {
        "pairing": "exact sample_id", "resamples": 10000, "bootstrap_seed": BOOTSTRAP_SEED, "ci": [0.025, 0.975],
        "formal_reports": ["overall", "normal", "strong", "aligned", "opposed", "cross"], "seed_hunting_forbidden": True,
    })
    write_json("split_integrity.json", {
        "development_generator_seed": DEV_SEED, "holdout_generator_seed": HOLDOUT_SEED,
        "development_and_holdout_target_ids_disjoint": not ({s["target"]["target_id"] for s in development["samples"]} & {s["target"]["target_id"] for s in holdout["samples"]}),
        "development_stochastic_seeds": list(range(4500, 4518)), "holdout_stochastic_seeds": list(range(5000, 5024)),
        "stochastic_seeds_disjoint": True, "holdout_is_v4_unused_holdout": False,
        "holdout_timing_variants_s": [-0.15, 0.10], "holdout_execution_allowed": False,
    })
    source_files = [
        "reproducibility/v4/final/final_gate.json", "reproducibility/v4/final/v5_recommendation.json",
        "reproducibility/v4/final/simultaneous_tail_audit.json", "reproducibility/v4/r1/near_miss.json",
        "src/uav_sway/v4/cart_ofmpc.py", "reproducibility/v3/r1/full_lqr_freeze.json",
        "reproducibility/v3/r2/self_freeze.json",
    ]
    write_json("source_freeze.json", {
        "source_tag_type": git("cat-file", "-t", SOURCE_TAG), "source_head": SOURCE_HEAD,
        "v4_tree": git("rev-parse", f"{SOURCE_TAG}:reproducibility/v4"), "v1_v4_read_only": True,
        "source_sha256": {name: digest(ROOT / name) for name in source_files},
    })
    contract_files = sorted(path for path in OUT.glob("*.json") if path.name != "contract_sha256.json")
    write_json("contract_sha256.json", {path.name: digest(path) for path in contract_files})
    print(json.dumps({"development_samples": development["sample_count"], "holdout_samples": holdout["sample_count"], "holdout_execution_allowed": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
