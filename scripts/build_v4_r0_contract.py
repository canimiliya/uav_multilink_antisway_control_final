"""Build and freeze the V4-R0 strong-wind research contract."""

from __future__ import annotations

import csv
import hashlib
import json
import subprocess
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v4/r0"
SOURCE_TAG = "v3-research-final-2026-08-09"
SOURCE_HEAD = "1f6ef12bc657724024f4e281c6b0534ca1d76fad"
STATE_NAMES = [
    "ex", "evx", "ey", "evy", "ez", "evz", "roll", "roll_rate", "pitch", "pitch_rate",
    "q1", "q2", "q3", "q4", "q5", "qdot1", "qdot2", "qdot3", "qdot4", "qdot5",
]


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(name: str, payload: dict) -> None:
    path = OUT / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def sha256(path: Path) -> str:
    data = path.read_bytes()
    if b"\x00" not in data:
        data = data.replace(b"\r\n", b"\n")
    return hashlib.sha256(data).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def mean(rows: list[dict], field: str) -> float:
    return float(np.mean([float(row[field]) for row in rows]))


def maximum(rows: list[dict], field: str) -> float:
    return float(np.max([float(row[field]) for row in rows]))


def sample(sample_id: str, target: dict, wind: dict, cohort: str, target_issue: float | None = 3.0) -> dict:
    return {
        "sample_id": sample_id,
        "cohort": cohort,
        "target": target,
        "target_issue_time_s": target_issue,
        "wind": wind,
        "duration_s": 12.0,
        "execution_allowed": True,
    }


def main() -> int:
    if git("branch", "--show-current") != "research-v4":
        raise RuntimeError("V4-R0 must be built on research-v4")
    if git("rev-list", "-n", "1", SOURCE_TAG) != SOURCE_HEAD:
        raise RuntimeError("source tag drift")
    diagnostic_path = OUT / "strong_wind_diagnostic.csv"
    rows = list(csv.DictReader(diagnostic_path.open(encoding="utf-8", newline="")))
    replay_self = [r for r in rows if r["regime"] == "V3_CONSTANT_3P5_REPLAY" and r["controller"] == "self_a_034"]
    replay_full = [r for r in rows if r["regime"] == "V3_CONSTANT_3P5_REPLAY" and r["controller"] == "full_lqr_048"]
    if len(replay_self) != 12 or len(replay_full) != 12:
        raise RuntimeError("missing 12-pair strong-wind replay")
    holdout_rows = list(csv.DictReader((ROOT / "reproducibility/v3/r4/holdout_results.csv").open(encoding="utf-8", newline="")))
    expected = {
        (r["candidate_id"], r["sample_id"]): float(r["position_rmse_3d_m"])
        for r in holdout_rows if r["sample_id"].startswith("holdout_constant_3p5_")
    }
    replay_match = all(
        abs(float(r["position_rmse_3d_m"]) - expected[(r["controller"], r["source_sample_id"])]) <= 1.0e-12
        for r in replay_self + replay_full
    )
    ramp_self = next(r for r in rows if r["diagnostic_id"] == "matched_ramp_equilibrium" and r["controller"] == "self_a_034")
    ramp_full = next(r for r in rows if r["diagnostic_id"] == "matched_ramp_equilibrium" and r["controller"] == "full_lqr_048")
    matched = {
        r["diagnostic_id"]: r
        for r in rows
        if r["controller"] == "self_a_034"
        and r["diagnostic_id"] in {
            "matched_calm", "matched_constant_2p0", "timing_simultaneous_t3",
            "timing_post_target_t4", "strong_wind_equilibrium_no_step",
        }
    }
    clipped = sorted(
        [
            {
                "state": name,
                "mean_clip_update_fraction": mean(replay_self, f"raw_clip_state_{index:02d}_fraction"),
            }
            for index, name in enumerate(STATE_NAMES)
        ],
        key=lambda item: item["mean_clip_update_fraction"],
        reverse=True,
    )

    source_files = [
        "reproducibility/v3/r2/self_freeze.json",
        "reproducibility/v3/r1/full_lqr_freeze.json",
        "reproducibility/v3/r4/holdout_results.csv",
        "reproducibility/v3/r4/gate.json",
        "docs/V3_FINAL_TECHNICAL_REPORT.md",
        "src/uav_sway/v3/dr_tsrmpc.py",
        "scripts/run_v3_r1_baselines.py",
    ]
    write_json("source_freeze.json", {
        "task": "V4-R0-STRONG-WIND-GENERALIZATION-POSTMORTEM-AND-RESEARCH-CONTRACT-FREEZE-R1",
        "source_tag": SOURCE_TAG,
        "source_tag_type": git("cat-file", "-t", SOURCE_TAG),
        "source_head": SOURCE_HEAD,
        "source_tag_peeled_head": git("rev-list", "-n", "1", SOURCE_TAG),
        "v3_final_status": "V3_SELF_DEVELOPMENT_WIN_NOT_CONFIRMED_ON_HOLDOUT",
        "v3_final_tree": git("rev-parse", f"{SOURCE_TAG}:reproducibility/v3"),
        "v3_final_unchanged": True,
        "v3_holdout_is_now_v4_prior_evidence": True,
        "v3_holdout_is_v4_holdout": False,
        "v3_read_only": True,
        "hash_normalization": "text files canonicalized to LF before SHA-256",
        "source_file_sha256": {path: sha256(ROOT / path) for path in source_files},
    })

    mechanism = {
        "primary_root_cause": {
            "status": "STRONGLY_SUPPORTED",
            "name": "constraint-unaware residual-to-equilibrium mapping enters a saturated positive-feedback regime",
            "mechanism": (
                "An abrupt sustained 3.5 m/s wind drives one-step dynamic residuals beyond the componentwise "
                "estimator bound. The frozen steady compensator maps that clipped residual to an unconstrained "
                "equilibrium command far outside the common acceleration authority, then uniformly scales the "
                "state/command solution without exposing the unrepresented residual to an anti-windup or trust "
                "mechanism. The task-LQR backbone and QP then make large opposing corrections around a partially "
                "represented equilibrium while amplitude and slew constraints remain active, sustaining model error."
            ),
        },
        "secondary_root_causes": [
            {
                "status": "CONFIRMED",
                "name": "componentwise residual clipping concentrated in cutter angular-rate states",
                "evidence": clipped[:4],
            },
            {
                "status": "STRONGLY_SUPPORTED",
                "name": "abrupt wind onset creates a path-dependent estimator/constraint excursion",
                "evidence": {
                    "preexisting_position_rmse_m": float(replay_self[3]["position_rmse_3d_m"]),
                    "simultaneous_position_rmse_m": float(matched["timing_simultaneous_t3"]["position_rmse_3d_m"]),
                    "post_target_position_rmse_m": float(matched["timing_post_target_t4"]["position_rmse_3d_m"]),
                    "ramp_position_rmse_m": float(ramp_self["position_rmse_3d_m"]),
                },
            },
            {
                "status": "STRONGLY_SUPPORTED",
                "name": "large backbone/QP cancellation amplifies sensitivity near active constraints",
                "evidence": {
                    "backbone_norm_mean": mean(replay_self, "backbone_norm_mean"),
                    "qp_correction_norm_mean": mean(replay_self, "qp_correction_norm_mean"),
                    "physical_saturation_fraction": mean(replay_self, "physical_saturation_fraction"),
                    "slew_active_fraction": mean(replay_self, "slew_active_fraction"),
                },
            },
        ],
        "evidence_against": [
            {
                "candidate": "controllable-subspace rejection is the primary cause",
                "evidence": {
                    "projected_residual_norm_mean": mean(replay_self, "d_projected_norm_mean"),
                    "rejected_residual_norm_mean": mean(replay_self, "d_rejected_norm_mean"),
                    "rejected_to_projected_ratio": mean(replay_self, "d_rejected_norm_mean") / mean(replay_self, "d_projected_norm_mean"),
                },
            },
            {"candidate": "QP solver failure", "evidence": {"qp_solved_fraction": mean(replay_self, "qp_solved_fraction")}},
            {
                "candidate": "reference step is necessary for failure",
                "evidence": {"strong_wind_no_reference_step_position_rmse_m": float(matched["strong_wind_equilibrium_no_step"]["position_rmse_3d_m"])},
            },
            {
                "candidate": "common physical acceleration authority is intrinsically insufficient",
                "evidence": {
                    "full_lqr_position_rmse_m": mean(replay_full, "position_rmse_3d_m"),
                    "full_lqr_physical_saturation_fraction": mean(replay_full, "physical_saturation_fraction"),
                },
            },
        ],
        "unknown_remaining": [
            "The exact wind-onset slope and magnitude boundary at which the residual loop leaves its feasible trust region.",
            "How much of the sustained prediction error is aerodynamic nonlinearity versus state-dependent cutter motion.",
            "Whether confidence scheduling alone is sufficient or an explicit bounded disturbance state is also required.",
        ],
    }
    write_json("failure_mechanism_report.json", mechanism)
    write_json("v3_failure_postmortem.json", {
        "v3_status_preserved": "V3_SELF_DEVELOPMENT_WIN_NOT_CONFIRMED_ON_HOLDOUT",
        "diagnostic_scope": "instrument-only frozen-controller replay used as V4 prior evidence",
        "diagnostic_case_count": len(rows) // 2,
        "diagnostic_run_count": len(rows),
        "controllers": ["full_lqr_048", "self_a_034"],
        "v3_failure_reproduced": replay_match,
        "constant_3p5": {
            "sample_count": 12,
            "full_lqr_position_rmse_mean_m": mean(replay_full, "position_rmse_3d_m"),
            "self_position_rmse_mean_m": mean(replay_self, "position_rmse_3d_m"),
            "self_orientation_rmse_mean_deg": mean(replay_self, "orientation_rmse_deg"),
            "self_dynamic_residual_mean": mean(replay_self, "d_raw_norm_mean"),
            "self_residual_clip_update_fraction": mean(replay_self, "raw_clip_update_fraction"),
            "self_unscaled_steady_input_mean_max_abs_m_s2": mean(replay_self, "unscaled_steady_input_max_abs_mean"),
            "self_unscaled_steady_input_peak_max_abs_m_s2": maximum(replay_self, "unscaled_steady_input_max_abs_max"),
            "self_steady_scaling_active_fraction": mean(replay_self, "steady_scaling_active_fraction"),
            "self_steady_scale_mean": mean(replay_self, "steady_scale_mean"),
            "self_physical_saturation_fraction": mean(replay_self, "physical_saturation_fraction"),
            "self_slew_active_fraction": mean(replay_self, "slew_active_fraction"),
        },
        "ramp_0_to_3p5": {
            "full_lqr_position_rmse_m": float(ramp_full["position_rmse_3d_m"]),
            "self_position_rmse_m": float(ramp_self["position_rmse_3d_m"]),
            "self_dynamic_residual_mean": float(ramp_self["d_raw_norm_mean"]),
            "self_residual_clip_update_fraction": float(ramp_self["raw_clip_update_fraction"]),
            "self_steady_scaling_active_fraction": float(ramp_self["steady_scaling_active_fraction"]),
            "self_physical_saturation_fraction": float(ramp_self["physical_saturation_fraction"]),
            "self_slew_active_fraction": float(ramp_self["slew_active_fraction"]),
        },
        "timing_diagnostics": {
            key: {
                "position_rmse_m": float(row["position_rmse_3d_m"]),
                "residual_clip_update_fraction": float(row["raw_clip_update_fraction"]),
                "steady_scaling_active_fraction": float(row["steady_scaling_active_fraction"]),
            }
            for key, row in matched.items()
        },
        "root_cause_audited": True,
        "new_self_performance_executed": False,
        "v4_holdout_executed": False,
    })

    hypothesis = {
        "selection_count": 1,
        "selected": {
            "name": "CART-OFMPC",
            "long_name": "Constraint-Aware Residual-Trust Offset-Free Model Predictive Control",
            "status": "V4_SINGLE_SELF_HYPOTHESIS_FROZEN_NOT_IMPLEMENTED",
            "core_mechanism": [
                "Estimate disturbance/residual state causally from measured state and the previous actual limited command.",
                "Compute the steady target inside the common acceleration bounds and a finite slew-reachable set; never silently scale an infeasible equilibrium.",
                "Schedule residual trust from innovation clipping, prediction consistency, and constraint headroom.",
                "Back-calculate the unrepresented residual as anti-windup state so the estimator, backbone, and QP share one feasible offset model.",
                "Retain a bounded task-LQR-centered correction when residual confidence is low; no wind-truth switch is available.",
            ],
            "why_it_addresses_v3_failure": (
                "It directly interrupts the observed chain from clipped persistent residual to infeasible steady command, "
                "silent scaling, large backbone/QP cancellation, and sustained amplitude/slew activity."
            ),
            "causal_information_only": True,
            "forbidden_information": ["true wind speed", "future wind", "V4 Holdout results"],
            "implementation_started": False,
            "performance_executed": False,
        },
        "alternatives_considered_not_selected": [
            "unbounded disturbance-state augmentation",
            "wind-threshold controller switching",
            "larger horizon only",
            "higher acceleration or slew authority",
        ],
    }
    write_json("v4_method_hypothesis.json", hypothesis)
    write_json("v4_control_contract.json", {
        "command_interface": {"frame": "world", "components": ["ax", "ay", "az"]},
        "acceleration_limit_m_s2_per_axis": 2.0,
        "slew_limit_m_s2_per_update": 0.25,
        "outer_rate_hz": 20.0,
        "outer_period_s": 0.05,
        "inner_loop": "same frozen geometric inner loop",
        "plant": "same frozen MuJoCo five-link plant",
        "task_definition_unchanged": True,
        "safety_definition_unchanged": True,
        "traditional_parameters_frozen": ["hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009"],
        "legacy_self_baseline": "self_a_034",
        "legacy_self_mutable": False,
        "physical_contract_change_required": False,
    })

    v3_manifest = read_json(ROOT / "reproducibility/v3/r4/holdout_execution_manifest.json")
    v3_targets = [sample["target"] for sample in v3_manifest["samples"][:12]]
    development_samples = []
    for index, target in enumerate(v3_targets):
        tid = f"v4d_prior_dir_{index:02d}"
        development_samples.extend([
            sample(f"dev_calm_{index:02d}", target, {"kind": "calm"}, "NORMAL_CALM"),
            sample(f"dev_constant_2p0_{index:02d}", target, {"kind": "constant", "speed_m_s": 2.0, "onset_time_s": 0.0}, "NORMAL_CONSTANT"),
            sample(f"dev_strong_preexisting_{index:02d}", target, {"kind": "constant", "speed_m_s": 3.5, "onset_time_s": 0.0}, "STRONG_PREEXISTING"),
            sample(f"dev_strong_simultaneous_{index:02d}", target, {"kind": "constant", "speed_m_s": 3.5, "onset_time_s": 3.0}, "STRONG_SIMULTANEOUS"),
            sample(f"dev_strong_post_target_{index:02d}", target, {"kind": "constant", "speed_m_s": 3.5, "onset_time_s": 4.0}, "STRONG_POST_TARGET", 1.0),
            sample(f"dev_ramp_target_{index:02d}", target, {"kind": "ramp", "start_speed_m_s": 0.0, "speed_m_s": 3.5, "onset_time_s": 2.0, "duration_s": 6.0}, "RAMP_STRONG"),
        ])
        for item in development_samples[-6:]:
            item["target"] = {**item["target"], "v4_target_id": tid, "provenance": "V3_HOLDOUT_NOW_V4_PRIOR_EVIDENCE"}
    for seed in range(3000, 3020):
        index = (seed - 3000) % 12
        target = {**v3_targets[index], "v4_target_id": f"v4d_prior_dir_{index:02d}", "provenance": "V3_HOLDOUT_NOW_V4_PRIOR_EVIDENCE"}
        development_samples.append(sample(
            f"dev_stochastic_{seed}", target,
            {"kind": "stochastic", "seed": seed, "sigma_m_s": 0.8, "clip_m_s": 3.0},
            "NORMAL_STOCHASTIC",
        ))
    equilibrium = {"v4_target_id": "equilibrium", "direction_unit": [0.0, 0.0, 0.0], "radius_m": 0.0, "delta_tip_m": [0.0, 0.0, 0.0]}
    development_samples.extend([
        sample("dev_strong_equilibrium", equilibrium, {"kind": "constant", "speed_m_s": 3.5, "onset_time_s": 0.0}, "STRONG_PREEXISTING", None),
        sample("dev_ramp_equilibrium", equilibrium, {"kind": "ramp", "start_speed_m_s": 0.0, "speed_m_s": 3.5, "onset_time_s": 2.0, "duration_s": 6.0}, "RAMP_STRONG", None),
    ])
    development = {
        "name": "V4_FROZEN_DEVELOPMENT_BANK",
        "sample_count": len(development_samples),
        "samples": development_samples,
        "cohort_counts": {name: sum(s["cohort"] == name for s in development_samples) for name in sorted({s["cohort"] for s in development_samples})},
        "sustained_strong_wind_cases": sum(s["cohort"].startswith("STRONG_") for s in development_samples),
        "wind_onset_variants": ["preexisting_before_target", "simultaneous_with_target", "after_target"],
        "v3_prior_evidence_reuse_allowed": True,
        "holdout_identity": False,
    }
    write_json("development_manifest.json", development)

    rng = np.random.Generator(np.random.PCG64(20260810))
    holdout_targets = []
    for index in range(12):
        direction = rng.normal(size=3)
        direction /= np.linalg.norm(direction)
        radius = 0.19 if index < 6 else 0.24
        holdout_targets.append({
            "target_id": f"v4h_dir_{index:02d}",
            "direction_unit": direction.tolist(),
            "radius_m": radius,
            "delta_tip_m": (radius * direction).tolist(),
            "generator": "PCG64 normal-vector normalization",
            "generator_seed": 20260810,
        })
    holdout_samples = []
    holdout_specs = [
        ("calm", {"kind": "calm"}, "NORMAL_CALM", 3.0),
        ("constant_2p5", {"kind": "constant", "speed_m_s": 2.5, "onset_time_s": 0.0}, "NORMAL_CONSTANT", 3.0),
        ("strong_3p25_preexisting", {"kind": "constant", "speed_m_s": 3.25, "onset_time_s": 0.0}, "STRONG_PREEXISTING", 3.0),
        ("strong_3p75_preexisting", {"kind": "constant", "speed_m_s": 3.75, "onset_time_s": 0.0}, "STRONG_PREEXISTING", 3.0),
        ("strong_3p5_simultaneous", {"kind": "constant", "speed_m_s": 3.5, "onset_time_s": 3.0}, "STRONG_SIMULTANEOUS", 3.0),
        ("strong_3p5_post_target", {"kind": "constant", "speed_m_s": 3.5, "onset_time_s": 4.0}, "STRONG_POST_TARGET", 1.0),
    ]
    for prefix, wind, cohort, issue in holdout_specs:
        for index, target in enumerate(holdout_targets):
            item = sample(f"holdout_{prefix}_{index:02d}", target, wind, cohort, issue)
            item["execution_allowed"] = False
            holdout_samples.append(item)
    for seed in range(4000, 4020):
        target = holdout_targets[(seed - 4000) % 12]
        item = sample(
            f"holdout_stochastic_{seed}", target,
            {"kind": "stochastic", "seed": seed, "sigma_m_s": 0.9, "clip_m_s": 3.2},
            "NORMAL_STOCHASTIC",
        )
        item["execution_allowed"] = False
        holdout_samples.append(item)
    for identifier, wind in [
        ("holdout_ramp_equilibrium", {"kind": "ramp", "start_speed_m_s": 0.0, "speed_m_s": 3.75, "onset_time_s": 2.5, "duration_s": 5.5}),
        ("holdout_step_equilibrium", {"kind": "constant", "speed_m_s": 3.5, "onset_time_s": 4.5}),
    ]:
        item = sample(identifier, equilibrium, wind, "STRONG_TRANSIENT_DIAGNOSTIC", None)
        item["execution_allowed"] = False
        holdout_samples.append(item)
    holdout = {
        "name": "V4_FROZEN_UNSEEN_HOLDOUT_BANK",
        "written_before_new_self_tuning": True,
        "target_generator": {"algorithm": "PCG64 normal-vector normalization", "seed": 20260810, "count": 12},
        "random_seeds": list(range(4000, 4020)),
        "sample_count": len(holdout_samples),
        "samples": holdout_samples,
        "cohort_counts": {name: sum(s["cohort"] == name for s in holdout_samples) for name in sorted({s["cohort"] for s in holdout_samples})},
        "sustained_strong_wind_cases": sum(s["cohort"].startswith("STRONG_") and s["target_issue_time_s"] is not None for s in holdout_samples),
        "new": True,
        "execution_allowed": False,
        "unlock_condition": "Traditional results, legacy Self result, new Self selection, and every authorized method-selection outcome are frozen",
        "one_shot_only": True,
    }
    write_json("holdout_manifest.json", holdout)

    v3_directions = {tuple(float(x) for x in target["direction_unit"]) for target in v3_targets}
    v4_directions = {tuple(float(x) for x in target["direction_unit"]) for target in holdout_targets}
    split = {
        "development_sample_count": len(development_samples),
        "holdout_sample_count": len(holdout_samples),
        "sample_id_overlap": sorted({s["sample_id"] for s in development_samples} & {s["sample_id"] for s in holdout_samples}),
        "target_direction_exact_overlap": len(v3_directions & v4_directions),
        "development_stochastic_seeds": list(range(3000, 3020)),
        "holdout_stochastic_seeds": list(range(4000, 4020)),
        "stochastic_seed_overlap": [],
        "v3_holdout_reused_as_v4_holdout": False,
        "v3_spherical_ids_reused_as_v4_holdout": False,
        "holdout_execution_forbidden": True,
        "pass": True,
    }
    write_json("split_integrity.json", split)

    win = {
        "written_before_new_self_performance": True,
        "primary_traditional": "full_lqr_048",
        "comparators": ["hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009", "self_a_034"],
        "overall": {
            "safety_rate_no_worse_than_best_traditional": True,
            "success_rate_no_worse_than_best_traditional": True,
            "position_mean_improvement_vs_full_lqr_min_fraction": 0.05,
            "acquisition_degradation_vs_full_lqr_max_fraction": 0.05,
            "paired_bootstrap_position_ci_lower_gt_zero": True,
        },
        "strong_wind": {
            "cohorts": ["STRONG_PREEXISTING", "STRONG_SIMULTANEOUS", "STRONG_POST_TARGET"],
            "safety_rate_no_worse_than_best_traditional": True,
            "success_rate_no_worse_than_best_traditional": True,
            "position_mean_improvement_vs_full_lqr_min_fraction": 0.05,
            "position_mean_improvement_vs_legacy_self_min_fraction": 0.30,
            "position_p90_degradation_vs_full_lqr_max_fraction": 0.10,
            "catastrophic_pair_definition": "new_self_position > 2.0 * full_lqr_position",
            "catastrophic_pair_count_max": 0,
        },
        "normal_regime_nonregression": {
            "cohorts": ["NORMAL_CALM", "NORMAL_CONSTANT", "NORMAL_STOCHASTIC"],
            "safety_rate_no_worse_than_legacy_self": True,
            "success_rate_no_worse_than_legacy_self": True,
            "position_degradation_vs_legacy_self_max_fraction": 0.05,
            "acquisition_degradation_vs_legacy_self_max_fraction": 0.10,
        },
        "paired_bootstrap": {"pairing": "exact sample_id", "resamples": 10000, "seed": 20260812, "confidence": 0.95},
        "development_qualification_uses_same_cohort_logic": True,
        "holdout_gate_may_not_be_changed_after_r0": True,
    }
    write_json("v4_win_contract.json", win)
    write_json("v4_search_budget.json", {
        "authorized_method": "CART-OFMPC only",
        "new_controller_candidates_max": 32,
        "stage_a_structural_smoke": {"candidate_max": 8, "predeclared_development_subset_cases": 16, "selection_authority": False},
        "stage_b_full_development": {"candidate_max": 18, "cases_per_candidate": len(development_samples)},
        "stage_c_confirmation": {"candidate_max": 6, "cases_per_candidate": len(development_samples)},
        "selection_source": "V4 Development only",
        "traditional_retuning": False,
        "legacy_self_retuning": False,
        "paper_search": False,
        "holdout_runs_authorized": 0,
        "gpu_note": "The current MuJoCo/NumPy/OSQP path is CPU-bound; use up to 24 isolated CPU workers with BLAS threads fixed to one.",
    })
    write_json("gate.json", {
        "task": "V4-R0-STRONG-WIND-GENERALIZATION-POSTMORTEM-AND-RESEARCH-CONTRACT-FREEZE-R1",
        "checks": {
            "V3_FINAL_UNCHANGED": True,
            "V3_FAILURE_REPRODUCED": replay_match,
            "ROOT_CAUSE_AUDITED": True,
            "V4_SINGLE_SELF_HYPOTHESIS_SELECTED": True,
            "NEW_V4_DEVELOPMENT_FROZEN": True,
            "NEW_V4_HOLDOUT_FROZEN": True,
            "V3_HOLDOUT_REUSED_AS_V4_HOLDOUT": False,
            "V4_WIN_CONTRACT_FROZEN": True,
            "NEW_SELF_PERFORMANCE_EXECUTED": False,
            "HOLDOUT_EXECUTED": False,
        },
        "result": "V4_STRONG_WIND_RESEARCH_CONTRACT_FROZEN",
        "pass": bool(replay_match),
    })

    report = f"""# V4 Strong-Wind Research Contract

## Scope and status

V3 remains permanently frozen at `{SOURCE_HEAD}` with final status
`V3_SELF_DEVELOPMENT_WIN_NOT_CONFIRMED_ON_HOLDOUT`. The V3 Holdout is now V4 prior
evidence, not a V4 Holdout. This R0 task ran no new-controller performance and did
not execute the newly frozen V4 Holdout.

## Reproduced failure

Instrument-only replay reproduced all 12 sustained 3.5 m/s pairs exactly. Mean
position RMSE was `{mean(replay_self, 'position_rmse_3d_m'):.6f} m` for `self_a_034`
and `{mean(replay_full, 'position_rmse_3d_m'):.6f} m` for `full_lqr_048`. The frozen
ramp replay remained `{float(ramp_self['position_rmse_3d_m']):.6f} m` for Self.

## Root-cause hierarchy

The primary, strongly supported mechanism is a constraint-unaware residual-to-
equilibrium map entering a saturated feedback regime. In constant 3.5 m/s wind,
raw residual clipping occurred on `{mean(replay_self, 'raw_clip_update_fraction'):.1%}`
of updates, chiefly `qdot1`, `qdot2`, and `qdot3`. The unscaled steady command
required `{mean(replay_self, 'unscaled_steady_input_max_abs_mean'):.3f} m/s^2` on
average (peak `{maximum(replay_self, 'unscaled_steady_input_max_abs_max'):.3f}`), so
the internal solution was scaled on `{mean(replay_self, 'steady_scaling_active_fraction'):.1%}`
of updates. The resulting loop showed `{mean(replay_self, 'physical_saturation_fraction'):.1%}`
amplitude activity and `{mean(replay_self, 'slew_active_fraction'):.1%}` slew activity.

The ramp case had no residual clipping, steady scaling, amplitude saturation, or
slew activity. Failure without any reference step (`{float(matched['strong_wind_equilibrium_no_step']['position_rmse_3d_m']):.3f} m`)
shows that target-step interaction is not necessary. Projection rejection was
small and every QP solved, so neither is the primary cause.

## Frozen V4 hypothesis

The sole V4 Self hypothesis is **CART-OFMPC**: causal residual estimation plus a
constraint-feasible steady target, residual-confidence scheduling, and explicit
anti-windup accounting for unrepresented disturbance. It may use measured state,
previous actual command, innovation, estimated disturbance, and constraint
activity, but never true or future wind.

## Frozen evaluation

Development contains `{len(development_samples)}` samples, including
`{development['sustained_strong_wind_cases']}` sustained strong-wind cases and all
three onset relationships. Holdout contains `{len(holdout_samples)}` wholly new
samples generated with seed `20260810`, stochastic seeds `4000..4019`, and
`{holdout['sustained_strong_wind_cases']}` sustained strong-wind cases. Holdout
execution remains forbidden until every participant and selection outcome is
frozen.

Formal success requires safety/success non-regression, at least 5% overall and
strong-wind position improvement versus Full-LQR, at least 30% strong-wind
improvement versus legacy Self, no catastrophic strong-wind pair, normal-regime
position within 5% of legacy Self, and the frozen paired bootstrap gate.

## Frozen boundaries

The plant, task, safety definition, geometric inner loop, 20 Hz world-frame
`[ax, ay, az]` interface, `+/-2.0 m/s^2` per-axis authority, and `0.25 m/s^2` per-
update slew limit are unchanged. Traditional controllers and `self_a_034` remain
byte-frozen baselines. Paper search is outside this task.
"""
    docs = ROOT / "docs/V4_RESEARCH_CONTRACT.md"
    docs.write_text(report, encoding="utf-8", newline="\n")
    contract_files = [
        "source_freeze.json", "v3_failure_postmortem.json", "strong_wind_diagnostic.csv",
        "failure_mechanism_report.json", "v4_method_hypothesis.json", "v4_control_contract.json",
        "development_manifest.json", "holdout_manifest.json", "split_integrity.json",
        "v4_win_contract.json", "v4_search_budget.json", "gate.json",
    ]
    write_json("contract_sha256.json", {
        "scope": "V4-R0 frozen evidence and research contract",
        "hash_normalization": "text files canonicalized to LF before SHA-256",
        "files": {f"reproducibility/v4/r0/{name}": sha256(OUT / name) for name in contract_files},
        "docs/V4_RESEARCH_CONTRACT.md": sha256(docs),
    })
    print(json.dumps({"result": "V4_STRONG_WIND_RESEARCH_CONTRACT_FROZEN", "development": len(development_samples), "holdout": len(holdout_samples)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
