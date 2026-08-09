"""Freeze the V7 Kang-2026 benchmark before any V7 performance."""

from __future__ import annotations

import hashlib
import json
import subprocess
from collections import Counter
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v7/r0"
SOURCE_TAG = "v6-research-final-2026-08-09"
SOURCE_HEAD = "6849c611c1361f5a88527753a5d1cb17dea3b76d"
DEV_SEED = 20260821
HOLDOUT_SEED = 20260822
DEV_BOOTSTRAP_SEED = 20260823
HOLDOUT_BOOTSTRAP_SEED = 20260824
PAPER_SHA256 = "1a2236debef68536d8ba3fb1b3e995f2d831eb7db08a21316d679428d6e45546"


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
    ranges = {"aligned": (0.58, 0.92), "opposed": (-0.92, -0.58), "cross": (-0.18, 0.18)}
    result = []
    for stratum, (low, high) in ranges.items():
        for index in range(count_per_stratum):
            x = float(rng.uniform(low, high))
            angle = float(rng.uniform(0.0, 2.0 * np.pi))
            radial = float(np.sqrt(1.0 - x * x))
            vector = np.asarray([x, radial * np.cos(angle), radial * np.sin(angle)])
            radius = (0.17, 0.21, 0.24)[index % 3]
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


def sample(sample_id: str, cohort: str, target: dict, wind: dict, issue: float, allowed: bool) -> dict:
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
    count_per_stratum = 6 if holdout else 8
    targets = directions(seed, f"{prefix}_dir", count_per_stratum)
    allowed = not holdout
    rows: list[dict] = []
    for index, target in enumerate(targets):
        rows.append(sample(f"{prefix}_calm_{index:02d}", "NORMAL_CALM", target, {"kind": "calm", "direction_sign": 0}, 3.0, allowed))
        sign = 1 if index % 2 == 0 else -1
        rows.append(sample(
            f"{prefix}_moderate_{index:02d}", "NORMAL_CONSTANT", target,
            {"kind": "constant", "speed_m_s": 2.45 if holdout else 2.20, "direction_sign": sign, "onset_time_s": 0.0}, 3.0, allowed,
        ))
    groups = {name: [t for t in targets if t["canonical_stratum"] == name] for name in ("aligned", "opposed", "cross")}
    specs = [
        ("preexisting", "STRONG_PREEXISTING", 0.0, 3.0),
        ("before", "STRONG_NEAR_SIMULTANEOUS", 2.58, 3.0),
        ("after", "STRONG_NEAR_SIMULTANEOUS", 3.42, 3.0),
    ] if not holdout else [
        ("preexisting_shifted", "STRONG_PREEXISTING", 0.27, 3.18),
        ("before_shifted", "STRONG_NEAR_SIMULTANEOUS", 2.49, 3.0),
        ("after_shifted", "STRONG_NEAR_SIMULTANEOUS", 3.51, 3.0),
    ]
    for label, cohort, onset, issue in specs:
        for stratum, group in groups.items():
            for local, target in enumerate(group):
                sign = 1 if local % 2 == 0 else -1
                rows.append(sample(
                    f"{prefix}_strong_{label}_{stratum}_{local:02d}", cohort, mirrored(target, sign),
                    {"kind": "constant", "speed_m_s": 3.75 if holdout else 3.55, "direction_sign": sign, "onset_time_s": onset}, issue, allowed,
                ))
    stochastic_count = 10 if holdout else 12
    stochastic_start = 9000 if holdout else 8000
    for offset in range(stochastic_count):
        target = targets[(5 * offset + 2) % len(targets)]
        sign = 1 if offset % 2 == 0 else -1
        wind_seed = stochastic_start + offset
        rows.append(sample(
            f"{prefix}_stochastic_{wind_seed}", "NORMAL_STOCHASTIC", target,
            {"kind": "stochastic", "seed": wind_seed, "sigma_m_s": 0.96 if holdout else 0.90, "clip_m_s": 3.2, "direction_sign": sign}, 3.0, allowed,
        ))
    for stratum, group in groups.items():
        for local, target in enumerate(group[:4]):
            sign = 1 if local % 2 == 0 else -1
            rows.append(sample(
                f"{prefix}_ramp_{stratum}_{local:02d}", "RAMP_STRONG", mirrored(target, sign),
                {"kind": "ramp", "start_speed_m_s": 0.0, "speed_m_s": 3.75 if holdout else 3.55,
                 "direction_sign": sign, "onset_time_s": 2.31 if holdout else 1.73, "duration_s": 5.37 if holdout else 6.27}, 3.0, allowed,
            ))
    expected = 112 if holdout else 144
    if len(rows) != expected:
        raise RuntimeError(f"sample count drift: {len(rows)} != {expected}")
    return {
        "name": "V7_FROZEN_HOLDOUT_BANK" if holdout else "V7_FROZEN_DEVELOPMENT_BANK",
        "generator_seed": seed,
        "sample_count": len(rows),
        "samples": rows,
        "cohort_counts": dict(sorted(Counter(row["cohort"] for row in rows).items())),
        "directional_counts": dict(sorted(Counter(row["directional_stratum"] for row in rows if row["directional_stratum"] != "not_applicable").items())),
        "performance_accessed_at_freeze": False,
        "execution_allowed": allowed,
    }


def main() -> int:
    if git("branch", "--show-current") != "research-v7":
        raise RuntimeError("V7 contract must be created on research-v7")
    if git("rev-parse", f"{SOURCE_TAG}^{{}}") != SOURCE_HEAD or git("rev-parse", "HEAD") != SOURCE_HEAD:
        raise RuntimeError("V7 must begin exactly at frozen V6")
    development = make_bank(DEV_SEED, "v7d", False)
    holdout = make_bank(HOLDOUT_SEED, "v7h", True)
    write("development_manifest.json", development)
    write("holdout_manifest.json", holdout)
    write("paper_source_audit.json", {
        "publication": {
            "authors": ["Junjie Kang", "Jinjun Shan"],
            "year": 2026,
            "title": "Robust control of aerial cable-suspended payload transportation via fully actuated system approach",
            "venue": "Control Engineering Practice 170, 106837",
            "doi": "10.1016/j.conengprac.2026.106837",
            "license": "CC BY-NC-ND 4.0",
        },
        "primary_source_path": "C:/Users/Administrator/Downloads/1-s2.0-S096706612600081X-main.pdf",
        "primary_source_sha256": PAPER_SHA256,
        "primary_source_pages": 9,
        "controller_equations_complete": True,
        "parameter_information_complete": True,
        "equation_inventory": {
            "plant": "Eqs. (1)-(9)", "coordinate_transform": "Eqs. (10)-(13)",
            "outer_fas_dob": "Eqs. (14)-(20)", "outer_stability": "Eqs. (21)-(28)",
            "inner_fas_dob": "Eqs. (29)-(44)", "disturbances": "Eqs. (45)-(46)",
        },
        "required_sensors": ["UAV position/velocity", "UAV attitude/rate", "payload swing angles/rates"],
        "required_states": ["UAV translation", "two cable swing angles", "UAV attitude"],
        "original_controller_output": "desired force vector outer loop and body torque inner loop",
        "original_payload_model": "point payload on a fixed-length lightweight rigid cable with two swing angles",
        "original_actuator_assumptions": "quadrotor total thrust and three body torques",
        "original_disturbance_model": "matched additive outer translational and inner attitude disturbances",
        "v7_fairness_mapping": "outer-loop structure only; output restricted to common world-frame acceleration command",
        "performance_executed": False,
    })
    write("research_contract.json", {
        "task": "V7-KANG2026-FAS-PAPER-STRONG-BASELINE-END-TO-END-R1",
        "source_tag": SOURCE_TAG, "source_head": SOURCE_HEAD, "branch": "research-v7",
        "paper_family": "KANG2026-FAS-DOB-ADAPTED-5LINK",
        "primary_goal": "validate a recent Paper baseline that is stronger than frozen Traditional controllers",
        "frozen": {
            "plant": "MuJoCo five-link plant", "metrics_and_safety": True, "geometric_inner_loop": True,
            "command_frame": "world [ax, ay, az]", "acceleration_limit_m_s2_per_axis": 2.0,
            "slew_limit_m_s2_per_update": 0.25, "outer_rate_hz": 20.0,
            "traditional": ["hybrid_x007_y041_z041", "full_lqr_048", "task_lqr_009"],
            "primary_traditional": "full_lqr_048", "self_reference": "satc_b_027",
        },
        "paper_selection_uses_satc": False,
        "satc_tuning": "FORBIDDEN", "traditional_tuning": "FORBIDDEN",
        "holdout_policy": "one shot only after a qualified Paper candidate is permanently frozen",
        "development_failure_action": "KANG2026_ADAPTATION_NOT_STRONGER_THAN_TRADITIONAL; Holdout not executed",
        "forbidden": ["V1-V6 mutation", "authority expansion", "plant mutation", "metric change", "Paper replacement", "Holdout access before Paper freeze"],
        "v7_performance_executed": False, "v7_holdout_executed": False,
    })
    write("adaptation_contract.json", {
        "adaptation_name": "KANG2026-FAS-DOB-ADAPTED-5LINK", "exact_reproduction": False,
        "preserved": ["reduced-order FAS coordinate", "virtual-constraint swing coupling", "cascade outer-loop structure", "payload swing suppression", "causal finite-time disturbance observer"],
        "adapted": {
            "payload": "point payload cable direction -> UAV-to-cutter-COM equivalent direction",
            "dimension": "two cable angles -> 3D equivalent swing displacement/rate",
            "input": "desired force -> world-frame acceleration command",
            "inner_loop": "paper torque loop -> frozen common geometric inner loop",
        },
        "omitted": ["paper-specific rotor thrust/torque allocation", "paper inner-loop torque DOB", "true disturbance inputs"],
        "causal_states_only": True, "physical_plant_modified": False, "controller_authority_expanded": False,
    })
    write("search_contract.json", {
        "maximum_unique_configurations": 96,
        "rounds": [
            {"name": "stage_a", "unique": 24, "samples_each": 24, "role": "implementation and coarse stability screen", "selection_authority": False},
            {"name": "stage_b", "unique": 48, "samples_each": 144, "role": "full Development qualification", "selection_authority": True},
            {"name": "stage_c", "unique": 24, "samples_each": 144, "role": "predeclared coordinate refinement", "selection_authority": True},
        ],
        "search_space": {
            "position_gain": [0.8, 1.4, 2.2, 3.2], "velocity_gain": [0.8, 1.3, 2.0, 2.8],
            "virtual_constraint_gain": [0.0, 0.35, 0.7, 1.1], "virtual_length_scale": [0.5, 0.8, 1.1],
            "observer_lambda1": [0.6, 1.0, 1.6, 2.4], "observer_lambda2": [0.4, 0.8, 1.4, 2.2],
            "observer_rho": [0.5, 1.0, 1.6], "observer_clip_m_s2": [0.4, 0.8, 1.2],
        },
        "selection_rule": "all gates, then gate count, strong P90, position, acquisition, orientation, effort, runtime, candidate id",
        "candidate_97_forbidden": True, "satc_not_used_for_selection": True,
    })
    write("win_contract.json", {
        "primary_traditional": "full_lqr_048", "best_traditional_fields": ["safety", "success"],
        "development_and_holdout": {
            "safety_no_worse_best_traditional": True, "success_no_worse_best_traditional": True,
            "position_improvement_vs_full_min": 0.05, "bootstrap_lower_gt_zero": True,
            "strong_p90_degradation_vs_full_max": 0.10, "catastrophic_pair_max": 0,
            "meaningful_extra": ["acquisition improvement >=5%", "strong position improvement >=10%", "orientation improvement >=10%", "effort improvement >=10%"],
        },
        "paper_vs_satc_is_gate": False,
    })
    write("statistical_protocol.json", {
        "pairing": "exact sample_id", "resamples": 10000,
        "development_bootstrap_seed": DEV_BOOTSTRAP_SEED, "holdout_bootstrap_seed": HOLDOUT_BOOTSTRAP_SEED,
        "ci_quantiles": [0.025, 0.975], "seed_hunting_forbidden": True,
    })
    old_manifests = sorted(ROOT.glob("reproducibility/v*/**/*holdout*manifest*.json"))
    old_ids: set[str] = set()
    old_hashes = {}
    for path in old_manifests:
        payload = json.loads(path.read_text(encoding="utf-8"))
        rows = payload.get("samples", payload.get("execution_samples", []))
        old_ids.update(str(row.get("sample_id", "")) for row in rows)
        old_hashes[path.relative_to(ROOT).as_posix()] = digest(path)
    dev_ids = {row["sample_id"] for row in development["samples"]}
    holdout_ids = {row["sample_id"] for row in holdout["samples"]}
    write("split_integrity.json", {
        "development_holdout_ids_disjoint": not (dev_ids & holdout_ids),
        "v7_holdout_old_holdout_ids_disjoint": not (holdout_ids & old_ids),
        "development_seed": DEV_SEED, "holdout_seed": HOLDOUT_SEED,
        "development_stochastic_seeds": list(range(8000, 8012)), "holdout_stochastic_seeds": list(range(9000, 9010)),
        "old_holdout_manifest_sha256": old_hashes, "holdout_execution_allowed": False,
    })
    protected = ["v2", "v3", "v4", "v5", "v6"]
    write("source_freeze.json", {
        "source_tag_type": git("cat-file", "-t", SOURCE_TAG), "source_head": SOURCE_HEAD,
        "protected_trees": {name: git("rev-parse", f"{SOURCE_TAG}:reproducibility/{name}") for name in protected},
        "traditional_and_satc_files": {
            name: digest(ROOT / name) for name in [
                "reproducibility/v3/r1r1/pid_freeze.json", "reproducibility/v3/r1/full_lqr_freeze.json",
                "reproducibility/v3/r1/task_lqr_freeze.json", "reproducibility/v5/self/self_freeze.json",
            ]
        },
        "v1_v6_read_only": True,
    })
    contract_files = sorted(path for path in OUT.glob("*.json") if path.name != "contract_sha256.json")
    write("contract_sha256.json", {path.name: digest(path) for path in contract_files})
    print(json.dumps({"development_samples": 144, "holdout_samples": 112, "paper_budget": 96, "holdout_execution_allowed": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
