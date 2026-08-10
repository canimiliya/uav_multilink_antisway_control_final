"""Build the pre-performance P2-R0G governance audit evidence.

This script is deliberately read-only with respect to controller evidence.  It
classifies frozen Development identities, audits the origin of competence v1,
and performs analytical authority checks.  It never executes Holdout or a
controller and it does not decide governance v2 thresholds from performance.
"""

from __future__ import annotations

import csv
import hashlib
import json
import subprocess
from collections import defaultdict
from pathlib import Path
from typing import Any

import numpy as np

from uav_sway.native_stack.case_semantics.resolver import NativeCaseResolver


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/native_stack/governance"
SOURCE_HEAD = "9d0264246408eced13b5c5029763443907610697"
BENCHMARK_TAG = "native-stack-benchmark-v1.1"
NOMINAL_WINDS = ("calm", "moderate", "stochastic")
CHALLENGE_WINDS = ("strong_sustained", "strong_transient", "ramp")
METHODS = {
    "native_pid_001": ROOT / "reproducibility/native_stack/r1r1/traditional/pid/development_results.csv",
    "native_full_lqr_006": ROOT / "reproducibility/native_stack/r1r1/traditional/full_lqr/development_results.csv",
    "native_task_lqr_004": ROOT / "reproducibility/native_stack/r1r1/traditional/task_lqr/development_results.csv",
}


def dump(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def rate(rows: list[dict[str, str]], key: str) -> float:
    return sum(row[key].lower() == "true" for row in rows) / len(rows)


def method_summary(method_id: str, path: Path) -> dict[str, Any]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream) if row["method_id"] == method_id]
    if len(rows) != 200:
        raise AssertionError(f"{method_id}: expected 200 rows, got {len(rows)}")
    cohorts = {
        "nominal": [row for row in rows if row["wind_kind"] in NOMINAL_WINDS],
        "challenge": [row for row in rows if row["wind_kind"] in CHALLENGE_WINDS],
    }
    by_wind: dict[str, Any] = {}
    for wind in (*NOMINAL_WINDS, *CHALLENGE_WINDS):
        subset = [row for row in rows if row["wind_kind"] == wind]
        by_wind[wind] = {
            "cases": len(subset),
            "success_count": sum(row["success"].lower() == "true" for row in subset),
            "success_rate": rate(subset, "success"),
            "safety_rate": rate(subset, "safe"),
        }
    by_task: dict[str, Any] = {}
    for task in ("setpoint", "smooth_trajectory"):
        subset = [row for row in rows if row["task_family"] == task]
        by_task[task] = {
            "cases": len(subset),
            "success_rate": rate(subset, "success"),
            "safety_rate": rate(subset, "safe"),
            "position_rmse_mean_m": float(np.mean([float(row["position_rmse_m"]) for row in subset])),
            "endpoint_position_error_mean_m": float(np.mean([float(row["endpoint_position_error_m"]) for row in subset])),
            "endpoint_velocity_error_mean_m_s": float(np.mean([float(row["endpoint_velocity_error_m_s"]) for row in subset])),
        }
    return {
        "method_id": method_id,
        "source": path.relative_to(ROOT).as_posix(),
        "all": {
            "cases": len(rows),
            "success_rate": rate(rows, "success"),
            "safety_rate": rate(rows, "safe"),
            "catastrophic_count": sum(row["catastrophic"].lower() == "true" for row in rows),
        },
        "nominal": {
            "cases": len(cohorts["nominal"]),
            "success_rate": rate(cohorts["nominal"], "success"),
            "safety_rate": rate(cohorts["nominal"], "safe"),
        },
        "challenge": {
            "cases": len(cohorts["challenge"]),
            "success_rate": rate(cohorts["challenge"], "success"),
            "safety_rate": rate(cohorts["challenge"], "safe"),
        },
        "by_wind": by_wind,
        "by_task_family": by_task,
    }


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    raw_development = json.loads((ROOT / "reproducibility/native_stack/r0/native_development_manifest.json").read_text(encoding="utf-8"))
    resolved_development = json.loads((ROOT / "reproducibility/native_stack/r0s/resolved_development_manifest.json").read_text(encoding="utf-8"))
    if raw_development["case_count"] != 200 or resolved_development["case_count"] != 200:
        raise AssertionError("Development identity count changed")

    counts: dict[str, int] = defaultdict(int)
    cross: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    for case in raw_development["cases"]:
        wind = case["wind_kind"]
        cohort = "nominal" if wind in NOMINAL_WINDS else "challenge"
        if wind not in (*NOMINAL_WINDS, *CHALLENGE_WINDS):
            raise AssertionError(f"unclassified wind: {wind}")
        counts[cohort] += 1
        counts[f"wind:{wind}"] += 1
        cross[cohort][case["task_family"]] += 1

    cohort_audit = {
        "version": "p2-r0g-cohort-audit-v1",
        "audit_source_head": SOURCE_HEAD,
        "benchmark_tag": BENCHMARK_TAG,
        "case_identity_changed": False,
        "classification_basis": "mission intent, wind physics, and engineering role; no controller result used",
        "nominal": {
            "wind_kinds": list(NOMINAL_WINDS),
            "role": "normal operational competence",
            "rationale": "calm is nominal; 1.5 m/s constant moderate wind is an ordinary operating disturbance; zero-mean 0.8 m/s OU stochastic wind tests ordinary variability and remains bounded by 3 m/s",
            "case_count": counts["nominal"],
            "task_family_counts": dict(cross["nominal"]),
        },
        "challenge": {
            "wind_kinds": list(CHALLENGE_WINDS),
            "role": "advanced robustness differentiation",
            "rationale": "3 m/s sustained, 3 m/s transient shock, and ramp-to-3 m/s profiles deliberately stress disturbance rejection and transient coordination",
            "case_count": counts["challenge"],
            "task_family_counts": dict(cross["challenge"]),
        },
        "wind_case_counts": {wind: counts[f"wind:{wind}"] for wind in (*NOMINAL_WINDS, *CHALLENGE_WINDS)},
        "balanced_100_100": counts["nominal"] == counts["challenge"] == 100,
    }
    dump(OUT / "cohort_audit.json", cohort_audit)

    origin = {
        "version": "p2-r0g-competence-origin-audit-v1",
        "source_contract": "reproducibility/native_stack/r1r1/protocol/competence_contract.json",
        "introducing_commit": "fe573ec0fb723d4d57325c05eec27e4a15b42cb1",
        "success_rate_min": 0.70,
        "frozen_before_performance": True,
        "claimed_inputs": ["2 m mission geometry", "0.5 m endpoint tolerance", "SafetyV2", "real-time periods"],
        "traceability": {
            "equation_or_derivation_present": False,
            "external_engineering_acceptance_requirement_present": False,
            "comparison_of_50_60_70_80_present": False,
            "controller_performance_used_to_set_v1": False,
        },
        "classification": {
            "PHYSICALLY_DERIVED_THRESHOLD": False,
            "ENGINEERING_REQUIREMENT": False,
            "HEURISTIC_RESEARCH_GATE": True,
        },
        "SUCCESS_70_GATE_PHYSICALLY_DERIVED": False,
        "conclusion": "The 70% number was preregistered, but the repository supplies no physical derivation or external engineering requirement that uniquely implies 0.70 rather than 0.50, 0.60, or 0.80.",
    }
    dump(OUT / "competence_origin_audit.json", origin)

    resolver = NativeCaseResolver()
    max_reference_acceleration = 0.0
    max_reference_speed = 0.0
    target_distances = []
    vertical = []
    for identity in raw_development["cases"]:
        case = resolver.resolve(identity)
        signals = resolver.canonical_signals(case)
        max_reference_acceleration = max(max_reference_acceleration, float(np.max(np.linalg.norm(signals["acceleration"], axis=1))))
        max_reference_speed = max(max_reference_speed, float(np.max(np.linalg.norm(signals["velocity"], axis=1))))
        delta = np.asarray(case.target["displacement_world_m"], dtype=float)
        target_distances.append(float(np.linalg.norm(delta)))
        vertical.append(float(delta[2]))

    total_mass = 9.74 + 1.0 + 2.5
    gravity = 9.81
    hover = total_mass * gravity
    thrust_max = 285.74568
    tilt_limit_rad = np.deg2rad(35.0)
    thrust_for_tilted_hover = hover / np.cos(tilt_limit_rad)
    horizontal_force_at_tilt = thrust_for_tilted_hover * np.sin(tilt_limit_rad)
    static_projected_area = 0.98 * 0.48 + 5 * (0.5 * 0.05) + 0.45 * 0.14
    weighted_area = 1.0 * (0.98 * 0.48) + 1.2 * 5 * (0.5 * 0.05) + 1.05 * (0.45 * 0.14)
    wind_force_3 = 0.5 * 1.225 * weighted_area * 3.0**2
    wind_force_12 = 0.5 * 1.225 * weighted_area * 12.0**2
    physical = {
        "version": "p2-r0g-physical-feasibility-v1",
        "analysis_scope": "Development identities and frozen physical limits only; no Holdout and no controller selection",
        "mass": {"airframe_kg": 9.74, "links_kg": 1.0, "cutter_kg": 2.5, "total_kg": total_mass},
        "hover": {
            "hover_thrust_N": hover,
            "max_thrust_N": thrust_max,
            "absolute_reserve_N": thrust_max - hover,
            "max_to_hover_ratio": thrust_max / hover,
            "hover_fraction_of_max": hover / thrust_max,
        },
        "torque": {
            "limits_Nm": [25.0, 25.0, 12.0],
            "airframe_inertia_diagonal_kg_m2": [0.655826666667, 0.966532666667, 1.24834333333],
            "minimum_axis_angular_acceleration_rad_s2": 12.0 / 1.24834333333,
            "interpretation": "substantial attitude authority; linked-load coupling still requires nonlinear coordination",
        },
        "wind": {
            "strong_speed_m_s": 3.0,
            "official_airframe_resistance_ceiling_m_s": 12.0,
            "conservative_static_projected_area_m2": static_projected_area,
            "drag_weighted_area_m2": weighted_area,
            "static_drag_at_3_m_s_N": wind_force_3,
            "static_drag_at_12_m_s_N": wind_force_12,
            "tilted_hover_horizontal_force_at_35_deg_N": horizontal_force_at_tilt,
            "3_m_s_drag_fraction_of_tilt_authority": wind_force_3 / horizontal_force_at_tilt,
            "classification": "PHYSICALLY_REASONABLE",
        },
        "reference": {
            "max_target_distance_m": max(target_distances),
            "min_target_distance_m": min(target_distances),
            "vertical_displacement_range_m": [min(vertical), max(vertical)],
            "mission_duration_s": 12.0,
            "max_smooth_reference_speed_m_s": max_reference_speed,
            "max_smooth_reference_acceleration_m_s2": max_reference_acceleration,
            "horizontal_acceleration_at_35_deg_m_s2": gravity * np.tan(tilt_limit_rad),
            "classification": "AGGRESSIVE_BUT_FEASIBLE",
        },
        "link_and_cutter_load": {
            "suspended_mass_kg": 3.5,
            "chain_length_m": 2.5,
            "interpretation": "material underactuated swing burden but far below vertical thrust reserve; it changes controller difficulty, not basic endpoint reachability",
        },
        "pre_oracle_conclusion": "The frozen envelope is demanding but has large static force and thrust reserves. Dynamic reachability is delegated to the preregistered diagnostic oracle.",
    }
    dump(OUT / "physical_feasibility.json", physical)

    existing = {method: method_summary(method, path) for method, path in METHODS.items()}
    dump(OUT / "existing_development_evidence.json", {
        "version": "p2-r0g-existing-evidence-v1",
        "selection_authority": "NONE",
        "performance_used_to_define_cohorts": False,
        "methods": existing,
    })

    holdout_raw = ROOT / "reproducibility/native_stack/r0/native_holdout_manifest.json"
    holdout_resolved = ROOT / "reproducibility/native_stack/r0s/resolved_holdout_manifest.json"
    resolved_holdout = json.loads(holdout_resolved.read_text(encoding="utf-8"))
    fingerprints = [case["case_semantic_fingerprint"] for case in resolved_holdout["cases"]]
    holdout = {
        "identity_manifest_sha256": sha256(holdout_raw),
        "resolved_manifest_sha256": sha256(holdout_resolved),
        "fingerprint_count": len(fingerprints),
        "unique_fingerprint_count": len(set(fingerprints)),
        "checks_performed": ["file sha256", "serialized semantic fingerprint uniqueness"],
        "performance_fields_read": False,
        "execution_allowed": False,
        "executed": False,
        "authoritative_runs": 0,
        "compromised": False,
    }
    dump(OUT / "holdout_status.json", holdout)

    changed = subprocess.run(
        ["git", "diff", "--name-only", SOURCE_HEAD, "--"], cwd=ROOT,
        check=True, capture_output=True, text=True,
    ).stdout.splitlines()
    protected_prefixes = (
        "reproducibility/frozen/", "src/uav_sway/native_stack/runner.py",
        "src/uav_sway/native_stack/actuation.py", "src/uav_sway/native_stack/case_semantics/",
        "reproducibility/native_stack/r0/native_", "reproducibility/native_stack/r0s/resolved_",
    )
    protected_changed = [path for path in changed if path.startswith(protected_prefixes)]
    dump(OUT / "protected_evidence_audit.json", {
        "audit_source_head": SOURCE_HEAD,
        "changed_paths_at_audit_time": changed,
        "protected_paths_changed": protected_changed,
        "protected_scope_clean": not protected_changed,
        "historical_statuses_preserved": {
            "P2-R1R1": "BLOCKED_P2_TRADITIONAL_COMPETENCE",
            "P2-R1R2": "P2_NATIVE_TRADITIONAL_RECOVERY_FAILED",
        },
    })

    dump(OUT / "oracle_protocol.json", {
        "version": "p2-r0g-noncausal-oracle-v1",
        "frozen_before_oracle_execution": True,
        "purpose": "diagnose physical endpoint reachability, never controller competence",
        "split": "Development only",
        "case_count": 200,
        "truth_access": ["full nonlinear state", "full future reference", "true applied aerodynamic disturbance", "perfect frozen model"],
        "physical_constraints_unchanged": True,
        "interface": "WrenchCommand[T,tau_x,tau_y,tau_z]",
        "method": "fixed-gain full-state computed-force tracking with exact wind-force cancellation and offline minimum-jerk shaping of setpoint steps",
        "parameter_basis": "critical-damping pole placement and frozen 35 degree attitude/actuator authority; no candidate search",
        "parameters": {
            "setpoint_transition_s": 4.0,
            "translation_natural_frequency_rad_s": 1.2,
            "translation_damping_ratio": 1.0,
            "tip_correction_gain": 0.65,
            "tip_velocity_correction_s": 0.30,
            "tip_correction_limit_m": 0.75,
            "acceleration_limit_m_s2": 4.0,
            "attitude_natural_frequency_rad_s": 8.0,
            "attitude_damping_ratio": 1.0,
            "outer_rate_hz": 200,
            "inner_rate_hz": 1000,
        },
        "search_or_tuning_allowed": False,
        "selection_authority": "NONE",
        "paper_or_satc_comparison_authority": "NONE",
        "holdout_allowed": False,
    })


if __name__ == "__main__":
    main()
