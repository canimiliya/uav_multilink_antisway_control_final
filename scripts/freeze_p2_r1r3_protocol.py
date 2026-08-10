"""Freeze the pre-performance protocol for P2-R1R3 Governance-v2 qualification."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "reproducibility/native_stack/r1r3"
PROTOCOL = BASE / "protocol"
SOURCE_TAG = "native-stack-benchmark-v1.2-governance"
SOURCE_HEAD = "88c3aef081fabab44d174bc8afd234ae634af3da"
R1R1 = "046c64eb8d364a58e5af5425df2f88b95c73b261"
R1R2 = "9d0264246408eced13b5c5029763443907610697"


def write(name: str, payload: object) -> None:
    path = PROTOCOL / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stage_a_cases(manifest: dict) -> list[str]:
    cases = manifest["cases"]
    selected: list[str] = []
    # Six per nominal wind/task stratum: near and far for all three frozen wind
    # directions.  This gives 36 nominal cases with distance/direction balance.
    for wind in ("calm", "moderate", "stochastic"):
        for task in ("setpoint", "smooth_trajectory"):
            stratum = [case for case in cases if case["identity"]["wind_kind"] == wind
                       and case["identity"]["task_family"] == task]
            for direction in ("aligned", "cross", "opposed"):
                values = [case for case in stratum if case["identity"]["wind_direction"] == direction]
                values.sort(key=lambda case: sum(v * v for v in case["target"]["displacement_world_m"]))
                selected.extend((values[0]["sample_id"], values[-1]["sample_id"]))
    # Six challenge safety probes: one deterministic mid-distance case per
    # challenge wind/task stratum, with directions rotated across the strata.
    for index, wind in enumerate(("strong_sustained", "strong_transient", "ramp")):
        for task_index, task in enumerate(("setpoint", "smooth_trajectory")):
            direction = ("aligned", "cross", "opposed")[(index + task_index) % 3]
            values = [case for case in cases if case["identity"]["wind_kind"] == wind
                      and case["identity"]["task_family"] == task
                      and case["identity"]["wind_direction"] == direction]
            values.sort(key=lambda case: sum(v * v for v in case["target"]["displacement_world_m"]))
            selected.append(values[len(values) // 2]["sample_id"])
    if len(selected) != 42 or len(set(selected)) != 42:
        raise AssertionError("Stage-A manifest must contain 42 unique cases")
    return selected


def common_parameters() -> dict:
    return {
        "attitude_wn": 4.0, "attitude_zeta": 0.95,
        "integral_limit": 1.2, "acceleration_limit": 5.0,
        "acceleration_slew_per_s": 12.0,
        "reference_velocity_gain": 1.0, "reference_acceleration_gain": 1.0,
        "tip_correction_kp": 0.12, "tip_correction_kd": 0.04,
        "tip_correction_limit_m": 0.30,
        "swing_angle_gain": 0.01, "swing_rate_gain": 0.05,
        "constraint_margin": 0.88,
    }


def pid_registry() -> list[dict]:
    # Deliberately centred on native_pid_001, with independent horizontal and
    # vertical authority, stronger terminal regulation, and bounded anti-windup.
    designs = [
        (.16,.16,.16,.60,.60,.60,.04,.04,.04,.08,.03,.20),
        (.20,.20,.18,.65,.65,.60,.05,.05,.04,.10,.04,.25),
        (.24,.24,.20,.70,.70,.65,.06,.06,.05,.12,.04,.30),
        (.28,.28,.22,.75,.75,.70,.07,.07,.05,.15,.05,.35),
        (.32,.32,.24,.80,.80,.75,.08,.08,.06,.18,.06,.40),
        (.36,.36,.26,.85,.85,.80,.09,.09,.06,.22,.07,.45),
        (.40,.40,.28,.90,.90,.85,.10,.10,.07,.25,.08,.50),
        (.44,.44,.30,.95,.95,.90,.11,.11,.08,.28,.09,.55),
        (.22,.30,.20,.70,.80,.65,.06,.07,.05,.14,.05,.32),
        (.30,.22,.20,.80,.70,.65,.07,.06,.05,.14,.05,.32),
        (.26,.34,.24,.75,.90,.75,.07,.09,.06,.18,.06,.38),
        (.34,.26,.24,.90,.75,.75,.09,.07,.06,.18,.06,.38),
        (.18,.26,.28,.65,.75,.85,.05,.06,.08,.12,.04,.30),
        (.26,.18,.28,.75,.65,.85,.06,.05,.08,.12,.04,.30),
        (.30,.38,.32,.85,1.00,.95,.08,.10,.09,.20,.07,.45),
        (.38,.30,.32,1.00,.85,.95,.10,.08,.09,.20,.07,.45),
        (.24,.24,.34,.72,.72,1.00,.06,.06,.10,.16,.05,.36),
        (.30,.30,.38,.82,.82,1.10,.08,.08,.12,.20,.06,.42),
        (.36,.36,.42,.92,.92,1.20,.10,.10,.14,.24,.08,.48),
        (.42,.42,.46,1.02,1.02,1.30,.12,.12,.16,.28,.10,.55),
        (.28,.36,.28,.78,.92,.88,.08,.10,.08,.24,.08,.50),
        (.36,.28,.28,.92,.78,.88,.10,.08,.08,.24,.08,.50),
        (.34,.42,.36,.90,1.08,1.05,.10,.12,.11,.30,.10,.60),
        (.42,.34,.36,1.08,.90,1.05,.12,.10,.11,.30,.10,.60),
    ]
    registry = []
    for index, values in enumerate(designs):
        kp = values[0:3]; kd = values[3:6]; ki = values[6:9]
        parameters = common_parameters()
        parameters.update({
            "kp": list(kp), "kd": list(kd), "ki": list(ki),
            "tip_correction_kp": values[9], "tip_correction_kd": values[10],
            "tip_correction_limit_m": values[11],
            "integral_limit": 0.8 + 0.2 * (index % 4),
            "acceleration_slew_per_s": (10.0, 14.0, 18.0)[index % 3],
        })
        registry.append({"method_id": f"r1r3_pid_a_{index:03d}", "family": "native_pid",
                         "outer_rate_hz": 100, "inner_rate_hz": 500, "parameters": parameters})
    return registry


def lqi_registry() -> list[dict]:
    designs = [
        (0.15,0.55,0.03), (0.20,0.65,0.04), (0.25,0.75,0.05), (0.30,0.85,0.06),
        (0.35,0.95,0.08), (0.40,1.05,0.10), (0.45,1.15,0.12), (0.50,1.25,0.14),
        (0.24,0.70,0.08), (0.28,0.80,0.10), (0.32,0.90,0.12), (0.36,1.00,0.14),
        (0.20,0.80,0.05), (0.28,0.95,0.07), (0.36,1.10,0.09), (0.44,1.25,0.11),
        (0.30,0.70,0.04), (0.38,0.85,0.06), (0.46,1.00,0.08), (0.54,1.15,0.10),
        (0.26,0.90,0.10), (0.34,1.05,0.12), (0.42,1.20,0.14), (0.50,1.35,0.16),
    ]
    registry = []
    for index, (natural_gain, damping_gain, integral_gain) in enumerate(designs):
        # CARE weights, not hand-entered feedback gains.  Axis scaling permits
        # the vertical channel to be stronger without changing family identity.
        parameters = common_parameters()
        parameters.update({
            "kp": [0.0, 0.0, 0.0], "kd": [0.0, 0.0, 0.0], "ki": [0.0, 0.0, 0.0],
            "q_position": [natural_gain**2, natural_gain**2, (1.15*natural_gain)**2],
            "q_velocity": [damping_gain**2, damping_gain**2, (1.10*damping_gain)**2],
            "q_integral": [integral_gain**2, integral_gain**2, (1.10*integral_gain)**2],
            "r_acceleration": [1.0, 1.0, 1.0],
            "tip_correction_kp": (0.10, 0.16, 0.22, 0.28)[index % 4],
            "tip_correction_kd": (0.04, 0.06, 0.08)[index % 3],
            "tip_correction_limit_m": (0.30, 0.40, 0.50)[index % 3],
            "integral_limit": (0.8, 1.0, 1.2, 1.4)[index % 4],
            "acceleration_slew_per_s": (10.0, 14.0, 18.0)[index % 3],
        })
        registry.append({"method_id": f"r1r3_lqi_a_{index:03d}", "family": "native_full_lqr",
                         "outer_rate_hz": 100, "inner_rate_hz": 500, "parameters": parameters})
    return registry


def main() -> None:
    manifest_path = ROOT / "reproducibility/native_stack/r0s/resolved_development_manifest.json"
    governance_path = ROOT / "reproducibility/native_stack/governance/competence_governance_v2.json"
    cohort_path = ROOT / "reproducibility/native_stack/governance/cohort_role_definition.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    case_ids = stage_a_cases(manifest)

    write("research_contract.json", {
        "task": "P2-R1R3-GOVERNANCE-V2-STRONG-TRADITIONAL-QUALIFICATION-AND-SATC-FREEZE-R1",
        "source_tag": SOURCE_TAG, "source_head": SOURCE_HEAD,
        "branch": "research/p2-baselines-governance-v2",
        "benchmark_changed": False, "governance_changed": False, "mission_changed": False,
        "success_definition_changed": False, "holdout_execution_allowed": False,
        "holdout_performance_access_allowed": False, "paper_search": False,
        "traditional_minimum": 2, "satc_search_gate": "competent_traditional_count >= 2",
        "stop_on_traditional_failure": True, "stop_on_satc_not_advanced": True,
    })
    write("search_contract.json", {
        "pid": {"stage_a_max": 32, "stage_b_full_200_max": 8, "local_max": 24, "total_max": 64},
        "full_lqr_lqi": {"stage_a_initial": 24, "stage_b_full_200_max": 8, "local_max": 24},
        "task_lqr": {"optional": True, "initial_configs": 0, "maximum_configs": 24},
        "satc_native": {"enabled_only_after_two_traditional_pass": True, "maximum_new_configs": 96},
        "stop_first_confirmed_pass": True,
    })
    write("selection_contract.json", {
        "stage_a_order": ["catastrophic_count_ascending", "safety_rate_descending",
                          "nominal_success_rate_descending", "endpoint_error_ascending", "nominal_rmse_ascending"],
        "stage_b_count_per_family": 6, "qualification_cases": 200,
        "confirmation": "independent second authoritative 200-case execution of the selected exact spec",
        "primary_order": ["all_safety_rate", "nominal_success_rate", "nominal_task_balance",
                          "nominal_rmse", "challenge_robustness", "physical_effort", "runtime"],
        "future_paper_may_replace_primary": False,
    })
    write("governance_reference.json", {
        "version": "native-stack-competence-governance-v2", "changed": False,
        "competence_governance_path": str(governance_path.relative_to(ROOT)).replace("\\", "/"),
        "competence_governance_sha256": sha256(governance_path),
        "cohort_definition_path": str(cohort_path.relative_to(ROOT)).replace("\\", "/"),
        "cohort_definition_sha256": sha256(cohort_path),
        "resolved_development_manifest_sha256": sha256(manifest_path),
    })
    write("candidate_import_audit.json", {
        "old_results_imported_as_new_results": False,
        "reference_heads": {"R1R1": R1R1, "R1R2": R1R2},
        "imported_files": [
            {"source_sha": R1R2, "path": "src/uav_sway/native_stack/r1r1_controllers.py"},
            {"source_sha": R1R2, "path": "src/uav_sway/native_stack/r1r1_evaluation.py"},
            {"source_sha": R1R2, "path": "src/uav_sway/native_stack/r1r2_controllers.py"},
            {"source_sha": R1R2, "path": "src/uav_sway/native_stack/r1r2_evaluation.py"},
        ],
        "new_formal_results_require_authoritative_rerun": True,
    })
    write("stage_a_manifest.json", {
        "case_count": len(case_ids), "case_ids": case_ids,
        "nominal_case_count": 36, "challenge_safety_probe_count": 6,
        "selection": "near/far per nominal wind-task-direction plus deterministic mid-distance challenge probes",
    })
    write("statistical_protocol.json", {
        "paired_case_identity_required": True, "bootstrap_replicates": 10000,
        "bootstrap_seed": 20260810, "cohorts": ["nominal", "challenge", "overall"],
        "satc_advanced_rule": {
            "single_metric_relative_improvement_min": 0.10,
            "two_metric_relative_improvement_min": 0.05,
            "nominal_not_materially_worse": True,
        },
    })
    write("native_pid_candidate_registry.json", pid_registry())
    write("native_full_lqr_candidate_registry.json", lqi_registry())
    print(json.dumps({"protocol_files": 9, "stage_a_cases": len(case_ids),
                      "pid_candidates": 24, "lqi_candidates": 24}, sort_keys=True))


if __name__ == "__main__":
    main()
