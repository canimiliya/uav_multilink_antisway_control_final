"""Freeze the P2-R1R2 recovery protocol and initial 48/config family registries."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/native_stack/r1r2/protocol"
R1R1 = ROOT / "reproducibility/native_stack/r1r1/protocol"
SOURCE_HEAD = "046c64eb8d364a58e5af5425df2f88b95c73b261"
DEV_ID = "0f03df8fe11310f6357197a9d03b605476831c76db58f64d04e92535e6df9473"
DEV_RESOLVED = "0034d481a1abe04a46505410a557dc24815932c8a07cf96b08156c374d1aefe9"
HOLD_ID = "63e6192faf992494f5a78f4c008d844564b0015fc26b43a94a0c98da659b2538"
HOLD_RESOLVED = "4a4b5d92027760e0d37176b7f768746c690a4ca53236ec60224cd423d6582df0"
RATES = [20, 50, 100, 200, 500, 1000]


def dump(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def file_hash(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def scale(u: float, low: float, high: float, log: bool = False) -> float:
    if log:
        return float(np.exp(np.log(low) + u * (np.log(high) - np.log(low))))
    return float(low + u * (high - low))


def lhs(count: int, dimensions: int, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    values = np.empty((count, dimensions))
    for dim in range(dimensions):
        values[:, dim] = (rng.permutation(count) + 0.5) / count
    return values


def common_rate(index: int) -> dict[str, int]:
    outer = RATES[index % len(RATES)]
    return {"outer_rate_hz": outer, "inner_rate_hz": max(outer, 500)}


def pid_registry() -> list[dict[str, Any]]:
    matrix = lhs(48, 18, 202608101)
    rows = []
    for i, u in enumerate(matrix):
        rows.append({
            "family": "native_pid", "method_id": f"r1r2_pid_{i:03d}", **common_rate(i),
            "parameters": {
                "kp": [scale(u[0], .20, 1.80), scale(u[1], .20, 1.80), scale(u[2], .15, 1.20)],
                "kd": [scale(u[3], .60, 2.40), scale(u[4], .60, 2.40), scale(u[5], .40, 1.80)],
                "ki": [scale(u[6], 0.00, .30), scale(u[7], 0.00, .30), scale(u[8], 0.00, .22)],
                "integral_limit": scale(u[9], .50, 3.00),
                "acceleration_limit": scale(u[10], 3.0, 8.0),
                "acceleration_slew_per_s": scale(u[11], 8.0, 60.0),
                "tip_correction_kp": scale(u[12], .05, .80),
                "tip_correction_kd": scale(u[13], .02, .40),
                "tip_correction_limit_m": scale(u[14], .15, .80),
                "swing_angle_gain": scale(u[15], 0.0, .30),
                "swing_rate_gain": scale(u[16], .02, .30),
                "reference_acceleration_gain": scale(u[17], .65, 1.25),
                "reference_velocity_gain": 1.0,
                "attitude_wn": 5.0,
                "attitude_zeta": .95,
            },
        })
    return rows


def lqr_registry(family: str, seed: int) -> list[dict[str, Any]]:
    matrix = lhs(48, 12, seed)
    prefix = "full" if family == "native_full_lqr" else "task"
    rows = []
    for i, u in enumerate(matrix):
        rows.append({
            "family": family, "method_id": f"r1r2_{prefix}_lqr_{i:03d}", **common_rate(i),
            "parameters": {
                "q_position": scale(u[0], 1.0, 120.0, True),
                "q_velocity": scale(u[1], .5, 50.0, True),
                "q_attitude": scale(u[2], 10.0, 250.0, True),
                "q_rate": scale(u[3], .5, 30.0, True),
                "q_integral": scale(u[4], .5, 150.0, True),
                "r_thrust": scale(u[5], .05, 8.0, True),
                "r_torque": scale(u[6], .05, 8.0, True),
                "integral_limit": scale(u[7], .3, 3.0),
                "reference_acceleration_gain": scale(u[8], .6, 1.3),
                "swing_angle_gain": scale(u[9], 0.0, .30),
                "swing_rate_gain": scale(u[10], .01, .30),
                "constraint_margin": scale(u[11], .70, .95),
                "servo": "15_state_LQI" if family == "native_full_lqr" else "15_state_output_weighted_LQT_LQI",
            },
        })
    return rows


def main() -> None:
    dev = json.loads((ROOT / "reproducibility/native_stack/r0s/resolved_development_manifest.json").read_text(encoding="utf-8"))
    hold = json.loads((ROOT / "reproducibility/native_stack/r0s/resolved_holdout_manifest.json").read_text(encoding="utf-8"))
    assert dev["identity_manifest_hash"] == DEV_ID and dev["resolved_manifest_hash"] == DEV_RESOLVED
    assert hold["identity_manifest_hash"] == HOLD_ID and hold["resolved_manifest_hash"] == HOLD_RESOLVED
    assert not hold["execution_allowed"] and not hold["executed"] and hold["authoritative_runs"] == 0
    competence_source = R1R1 / "competence_contract.json"
    competence = json.loads(competence_source.read_text(encoding="utf-8"))
    stage_a = dev["cases"][:48]
    coverage = {
        key: sorted({case["identity"][key] for case in stage_a})
        for key in ("task_family", "trajectory_type", "wind_kind", "wind_direction")
    }
    dump(OUT / "research_contract.json", {
        "task": "P2-R1R2-STRONG-TRADITIONAL-COMPETENCE-RECOVERY-AND-SATC-FINAL-FREEZE-R1",
        "source_head": SOURCE_HEAD, "benchmark_tag": "native-stack-benchmark-v1.1",
        "development_iteration": "P2-R1R2", "r1r1_result_immutable": "BLOCKED_P2_TRADITIONAL_COMPETENCE",
        "failure_audit_required_before_protocol": True,
        "frozen_assets": {"development_identity_hash": DEV_ID, "development_resolved_hash": DEV_RESOLVED,
                          "holdout_identity_hash": HOLD_ID, "holdout_resolved_hash": HOLD_RESOLVED,
                          "case_semantics": "native-case-semantics-v1", "benchmark_changed": False,
                          "plant_changed": False, "metrics_changed": False, "competence_gate_changed": False},
        "holdout": {"execution_allowed": False, "executed": False, "authoritative_runs": 0,
                    "permitted_operations": ["hash verification", "semantic fingerprint verification"]},
        "paper": {"new_paper_selected": False, "search": False, "implementation": False, "performance": False},
    })
    dump(OUT / "competence_reference.json", {
        "inheritance": "byte-exact semantic inheritance from P2-R1R1",
        "source_path": "reproducibility/native_stack/r1r1/protocol/competence_contract.json",
        "source_sha256": file_hash(competence_source), "contract": competence,
    })
    dump(OUT / "rate_contract.json", {
        "physics_hz": 1000, "supported_outer_hz": RATES, "supported_inner_hz": RATES,
        "candidate_rate_is_frozen_parameter": True, "post_result_rate_addition_forbidden": True,
        "equal_rate_diagnostic": {"allowed_only_after_all_final_freezes": True, "rate_hz": 100,
                                  "selection_authority": False, "retuning_allowed": False},
    })
    dump(OUT / "search_contract.json", {
        "initial_unique_configs_per_family": 48, "maximum_unique_configs_per_family": 96,
        "satc_maximum_unique_configs": 128, "initial_design": "deterministic centered Latin hypercube",
        "initial_design_seeds": {"native_pid": 202608101, "native_full_lqr": 202608102, "native_task_lqr": 202608103},
        "stage_a": {"case_count": 48, "all_initial_candidates": True,
                    "rank": ["catastrophic_count_asc", "success_rate_desc", "large_endpoint_failure_fraction_asc",
                             "safety_rate_desc", "setpoint_rmse_asc", "trajectory_rmse_asc", "method_id_asc"]},
        "stage_b": {"full_development_cases": 200, "finalists_per_family": 6,
                    "competent_claim_requires_complete_200": True},
        "stage_c": {"allowed_if_no_confirmed_competence": True, "method": "deterministic coordinate refinement around top two Stage-B candidates",
                    "additional_unique_configs": 48, "within_frozen_ranges": True},
        "early_stop": {"allowed_after_confirmed_complete_200_competence": True,
                       "minimum_unique_configs_already_evaluated": 48},
        "benchmark_specific_hard_coding_forbidden": True,
    })
    dump(OUT / "selection_contract.json", {
        "traditional_minimum_competent_families": 2,
        "family_selection": ["competence_pass_desc", "safety_rate_desc", "success_rate_desc",
                             "setpoint_rmse_asc", "trajectory_rmse_asc", "strong_p90_asc",
                             "acquisition_asc", "effort_asc", "runtime_p95_asc", "method_id_asc"],
        "primary_eligibility": "competent families only",
        "primary_selection": ["safety_rate_desc", "success_rate_desc", "setpoint_rmse_asc",
                              "trajectory_rmse_asc", "strong_p90_asc", "acquisition_asc",
                              "effort_asc", "runtime_p95_asc", "method_id_asc"],
        "traditional_envelope_metrics": ["safety", "success", "setpoint_rmse", "trajectory_rmse", "acquisition",
                                         "strong_mean", "strong_p90", "orientation", "effort", "runtime"],
        "satc_start_gate": "competent_traditional_count >= 2",
        "satc_qualification": competence["satc_advanced"],
    })
    dump(OUT / "stage_a_manifest.json", {
        "case_count": 48, "case_ids": [case["sample_id"] for case in stage_a], "coverage": coverage,
        "target_coverage": {"horizontal": ["near", "mid", "far"], "vertical": ["low", "mid", "high"]},
        "selection_authority": True,
    })
    registries = {
        "native_pid": pid_registry(),
        "native_full_lqr": lqr_registry("native_full_lqr", 202608102),
        "native_task_lqr": lqr_registry("native_task_lqr", 202608103),
    }
    for family, registry in registries.items():
        assert len(registry) == 48 and len({row["method_id"] for row in registry}) == 48
        dump(OUT / f"{family}_candidate_registry.json", registry)
    files = sorted(path for path in OUT.glob("*.json") if path.name != "protocol_manifest.json")
    dump(OUT / "protocol_manifest.json", {
        "frozen_before_new_candidate_performance": True,
        "files": [{"path": path.relative_to(ROOT).as_posix(), "sha256": file_hash(path)} for path in files],
    })


if __name__ == "__main__":
    main()
