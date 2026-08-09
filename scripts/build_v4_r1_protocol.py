"""Build the performance-before-data V4-R1 CART-OFMPC protocol."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v4/r0"
R1 = ROOT / "reproducibility/v4/r1"
START_HEAD = "b0dd45013e496bfd0f0371ef7c4f7e2333f8ab80"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


BASE = {
    "architecture_version": "CART-OFMPC-A1",
    "backbone": "task_lqr_009",
    "horizon_updates": 8,
    "residual_beta": 0.30,
    "residual_component_limit": 0.20,
    "steady_equality_weight": 600.0,
    "steady_input_weight": 0.05,
    "steady_state_regularization": 1.0e-5,
    "steady_task_weights": [80.0, 80.0, 80.0, 4.0, 4.0, 4.0, 1.0, 1.0, 1.0, 0.2, 0.2, 0.2],
    "steady_reach_updates": 8,
    "steady_feasibility_tolerance": 0.20,
    "trust_floor": 0.30,
    "trust_beta": 0.30,
    "trust_clip_scale": 2.0,
    "trust_feasibility_scale": 2.0,
    "trust_debt_scale": 0.8,
    "antiwindup_gain": 0.40,
    "debt_decay": 0.10,
    "debt_feedback": 0.10,
    "debt_limit": 0.50,
    "task_position_weight": 40.0,
    "task_velocity_weight": 2.0,
    "orientation_weight": 1.0,
    "angular_velocity_weight": 0.2,
    "correction_weight": 0.20,
    "rate_weight": 1.0,
}


def candidate(candidate_id: str, **overrides: object) -> dict:
    result = {**BASE, **overrides, "candidate_id": candidate_id}
    return result


STAGE_A = [
    candidate("cart_a_001"),
    candidate("cart_a_002", horizon_updates=6, steady_reach_updates=6),
    candidate("cart_a_003", horizon_updates=10, steady_reach_updates=10),
    candidate("cart_a_004", residual_beta=0.15, trust_beta=0.15),
    candidate("cart_a_005", residual_beta=0.50, trust_beta=0.50),
    candidate("cart_a_006", steady_equality_weight=120.0, trust_floor=0.15),
    candidate("cart_a_007", steady_equality_weight=3000.0, trust_floor=0.55),
    candidate("cart_a_008", debt_feedback=0.25, antiwindup_gain=0.70, trust_debt_scale=1.5),
]


STAGE_B = [
    candidate("cart_b_001", steady_equality_weight=120.0, trust_floor=0.10, trust_clip_scale=4.0, trust_feasibility_scale=4.0),
    candidate("cart_b_002", steady_equality_weight=120.0, trust_floor=0.30, trust_clip_scale=3.0, trust_feasibility_scale=3.0),
    candidate("cart_b_003", steady_equality_weight=120.0, trust_floor=0.55, trust_clip_scale=2.0, trust_feasibility_scale=2.0),
    candidate("cart_b_004", steady_equality_weight=300.0, trust_floor=0.10, trust_clip_scale=4.0, trust_feasibility_scale=3.0),
    candidate("cart_b_005", steady_equality_weight=300.0, trust_floor=0.30, trust_clip_scale=3.0, trust_feasibility_scale=2.0),
    candidate("cart_b_006", steady_equality_weight=300.0, trust_floor=0.55, trust_clip_scale=2.0, trust_feasibility_scale=1.5),
    candidate("cart_b_007", steady_equality_weight=600.0, trust_floor=0.10, trust_clip_scale=4.0, trust_feasibility_scale=3.0),
    candidate("cart_b_008", steady_equality_weight=600.0, trust_floor=0.30, trust_clip_scale=2.0, trust_feasibility_scale=2.0),
    candidate("cart_b_009", steady_equality_weight=600.0, trust_floor=0.55, trust_clip_scale=1.5, trust_feasibility_scale=1.5),
    candidate("cart_b_010", steady_equality_weight=1500.0, trust_floor=0.10, trust_clip_scale=4.0, trust_feasibility_scale=4.0),
    candidate("cart_b_011", steady_equality_weight=1500.0, trust_floor=0.30, trust_clip_scale=2.5, trust_feasibility_scale=2.5),
    candidate("cart_b_012", steady_equality_weight=1500.0, trust_floor=0.55, trust_clip_scale=1.5, trust_feasibility_scale=1.5),
    candidate("cart_b_013", residual_beta=0.15, residual_component_limit=0.12, trust_beta=0.15, steady_equality_weight=300.0, task_position_weight=80.0, correction_weight=0.10, rate_weight=0.5),
    candidate("cart_b_014", residual_beta=0.20, residual_component_limit=0.20, steady_equality_weight=600.0, horizon_updates=10, steady_reach_updates=10, task_position_weight=80.0, task_velocity_weight=4.0, correction_weight=0.10, rate_weight=0.5),
    candidate("cart_b_015", residual_beta=0.40, residual_component_limit=0.12, steady_equality_weight=300.0, task_position_weight=80.0, task_velocity_weight=4.0, correction_weight=0.40, rate_weight=2.0),
    candidate("cart_b_016", debt_feedback=0.05, antiwindup_gain=0.25, debt_decay=0.15, trust_debt_scale=0.4, steady_equality_weight=600.0, trust_floor=0.40),
    candidate("cart_b_017", debt_feedback=0.20, antiwindup_gain=0.70, debt_decay=0.08, trust_debt_scale=1.5, steady_equality_weight=300.0, trust_floor=0.20),
    candidate("cart_b_018", steady_input_weight=0.50, steady_equality_weight=300.0, trust_floor=0.30, horizon_updates=6, steady_reach_updates=6, task_position_weight=120.0, correction_weight=0.10, rate_weight=0.5),
]


STAGE_A_IDS = [
    "dev_calm_00", "dev_calm_06", "dev_constant_2p0_00", "dev_constant_2p0_06",
    "dev_stochastic_3000", "dev_stochastic_3010", "dev_strong_preexisting_00", "dev_strong_preexisting_06",
    "dev_strong_simultaneous_00", "dev_strong_simultaneous_06", "dev_strong_post_target_00", "dev_strong_post_target_06",
    "dev_ramp_target_00", "dev_ramp_target_06", "dev_strong_equilibrium", "dev_ramp_equilibrium",
]


def main() -> int:
    manifest = json.loads((R0 / "development_manifest.json").read_text(encoding="utf-8"))
    ids = {row["sample_id"] for row in manifest["samples"]}
    if len(manifest["samples"]) != 94 or not set(STAGE_A_IDS) <= ids:
        raise RuntimeError("frozen Development bank does not match protocol assumptions")
    protocol = {
        "task": "V4-R1-CART-OFMPC-DEVELOPMENT-AND-SELF-FREEZE-R1",
        "start_head": START_HEAD,
        "branch": "research-v4",
        "method": "CART-OFMPC",
        "architecture_version": "CART-OFMPC-A1",
        "written_before_first_new_self_development_performance": True,
        "architecture": {
            "causal_residual_estimator": "one-step A/B innovation using previous actual limited acceleration; component-bounded low-pass estimate projected into the frozen controllability basis",
            "constraint_feasible_steady_target": "three-input active-set box QP with physical amplitude and finite-slew reachable bounds; soft residual balance exposes rather than silently scales unrepresented disturbance",
            "trust_scheduler": "bounded causal score from innovation clipping, steady model residual, and anti-windup debt",
            "residual_antiwindup": "bounded back-calculation debt from the unrepresented steady residual with decay and feedback",
            "predictive_layer": "finite-horizon task-space residual QP constraining every predicted physical acceleration and slew move",
            "nominal_backbone": "immutable task_lqr_009 gain",
            "forbidden_information": ["true wind", "future wind", "future state", "V4 Holdout"],
        },
        "parameter_semantics": {
            "residual_beta": "causal innovation low-pass update fraction",
            "residual_component_limit": "per-state innovation component bound before filtering",
            "steady_equality_weight": "soft model-balance penalty inside the bounded steady target",
            "steady_reach_updates": "number of frozen 0.25 m/s2 slew steps defining steady-input reachability",
            "trust_floor": "minimum residual-model trust; not a wind threshold",
            "debt_feedback": "fraction of bounded unrepresented-residual debt removed from the next steady request",
            "horizon_updates": "20 Hz predictive horizon length",
        },
        "search": {
            "maximum_unique_configurations": 32,
            "stage_a": {"purpose": "structural smoke only", "selection_authority": False, "candidate_max": 8, "sample_count_each": 16, "sample_ids": STAGE_A_IDS, "candidates": STAGE_A, "failure_handling": "record numerical or safety failure; do not promote or use Stage-A metrics for selection"},
            "stage_b": {"purpose": "full frozen Development search", "candidate_max": 18, "sample_count_each": 94, "candidates": STAGE_B, "selection_rule": "evaluate immutable hard gates; rank gate-pass candidates by strong position margin, overall position, normal position, acquisition, effort, runtime; rank near-misses first by hard-gate count then the same tuple"},
            "stage_c": {"purpose": "independent full-bank confirmation", "candidate_max": 6, "sample_count_each": 94, "candidate_source": "top six Stage-B configurations under the frozen rule; no new parameters", "confirmation_rule": "a candidate must independently pass every hard gate on the Stage-C rerun; select the highest Stage-B-ranked confirmed pass"},
            "maximum_unique_if_stage_c_reuses_stage_b": 26,
        },
        "hard_gates_source": "reproducibility/v4/r0/v4_win_contract.json",
        "failure_handling": {"simulation_or_solver_failure": "record failed sample and candidate; no hidden retry except identical infrastructure retry", "no_candidate_pass": "close with best preregistered near-miss after Stage C; no 33rd configuration", "code_bug": "fix transparently without changing architecture or search space; invalidate and exactly rerun affected Development rows"},
        "logging_fields": ["raw/clipped/represented/unrepresented residual", "residual debt", "trust", "requested and feasible steady input", "steady feasibility and constraint activity", "backbone/QP cancellation", "amplitude/slew activity", "limiter mismatch", "solver status/runtime"],
        "frozen_inputs": {
            "development_manifest_sha256": sha256(R0 / "development_manifest.json"),
            "holdout_manifest_sha256": sha256(R0 / "holdout_manifest.json"),
            "win_contract_sha256": sha256(R0 / "v4_win_contract.json"),
            "search_budget_sha256": sha256(R0 / "v4_search_budget.json"),
            "baseline_results_sha256": sha256(R1 / "baseline_development_results.csv"),
            "baseline_summary_sha256": sha256(R1 / "baseline_summary.json"),
        },
        "new_self_performance_executed_at_freeze": False,
        "holdout_executed": False,
        "paper_search": False,
    }
    write_json(R1 / "development_protocol.json", protocol)
    print(json.dumps({"stage_a": len(STAGE_A), "stage_b": len(STAGE_B), "unique": len({row['candidate_id'] for row in STAGE_A + STAGE_B}), "holdout_executed": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
