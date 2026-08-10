"""Freeze the final V10 FxTDO-MPC protocol before any V10 performance run."""

from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v10/r0"
SOURCE_TAG = "v9-research-final-2026-08-10"
SOURCE_HEAD = "46b4c31d45d427f790ef2978702552b5ee8235ab"
PAPER_PDF_SHA256 = "cf497d3f6bb1a7322119ce42459de41af8d99f45b7c80582bc4ddf2d6864af97"
PAPER_TEX_SHA256 = "93d61edf3e5fa29925bfa0e84c2465b3b025181e1ab5e6cc4127f4defbc0e44a"
SEARCH_SEED = 20260831
BOOTSTRAP_SEED = 20260901
HOLDOUT_BOOTSTRAP_SEED = 20260902


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(name: str, payload: object) -> None:
    path = OUT / name
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
        stream.write("\n")


def candidate(index: int, rng: np.random.Generator) -> dict:
    def pick(values):
        return values[int(rng.integers(0, len(values)))]

    return {
        "candidate_id": f"fxtdo_v10_{index:03d}",
        "method": "XU-FXTDO-MPC-ADAPTED-5LINK",
        "horizon_updates": int(pick([4, 6, 8, 10, 12])),
        "position_weight": float(pick([80.0, 150.0, 300.0, 600.0])),
        "velocity_weight": float(pick([4.0, 12.0, 30.0, 60.0])),
        "orientation_weight": float(pick([0.5, 1.0, 3.0, 8.0])),
        "angular_weight": float(pick([0.1, 0.2, 0.8, 2.0])),
        "input_weight": float(pick([0.02, 0.08, 0.3, 1.0])),
        "rate_weight": float(pick([0.05, 0.2, 0.8, 2.0])),
        "terminal_multiplier": float(pick([1.0, 2.0, 4.0])),
        "disturbance_compensation_scale": float(pick([0.35, 0.6, 0.85, 1.0])),
        "observer_l1": float(pick([0.5, 1.0, 2.0, 4.0])),
        "observer_l2": float(pick([0.5, 1.0, 2.0, 4.0])),
        "observer_k1": float(pick([1.0, 2.0, 3.0])),
        "observer_k1_prime": float(pick([0.3, 0.6, 1.2])),
        "observer_k1_double_prime": float(pick([1.5, 3.0, 6.0])),
        "observer_k2": float(pick([1.0, 2.0, 3.0])),
        "observer_k2_prime": float(pick([0.3, 0.6, 1.2])),
        "observer_k2_double_prime": float(pick([1.5, 3.0, 6.0])),
        "observer_d_infinity": float(pick([1.0 / 3.0, 0.5])),
        "observer_boundary": float(pick([1.0e-4, 1.0e-3, 1.0e-2])),
        "observer_clip_m_s2": float(pick([0.5, 1.0, 2.0, 4.0])),
        "observer_substeps": int(pick([5, 10, 20])),
        "regularization": float(pick([1.0e-8, 1.0e-6, 1.0e-4])),
    }


def main() -> int:
    if git("branch", "--show-current") != "research-v10":
        raise RuntimeError("V10 contract must be frozen on research-v10")
    if git("rev-parse", f"{SOURCE_TAG}^{{}}") != SOURCE_HEAD or git("rev-parse", "HEAD") != SOURCE_HEAD:
        raise RuntimeError("V10 must begin exactly at the frozen V9 source")
    OUT.mkdir(parents=True, exist_ok=True)
    for name in ("development_manifest.json", "holdout_manifest.json"):
        shutil.copyfile(ROOT / "reproducibility/v7/r0" / name, OUT / name)
    dev = OUT / "development_manifest.json"
    holdout = OUT / "holdout_manifest.json"
    if sha256(dev) != "b51d9958b706354867d593d61a93336c702561e1e3b3e9dfcd5488ced0886cc3":
        raise RuntimeError("inherited Development manifest drift")
    if sha256(holdout) != "3c903826ec0d6ba3ea0c941a07f7499f0863c38b8474a40c5acc2a1d14b87d65":
        raise RuntimeError("inherited Holdout manifest drift")
    rng = np.random.Generator(np.random.PCG64(SEARCH_SEED))
    candidates = [candidate(index, rng) for index in range(1, 65)]
    dev_rows = json.loads(dev.read_text(encoding="utf-8"))["samples"]
    core_ids = [
        next(row["sample_id"] for row in dev_rows if row["cohort"] == cohort and row["sample_id"] not in set())
        for cohort in (
            "NORMAL_CALM", "NORMAL_CONSTANT", "NORMAL_STOCHASTIC",
            "RAMP_STRONG", "STRONG_PREEXISTING", "STRONG_NEAR_SIMULTANEOUS",
        )
    ]
    # Include a second deterministic member of every cohort.
    for cohort in (
        "NORMAL_CALM", "NORMAL_CONSTANT", "NORMAL_STOCHASTIC",
        "RAMP_STRONG", "STRONG_PREEXISTING", "STRONG_NEAR_SIMULTANEOUS",
    ):
        rows = [row for row in dev_rows if row["cohort"] == cohort]
        core_ids.append(rows[len(rows) // 2]["sample_id"])
    write("paper_source_audit.json", {
        "title": "Fixed-Time Disturbance Observer-Based MPC Robust Trajectory Tracking Control of Quadrotor",
        "authors": ["Liwen Xu", "Bailing Tian", "Cong Wang", "Junjie Lu", "Dandan Wang", "Zhiyu Li", "Qun Zong"],
        "primary_source": "arXiv:2408.15019v2", "version_date": "2024-08-30",
        "pdf_sha256": PAPER_PDF_SHA256, "tex_bundle_sha256": PAPER_TEX_SHA256,
        "full_text_retrieved": True, "tex_source_retrieved": True, "performance_executed": False,
        "equation_inventory": {
            "quadrotor_dynamics": "Eqs. (1a)-(1d)", "lumped_force": "Eq. (1b)",
            "observer_system": "Eq. (5)", "observer_states": "Eq. (6)",
            "phi_1_phi_2": "Eq. (7)", "observer_error": "Eq. (8)",
            "fixed_time_conditions": "Eqs. (9), (20)-(22)",
            "prediction_model": "Eqs. (23a)-(23c)", "mpc_state_input": "Eqs. (24)-(25)",
            "mpc_cost_constraints": "Eqs. (26)-(27)", "indi": "Eqs. (30)-(33)",
        },
        "paper_parameters": {"N": 10, "dt_s": 0.1, "k_i": 2.0, "k_i_prime": 0.6, "k_i_double_prime": 3.0, "d_infinity": 1.0 / 3.0, "L1": 1.0, "L2": 1.0},
        "paper_claimed_observer_convergence_s": 0.94,
    })
    write("research_contract.json", {
        "task": "V10-FXTDO-MPC-RECENT-PAPER-STRONG-BASELINE-AND-FINAL-PROJECT-CLOSURE-R1",
        "source_tag": SOURCE_TAG, "source_head": SOURCE_HEAD, "branch": "research-v10",
        "paper": "Liwen Xu et al., arXiv:2408.15019", "adaptation": "XU-FXTDO-MPC-ADAPTED-5LINK",
        "fidelity": "ADAPTED_NOT_EXACT_REPRODUCTION",
        "FINAL_PREREGISTERED_EXTERNAL_PAPER_ROUTE": True, "NO_AUTOMATIC_V11": True,
        "selection_reason": "V6-V9 failure-driven: no pendulum reduction, modal payload controller, or offline predictor; paper natively estimates wind/payload/aerodynamic/model effects as a causal lumped disturbance and injects it into MPC.",
        "frozen": {"plant": "MuJoCo five-link", "outer_rate_hz": 20.0, "command": "world [ax,ay,az]", "amplitude_per_axis": 2.0, "slew_per_update": 0.25, "geometric_inner_loop": True, "traditional": ["corrected_pid", "full_lqr_048", "task_lqr_009"], "satc": "satc_b_027"},
        "forbidden": ["V1-V9 mutation", "Traditional retune", "SATC retune", "plant change", "control-authority expansion", "payload reduction", "SATC proprietary mechanisms", "Paper replacement", "Holdout before Paper freeze"],
        "development_failure": {"result": "V10_FXTDO_MPC_NOT_STRONGER_THAN_TRADITIONAL", "holdout_executed": False, "project_complete": False, "FINAL_EXTERNAL_PAPER_SEARCH_CLOSED": True, "NO_V11": True},
    })
    write("adaptation_contract.json", {
        "adaptation_name": "XU-FXTDO-MPC-ADAPTED-5LINK", "fidelity": "ADAPTED_NOT_EXACT_REPRODUCTION",
        "preserved": ["multivariable FxTDO", "fixed-time bi-homogeneous phi structure", "causal lumped-disturbance estimate", "constant disturbance injection over prediction horizon", "receding-horizon constrained MPC"],
        "adapted": ["force observer mass-normalized to world acceleration", "paper 10D p-v-q model replaced by frozen 20D five-link linear prediction model", "trajectory cost replaced by frozen task-output C_task_v3", "100 Hz source controller executed at fair 20 Hz"],
        "omitted": ["torque disturbance observer", "INDI torque command", "paper thrust/body-rate authority", "differential-flatness reference generator"],
        "omission_reason": "common benchmark grants only world acceleration and common geometric inner loop",
        "runtime_inputs": ["current/past UAV position", "current/past UAV velocity", "previous actual limited acceleration", "current/past plant observation", "current reference"],
        "forbidden_runtime_inputs": ["true wind", "MuJoCo hidden forces", "future state", "future wind", "SATC internals"],
    })
    write("search_contract.json", {
        "search_seed": SEARCH_SEED, "maximum_unique_configurations": 128, "planned_unique_configurations": 64,
        "stage_0": "equation/frame/observer unit diagnostics only",
        "stage_a": {"candidates": 64, "samples_each": 12, "sample_ids": core_ids, "selection_rule": "finite and stable, then safety, success, position, strong position, orientation, effort, runtime, id"},
        "stage_b": {"advance_top": 16, "samples_each": 144, "selection_authority": True},
        "confirmation": {"candidate_count": 1, "samples": 144, "performance_retry": False, "uses_cached_exact-pair_results": True},
        "parameter_sources": {"observer": "paper-native values and positive gain/boundary numerical adaptations", "mpc": "paper N/Q/R/P concepts adapted to C_task_v3 and common acceleration/rate constraints"},
        "candidates": candidates,
    })
    write("win_contract.json", {
        "primary_traditional": "full_lqr_048", "pairs": 144, "bootstrap_resamples": 10000, "bootstrap_seed": BOOTSTRAP_SEED,
        "hard_gates": {"safety_at_least_best_traditional": True, "success_at_least_best_traditional": True, "position_improvement_min": 0.05, "acquisition_degradation_max": 0.05, "paired_bootstrap_lower_gt": 0.0, "strong_position_improvement_min": 0.05, "strong_p90_degradation_max": 0.10, "catastrophic_max": 0},
        "advanced_value_any": {"acquisition_improvement_min": 0.05, "orientation_improvement_min": 0.10, "strong_mean_improvement_min": 0.10, "effort_improvement_min": 0.10, "runtime_improvement_vs_satc_min": 0.50},
        "selection_rule": "qualified first; then gate margin, position, strong P90, acquisition, orientation, effort, runtime, candidate_id",
    })
    write("statistical_protocol.json", {"pair_key": "sample_id", "development_seed": BOOTSTRAP_SEED, "holdout_seed": HOLDOUT_BOOTSTRAP_SEED, "resamples": 10000, "ci": [0.025, 0.975], "seed_hunting_forbidden": True})
    write("split_integrity.json", {
        "development_source": "reproducibility/v7/r0/development_manifest.json", "development_sha256": sha256(dev), "development_samples": 144,
        "holdout_source": "reproducibility/v7/r0/holdout_manifest.json", "holdout_sha256": sha256(holdout), "holdout_samples": 112,
        "V7_HOLDOUT_HASH_UNCHANGED": True, "V8_HOLDOUT_EXECUTED": False, "V9_HOLDOUT_EXECUTED": False, "v10_holdout_executed": False,
    })
    hashes = {path.name: sha256(path) for path in sorted(OUT.glob("*.json")) if path.name != "contract_sha256.json"}
    write("contract_sha256.json", {"files": hashes, "performance_executed": False})
    print(json.dumps({"branch": git("branch", "--show-current"), "source": git("rev-parse", "HEAD"), "contracts": len(hashes), "unique_configs": len(candidates)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
