"""Build the auditable V3-R4 unlock decision and exact one-shot manifest."""

from __future__ import annotations

import hashlib
import json
import subprocess
from copy import deepcopy
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v3/r0"
R1 = ROOT / "reproducibility/v3/r1"
R1R1 = ROOT / "reproducibility/v3/r1r1"
R2 = ROOT / "reproducibility/v3/r2"
R3 = ROOT / "reproducibility/v3/r3"
R4 = ROOT / "reproducibility/v3/r4"
START_HEAD = "a668e1ebdd9de1341e6c1cd278ed0b21f730ed83"
EXPECTED_TREES = {
    "reproducibility/v3/r0": "36c63351b55e827e21423cd792ab5b96b294a479",
    "reproducibility/v3/r1": "39855236f5c2890f184b38dd4d07394921b2cc23",
    "reproducibility/v3/r1r1": "345472cc7ce95bcfe4660defc1c7c2194eb23fc4",
    "reproducibility/v3/r2": "5db2db085ac2c3167eb7ff2d15b49cd5bca37f61",
    "reproducibility/v3/r3": "230fa9292c878154e341e4b70fa27865629d6aae",
}
ADVANCED_CONTRACT_SHA256 = "aa39eb5bb673d06d3a4047423fce5d8504eda60b5f3e563e05dcbabfc8763480"


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def build_samples(source: dict) -> list[dict]:
    targets = source["targets"]
    if len(targets) != 12:
        raise RuntimeError("R0 Holdout direction count drift")
    samples: list[dict] = []

    def add(sample_id: str, scenario: str, target: dict, wind: dict) -> None:
        samples.append({
            "sample_id": sample_id,
            "scenario": scenario,
            "target": deepcopy(target),
            "wind": wind,
            "execution_allowed": True,
        })

    for target in targets:
        add(f"holdout_calm_{target['target_id']}", "CALM_3D_SETPOINT", target, {"kind": "calm", "speed_m_s": 0.0})
    for speed, label in ((2.0, "2p0"), (3.5, "3p5")):
        for target in targets:
            add(
                f"holdout_constant_{label}_{target['target_id']}",
                "WIND_3D_SETPOINT",
                target,
                {"kind": "constant", "speed_m_s": speed},
            )
    for seed in source["holdout_seeds"]:
        target = targets[(int(seed) - 3000) % 12]
        add(
            f"holdout_stochastic_{seed}",
            "WIND_3D_SETPOINT",
            target,
            {"kind": "stochastic", "speed_m_s": 0.0, "seed": int(seed)},
        )
    add(
        "holdout_ramp_wind_equilibrium_hold",
        "RAMP_WIND_EQUILIBRIUM_HOLD",
        {
            "target_id": "equilibrium",
            "direction_unit": [0.0, 0.0, 0.0],
            "radius_m": 0.0,
            "delta_tip_m": [0.0, 0.0, 0.0],
        },
        {"kind": "ramp", "start_speed_m_s": 0.0, "speed_m_s": 3.5},
    )
    if len(samples) != 57 or len({row["sample_id"] for row in samples}) != 57:
        raise RuntimeError("mechanical Holdout expansion did not produce 57 unique samples")
    return samples


def main() -> int:
    if git("rev-parse", "HEAD") != START_HEAD:
        raise RuntimeError("protocol builder must start from the authorized START_HEAD")
    if git("branch", "--show-current") != "research-v3":
        raise RuntimeError("wrong branch")
    if git("status", "--porcelain"):
        allowed = {"M scripts/run_v3_r1_baselines.py", "?? scripts/build_v3_r4_holdout_protocol.py", "?? scripts/run_v3_r4_holdout.py", "?? scripts/finalize_v3_r4_holdout.py", "?? tests/v3/test_r4_protocol.py"}
        actual = set(git("status", "--porcelain").splitlines())
        if not actual <= allowed:
            raise RuntimeError(f"unexpected pre-protocol worktree changes: {sorted(actual - allowed)}")

    current_trees = {path: git("rev-parse", f"HEAD:{path}") for path in EXPECTED_TREES}
    r3_gate = read_json(R3 / "gate.json")
    traditional_finalized = all(read_json(path)["holdout_executed"] is False for path in (
        R1R1 / "pid_freeze.json", R1 / "full_lqr_freeze.json", R1 / "task_lqr_freeze.json"
    ))
    self_finalized = read_json(R2 / "self_freeze.json")["candidate_id"] == "self_a_034"
    paper_finalized = r3_gate["paper_route_closed"] and r3_gate["result"] == "CLOSED_WITH_NO_V3_PAPER_ADVANCED_WIN"
    protected = current_trees == EXPECTED_TREES
    contract_sha = sha256(R1R1 / "advanced_numeric_win_contract.json")
    all_final = traditional_finalized and self_finalized and paper_finalized and protected and contract_sha == ADVANCED_CONTRACT_SHA256
    if not all_final:
        raise RuntimeError("BLOCKED_V3_HOLDOUT_UNLOCK_CONTRACT")

    R4.mkdir(parents=True, exist_ok=True)
    write_json(R4 / "holdout_unlock_audit.json", {
        "task": "V3-R4-ONE-SHOT-HOLDOUT-VALIDATION-AND-FINAL-EVIDENCE-FREEZE-R1",
        "audit_performed_before_holdout_execution": True,
        "traditional_finalized": True,
        "self_finalized": True,
        "paper_selection_process_finalized": True,
        "paper_qualified_candidate_exists": False,
        "paper_holdout_status": "NOT_RUN_DEVELOPMENT_INELIGIBLE",
        "all_method_selection_outcomes_final": True,
        "original_r0_contract_modified": False,
        "interpretation": "A permanently closed Paper route with no qualified candidate is a final frozen method-selection outcome; it creates no Paper Holdout participant and does not alter the R0 contract.",
        "protected_trees": {path: {"expected": EXPECTED_TREES[path], "actual": current_trees[path], "unchanged": True} for path in EXPECTED_TREES},
        "advanced_numeric_win_contract_sha256": contract_sha,
        "no_further_method_search_authorized": True,
        "holdout_executed": False,
        "result": "V3_HOLDOUT_UNLOCKED_WITHOUT_R0_CONTRACT_MODIFICATION",
    })

    source = read_json(R0 / "holdout_manifest.json")
    if source["execution_allowed"] is not False or source["direction_sha256"] != "7be79fa030f19159bf66bf447a3ec10c23c0a2231a0f3377cf6cf3fe899abddd":
        raise RuntimeError("R0 Holdout source manifest drift")
    samples = build_samples(source)
    execution = {
        "split": "holdout",
        "one_shot": True,
        "source": "reproducibility/v3/r0/holdout_manifest.json",
        "source_sha256": sha256(R0 / "holdout_manifest.json"),
        "source_direction_sha256": source["direction_sha256"],
        "sample_count": 57,
        "duration_s": 12.0,
        "physics_dt_s": 0.001,
        "log_period_s": 0.005,
        "outer_period_s": 0.05,
        "stochastic_process": "Frozen V3 runner: PCG64(seed), first-order alpha=exp(-0.005/1.0), sigma=0.8 m/s, clip [-3,3] m/s",
        "mapping": {
            "calm": "12 frozen spherical targets",
            "constant_2p0_m_s": "12 frozen spherical targets",
            "constant_3p5_m_s": "12 frozen spherical targets",
            "stochastic": "seeds 3000..3019; target index=(seed-3000) mod 12",
            "ramp": "one equilibrium hold; 0 to 3.5 m/s from t=2 to t=8 s",
        },
        "controllers": [
            {"kind": "corrected_pid", "candidate_id": "hybrid_x007_y041_z041", "freeze": "reproducibility/v3/r1r1/pid_freeze.json"},
            {"kind": "full_lqr", "candidate_id": "full_lqr_048", "freeze": "reproducibility/v3/r1/full_lqr_freeze.json", "primary_traditional": True},
            {"kind": "task_lqr", "candidate_id": "task_lqr_009", "freeze": "reproducibility/v3/r1/task_lqr_freeze.json"},
            {"kind": "self_dr_tsrmpc", "candidate_id": "self_a_034", "method": "3D-DR-TSRMPC", "freeze": "reproducibility/v3/r2/self_freeze.json"},
        ],
        "paper": {"method": "LV2026-SPACC-ADAPTED-3D", "status": "NOT_RUN_DEVELOPMENT_INELIGIBLE"},
        "authoritative_run_count": 228,
        "samples": samples,
    }
    write_json(R4 / "holdout_execution_manifest.json", execution)

    hashed_files = [
        "scripts/run_v3_r1_baselines.py",
        "scripts/run_v3_r4_holdout.py",
        "scripts/finalize_v3_r4_holdout.py",
        "src/uav_sway/v3/controllers.py",
        "src/uav_sway/v3/dr_tsrmpc.py",
        "src/uav_sway/v3/observation.py",
        "src/uav_sway/v3/metrics.py",
        "reproducibility/frozen/model/model_5link_controlled.xml",
        "configs/model_5link.yaml",
        "configs/aerodynamics.yaml",
        "configs/s3_pid.yaml",
        "reproducibility/v3/r1r1/pid_freeze.json",
        "reproducibility/v3/r1/full_lqr_freeze.json",
        "reproducibility/v3/r1/task_lqr_freeze.json",
        "reproducibility/v3/r2/self_freeze.json",
        "reproducibility/v3/r0/win_contract.json",
        "reproducibility/v3/r4/holdout_execution_manifest.json",
    ]
    write_json(R4 / "holdout_protocol_freeze.json", {
        "task": "V3-R4-ONE-SHOT-HOLDOUT-VALIDATION-AND-FINAL-EVIDENCE-FREEZE-R1",
        "start_head": START_HEAD,
        "freeze_commit_definition": "the Git commit containing this file, the exact execution manifest, and the exact runner hashes below",
        "freeze_commit_message": "research(v3): freeze one-shot Holdout protocol",
        "protocol_frozen_before_first_holdout_trajectory": True,
        "original_r0_contract_modified": False,
        "primary_traditional": "full_lqr_048",
        "bootstrap": {"pairing": "exact sample_id", "resamples": 10000, "seed": 20260809, "confidence_interval": 0.95, "required_lower_bound": "> 0"},
        "file_sha256": {path: sha256(ROOT / path) for path in hashed_files},
        "protected_trees": EXPECTED_TREES,
        "paper_holdout_status": "NOT_RUN_DEVELOPMENT_INELIGIBLE",
        "second_holdout_authorized": False,
    })
    print(json.dumps({"result": "V3_HOLDOUT_PROTOCOL_READY_TO_FREEZE", "sample_count": 57, "authoritative_runs": 228}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
