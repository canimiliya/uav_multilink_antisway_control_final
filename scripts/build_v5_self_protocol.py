"""Freeze the exact SATC-OFMPC implementation/search protocol."""

from __future__ import annotations

import hashlib
import json
import subprocess
from itertools import product
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v5/r0"
OUT = ROOT / "reproducibility/v5/self"
START_HEAD = "31c28358371769f3f3510078d3c818afafd2902a"


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(name: str, payload: object) -> None:
    path = OUT / name; path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


CART_B001 = read(ROOT / "reproducibility/v4/r1/near_miss.json")["parameters"]
CART_B014 = next(row["parameters"] for row in read(ROOT / "reproducibility/v4/r1/stage_b_summary.json")["summaries"] if row["candidate_id"] == "cart_b_014")

WRAPPER = {
    "architecture_version": "SATC-OFMPC-A1",
    "shock_reference_gain": 8.0,
    "shock_innovation_scale": 0.10,
    "shock_decay": 0.88,
    "conflict_beta": 0.30,
    "shock_base_weight": 0.60,
    "conflict_gain": 0.80,
    "cancellation_gain": 0.40,
    "cancellation_norm_scale": 0.50,
    "robust_blend_max": 1.0,
    "offset_disengage_rate": 0.25,
    "offset_engage_rate": 0.05,
    "robust_gain_scale": 1.0,
    "amplitude_reserve_fraction": 0.0,
    "slew_reserve_fraction": 0.0,
}


def candidate(candidate_id: str, cart: str = "b001", **overrides: object) -> dict:
    base = CART_B001 if cart == "b001" else CART_B014
    return {**base, **WRAPPER, **overrides, "cart_source": f"cart_{cart}", "candidate_id": candidate_id}


STAGE_A = [
    candidate("satc_a_001"),
    candidate("satc_a_002", robust_blend_max=0.60),
    candidate("satc_a_003", shock_decay=0.80),
    candidate("satc_a_004", shock_decay=0.94),
    candidate("satc_a_005", offset_disengage_rate=0.50),
    candidate("satc_a_006", offset_engage_rate=0.10),
    candidate("satc_a_007", conflict_gain=0.0, cancellation_gain=0.0),
    candidate("satc_a_008", conflict_gain=1.50, cancellation_gain=0.80),
    candidate("satc_a_009", robust_gain_scale=0.80),
    candidate("satc_a_010", slew_reserve_fraction=0.25),
    candidate("satc_a_011", amplitude_reserve_fraction=0.15),
    candidate("satc_a_012", cart="b014"),
]


STAGE_B: list[dict] = []
index = 1
# Family 1: broad shock/fallback duration and strength around the best CART near-miss.
for base, blend, decay in product((0.40, 0.70, 1.00), (0.65, 1.00), (0.82, 0.93)):
    STAGE_B.append(candidate(f"satc_b_{index:03d}", shock_base_weight=base, robust_blend_max=blend, shock_decay=decay, conflict_gain=0.60, cancellation_gain=0.30)); index += 1
# Family 2: geometry/cancellation-driven coordination, including rapid disengagement.
for conflict, cancellation, engage in product((0.50, 1.00, 1.50), (0.0, 0.70), (0.03, 0.08)):
    STAGE_B.append(candidate(f"satc_b_{index:03d}", shock_base_weight=0.35, conflict_gain=conflict, cancellation_gain=cancellation, offset_engage_rate=engage, offset_disengage_rate=0.50)); index += 1
# Family 3: high-response CART source crossed with causal authority reserves.
for blend, slew_reserve, gain_scale in product((0.75, 1.00), (0.0, 0.20, 0.35), (0.80, 1.00)):
    STAGE_B.append(candidate(f"satc_b_{index:03d}", cart="b014", shock_base_weight=0.70, robust_blend_max=blend, slew_reserve_fraction=slew_reserve, robust_gain_scale=gain_scale, offset_disengage_rate=0.50)); index += 1

STAGE_A_IDS = [
    *[f"v5d_strong_simultaneous_{stratum}_{i:02d}" for stratum in ("aligned", "opposed", "cross") for i in range(4)],
    "v5d_calm_00", "v5d_calm_06", "v5d_calm_12", "v5d_moderate_00", "v5d_moderate_06", "v5d_moderate_12",
    "v5d_strong_preexisting_aligned_00", "v5d_strong_preexisting_aligned_01",
    "v5d_strong_preexisting_opposed_00", "v5d_strong_preexisting_opposed_01",
    "v5d_strong_preexisting_cross_00", "v5d_strong_preexisting_cross_01",
]


def main() -> int:
    if git("branch", "--show-current") != "research-v5": raise RuntimeError("wrong branch")
    if git("rev-parse", "HEAD") != START_HEAD: raise RuntimeError("Self protocol must freeze directly after V5 contract checkpoint")
    manifest = read(R0 / "development_manifest.json")
    ids = {row["sample_id"] for row in manifest["samples"]}
    if len(manifest["samples"]) != 120 or len(STAGE_A_IDS) != 24 or not set(STAGE_A_IDS) <= ids: raise RuntimeError("Development assumptions drift")
    if len(STAGE_A) != 12 or len(STAGE_B) != 36: raise RuntimeError("search budget construction drift")
    unique = {row["candidate_id"] for row in STAGE_A + STAGE_B}
    if len(unique) != 48: raise RuntimeError("candidate IDs are not unique")
    write("development_protocol.json", {
        "task": "V5-SATC-OFMPC-DEVELOPMENT-SEARCH-AND-FREEZE-R1", "start_head": START_HEAD,
        "method": "SATC-OFMPC", "architecture_version": "SATC-OFMPC-A1",
        "written_before_first_satc_performance": True,
        "architecture": {
            "shock_detection": "hysteretic max of causal task-reference jump and one-step innovation-rate score",
            "bumpless_offset_engagement": "rate-limited CART-to-robust engagement state; no abrupt offset switch",
            "final_input_coordination": "single physical command blends CART's constrained command and immutable Full-LQR stabilizer before a common limiter",
            "slew_headroom": "optional shock-dependent command amplitude and per-update reserve within, never beyond, frozen authority",
            "conflict_index": "coordinate-free opposition cosine between controllable innovation-equivalent acceleration and measured desired tip displacement",
            "cancellation_index": "opposition and co-activation of task-LQR backbone and CART QP correction",
            "causal_inputs_only": True, "forbidden": ["wind truth", "future wind", "future state", "target-x sign branch", "V5 Holdout"],
        },
        "search": {
            "maximum_unique_configurations": 64,
            "stage_a": {"purpose": "structural smoke only", "selection_authority": False, "candidate_count": 12, "sample_ids": STAGE_A_IDS, "candidates": STAGE_A},
            "stage_b": {"purpose": "full 120-sample frozen Development qualification", "candidate_count": 36, "candidates": STAGE_B, "selection_rule": "all frozen hard gates, then worst directional P90, strong mean, overall mean, normal mean, acquisition, effort, runtime, candidate id"},
            "stage_c": {"purpose": "exact independent confirmation", "candidate_count_max": 8, "candidate_source": "top eight Stage-B parameter sets; no new configuration", "confirmation_rule": "all frozen gates must pass again"},
            "unique_configuration_count": 48,
        },
        "rationale": "Broadly vary causal shock duration/blend, geometric conflict/cancellation coordination, and conservative authority reserve; include both V4 CART Pareto anchors without retuning either as V4.",
        "failure_rule": "If no Stage-C confirmed pass, close V5_SELF_NO_DEVELOPMENT_QUALIFICATION; do not run Paper or Holdout.",
        "trace_rule": {"stage_c_representatives": ["v5d_calm_00", "v5d_strong_simultaneous_aligned_00", "v5d_strong_simultaneous_opposed_00", "v5d_strong_simultaneous_cross_00"], "fields": ["shock", "innovation", "residual", "trust", "conflict", "steady target", "backbone", "MPC", "final command", "slew headroom", "constraints", "position", "orientation", "joint motion"]},
        "frozen_inputs": {name: sha(R0 / name) for name in ("development_manifest.json", "holdout_manifest.json", "win_contract.json", "self_search_budget.json", "self_method_contract.json")},
        "satc_performance_executed_at_freeze": False, "holdout_executed": False,
    })
    print(json.dumps({"stage_a": len(STAGE_A), "stage_b": len(STAGE_B), "unique": len(unique), "performance_executed": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
