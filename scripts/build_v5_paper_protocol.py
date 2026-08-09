"""Freeze the unique V5 Paper selection, adaptation and search protocol."""

from __future__ import annotations

import json
import subprocess
from itertools import product
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "reproducibility/v5/paper"
START_HEAD = "c1b63ab318b7d5db9f60b191614e39884cba564b"


def git(*args: str) -> str: return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()
def write(name: str, value: object) -> None:
    path = OUT / name; path.parent.mkdir(parents=True, exist_ok=True); path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def candidate(candidate_id: str, **values: object) -> dict:
    return {"method": "JIROUSEK2025-INCREMENTAL-MPC-ADAPTED-5LINK", "candidate_id": candidate_id, "horizon_updates": 10, "position_weight": 60.0, "velocity_weight": 8.0, "orientation_weight": 2.0, "angular_weight": 0.5, "input_weight": 0.10, "delta_weight": 2.0, **values}


STAGE_A = [candidate("paper_a_001"), candidate("paper_a_002", horizon_updates=6), candidate("paper_a_003", horizon_updates=14), candidate("paper_a_004", position_weight=20.0), candidate("paper_a_005", position_weight=120.0), candidate("paper_a_006", delta_weight=0.5), candidate("paper_a_007", delta_weight=8.0), candidate("paper_a_008", orientation_weight=8.0, angular_weight=2.0)]
STAGE_B = []
index = 1
for horizon, position, delta in product((6, 10, 14), (30.0, 70.0), (0.5, 3.0)):
    STAGE_B.append(candidate(f"paper_b_{index:03d}", horizon_updates=horizon, position_weight=position, velocity_weight=0.15 * position, delta_weight=delta)); index += 1
for orientation, input_weight, delta in product((0.5, 3.0), (0.03, 0.30), (1.0, 6.0)):
    STAGE_B.append(candidate(f"paper_b_{index:03d}", position_weight=90.0, velocity_weight=12.0, orientation_weight=orientation, angular_weight=0.25 * orientation, input_weight=input_weight, delta_weight=delta)); index += 1
for horizon, position in product((8, 12), (45.0, 90.0)):
    STAGE_B.append(candidate(f"paper_b_{index:03d}", horizon_updates=horizon, position_weight=position, velocity_weight=0.20 * position, orientation_weight=5.0, angular_weight=1.0, input_weight=0.08, delta_weight=1.5)); index += 1


def main() -> int:
    if git("rev-parse", "HEAD") != START_HEAD: raise RuntimeError("Paper protocol must follow completed Self mechanism evidence")
    if len(STAGE_A) != 8 or len(STAGE_B) != 24: raise RuntimeError("Paper budget drift")
    write("literature_matrix.json", {
        "workflow": "multi-source-search", "search_date": "2026-08-09", "academic_search_mcp_available": False, "fallback": "T1 primary-source web search",
        "candidates": [
            {"title": "Towards Fully Onboard State Estimation and Trajectory Tracking for UAVs with Suspended Payloads", "year": 2025, "venue": "ICINCO 2025", "doi": "10.5220/0013789200003982", "primary_full_source": "https://arxiv.org/html/2508.11547v2", "method": "linear Kalman filter + incremental MPC + MPCC", "five_link_adaptability": "high", "same_authority_adaptability": "high", "selected": True},
            {"title": "ES-HPC-MPC: Exponentially Stable Hybrid Perception Constrained MPC for Quadrotor with Suspended Payloads", "year": 2025, "venue": "IEEE RA-L", "primary_full_source": "https://arxiv.org/html/2504.08841v2", "method": "hybrid MPC with ES-CLF and perception CBF", "five_link_adaptability": "low: benchmark has no slack cable or camera FoV state", "selected": False},
            {"title": "Trajectory tracking and load anti-swing control under time-varying disturbances", "year": 2025, "venue": "Control Engineering Practice", "doi": "10.1016/j.conengprac.2025.106518", "method": "dual TVUDE", "five_link_adaptability": "medium", "selected": False},
            {"title": "Improved extended disturbance observer-based integral backstepping sliding mode control", "year": 2025, "venue": "Aerospace Science and Technology", "doi": "10.1016/j.ast.2025.110042", "method": "IEDO + integral backstepping sliding mode", "five_link_adaptability": "medium-low: single-load angle decoupling", "selected": False},
            {"title": "Modeling and Control for UAV with Off-center Slung Load", "year": 2026, "primary_full_source": "https://arxiv.org/abs/2601.03386", "method": "suspension-point cascade", "selected": False, "reason": "already adapted and closed in V3; not a fresh V5 Paper route"},
        ],
        "deduplication": "normalized DOI, then normalized title + first author", "selection_not_based_on_ease_of_defeat": True,
    })
    write("paper_selection.json", {
        "selected_title": "Towards Fully Onboard State Estimation and Trajectory Tracking for UAVs with Suspended Payloads", "authors": ["Martin Jirousek", "Tomas Baca", "Martin Saska"], "year": 2025, "venue": "ICINCO 2025", "doi": "10.5220/0013789200003982", "arxiv": "2508.11547v2", "primary_source_complete": True,
        "selection_reason": "Published, openly available, complete incremental-MPC mathematics, payload-position objective, causal onboard sensing, direct compatibility with the frozen acceleration and slew interface.",
        "paper_claim_preserved": "incremental augmented-state MPC optimizes command increments and constrained physical inputs to smooth actuation while tracking payload position", "performance_seen_before_selection": False,
    })
    write("adaptation_protocol.json", {
        "task": "V5-PAPER-ADVANCED-JIROUSEK2025-INCREMENTAL-MPC-R1", "start_head": START_HEAD, "written_before_paper_performance": True,
        "source_equations": {"24": "augmented [x,u] dynamics driven by delta-u", "22_23": "QP with quadratic tracking/input costs and constraints", "25_26": "payload position plus input output penalty and delta-u penalty"},
        "adaptation": {
            "source_single_cable_state": "replaced by frozen identified 20D five-link error model without changing plant", "payload_position_output": "frozen C_pos tip-position map", "source_attitude_command": "frozen world-frame acceleration command",
            "incremental_decision": "delta world acceleration", "input_constraints": "per-axis absolute 2.0 m/s2", "increment_constraints": "per-update 0.25 m/s2", "outer_rate_hz": 20.0,
            "state_estimator": "not duplicated because the benchmark already exposes the same causal measured state contract to every method", "trajectory_planner": "not adapted because the task target and reference generator are frozen",
        },
        "core_mechanism_preserved": True, "noncausal_information": False, "traditional_self_unchanged": True,
        "search": {"maximum_unique_configurations": 32, "stage_a": {"selection_authority": False, "candidate_count": 8, "sample_count_each": 24, "candidates": STAGE_A}, "stage_b": {"candidate_count": 24, "sample_count_each": 120, "candidates": STAGE_B, "selection_rule": "V5 Overall gate first; then position, acquisition, success, effort, runtime, candidate id"}},
        "qualification": "all frozen V5 Overall gates", "no_qualified_candidate": "PAPER_DEVELOPMENT_INELIGIBLE; no replacement paper", "holdout_executed": False,
    })
    print(json.dumps({"selected": "Jirousek2025 incremental MPC", "stage_a": 8, "stage_b": 24, "performance": False}, indent=2))
    return 0


if __name__ == "__main__": raise SystemExit(main())
