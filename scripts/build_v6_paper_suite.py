"""Freeze the complete V6 Paper suite, adaptations, and search spaces."""

from __future__ import annotations

import json
import subprocess
from itertools import product
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v6/r0"
OUT = ROOT / "reproducibility/v6/papers"
START_HEAD = "8cb763054d292f0bf40e2b9be08c2635b90fdb53"


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(name: str, payload: object) -> None:
    path = OUT / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def yu(candidate_id: str, **updates: object) -> dict:
    value = {
        "method": "YU2026-FT-CFO-ADAPTED-5LINK", "candidate_id": candidate_id,
        "finite_time_power_r": 0.85, "position_gain": 2.0, "velocity_gain": 1.4,
        "observer_eta1": 0.1, "observer_eta2": 2.0, "observer_n1": 2.0, "observer_n2": 20.0,
        "observer_scale": 0.5, "observer_clip_m_s2": 2.0,
        "swing_position_gain": 0.8, "swing_velocity_gain": 0.35,
    }
    value.update(updates)
    return value


def sep(candidate_id: str, **updates: object) -> dict:
    value = {
        "method": "SEP2026-PASSIVITY-NMPC-ADAPTED-5LINK", "candidate_id": candidate_id,
        "horizon_updates": 10, "position_weight": 60.0, "velocity_weight": 8.0,
        "orientation_weight": 2.0, "angular_weight": 0.5, "input_weight": 0.10,
        "rate_weight": 1.0, "storage_position_gain": 1.0,
        "passivity_rho": 0.20, "passivity_epsilon": 0.02,
    }
    value.update(updates)
    return value


def main() -> int:
    if git("rev-parse", "HEAD") != START_HEAD:
        raise RuntimeError("Paper suite must freeze immediately after V6 contract")
    manifest = read(R0 / "development_manifest.json")
    samples = manifest["samples"]
    stage_a_ids = []
    targets = [
        ("NORMAL_CALM", 6), ("NORMAL_CONSTANT", 6),
        ("STRONG_NEAR_SIMULTANEOUS", 6), ("NORMAL_STOCHASTIC", 6),
    ]
    for cohort, count in targets:
        matching = [row["sample_id"] for row in samples if row["cohort"] == cohort]
        stage_a_ids.extend(matching[:count])
    if len(stage_a_ids) != 24 or len(set(stage_a_ids)) != 24:
        raise RuntimeError("Stage-A smoke sample selection drift")

    yu_a = [
        yu("yu_a_001"), yu("yu_a_002", finite_time_power_r=0.70),
        yu("yu_a_003", finite_time_power_r=0.95), yu("yu_a_004", observer_scale=0.25),
        yu("yu_a_005", observer_scale=0.75), yu("yu_a_006", swing_position_gain=0.3),
        yu("yu_a_007", swing_position_gain=1.4, swing_velocity_gain=0.7),
        yu("yu_a_008", position_gain=3.0, velocity_gain=2.0),
    ]
    yu_b = []
    for index, (r, kp, kd, observer_scale) in enumerate(product((0.70, 0.85, 0.95), (1.5, 3.0), (0.9, 1.8), (0.25, 0.75)), 1):
        yu_b.append(yu(f"yu_b_{index:03d}", finite_time_power_r=r, position_gain=kp, velocity_gain=kd, observer_scale=observer_scale))
    sep_a = [
        sep("sep_a_001"), sep("sep_a_002", horizon_updates=6), sep("sep_a_003", horizon_updates=14),
        sep("sep_a_004", position_weight=25.0), sep("sep_a_005", position_weight=100.0),
        sep("sep_a_006", passivity_rho=0.05, passivity_epsilon=0.005),
        sep("sep_a_007", passivity_rho=0.60, passivity_epsilon=0.05),
        sep("sep_a_008", storage_position_gain=2.0, rate_weight=3.0),
    ]
    sep_b = []
    for index, (horizon, position, rho, epsilon) in enumerate(product((6, 10, 14), (30.0, 80.0), (0.08, 0.40), (0.005, 0.03)), 1):
        sep_b.append(sep(
            f"sep_b_{index:03d}", horizon_updates=horizon, position_weight=position,
            velocity_weight=0.15 * position, passivity_rho=rho, passivity_epsilon=epsilon,
        ))
    if any(len(values) != expected for values, expected in ((yu_a, 8), (yu_b, 24), (sep_a, 8), (sep_b, 24))):
        raise RuntimeError("Paper search budget drift")

    write("literature_review.json", {
        "workflow": "multi-source-search", "search_date": "2026-08-09",
        "routing": "T1 primary publisher/arXiv sources; academic-search MCP unavailable, primary-source web fallback used",
        "selection_completed_before_v6_paper_performance": True,
        "deduplication": "normalized DOI/arXiv identifier, then normalized title and first author",
        "candidates": [
            {
                "paper_id": "YU2026", "title": "Robust finite-time anti-swing control for quadrotor slung-load system based on compensation function observer",
                "authors": "Yu et al.", "venue": "PLOS ONE 21(4):e0331662", "year": 2026,
                "doi": "10.1371/journal.pone.0331662", "peer_reviewed": True, "open_access": True,
                "primary_article": "https://journals.plos.org/plosone/article?id=10.1371/journal.pone.0331662",
                "manuscript_sha256": "d741f5409be7d6bbd1c0016515264e1bad62e3ec621b99b38bd0ec198fa95b34",
                "supplementary_package_sha256": "468a6f8d62269c41382edb6209d7d09c2e2b96defbbbbbbd7bdf8eaf522369ec",
                "supplementary_audit": "Package contains LaTeX manuscript source, figures, and bibliography; it does not contain executable simulation source code.",
                "full_equations_available": True, "selected": True, "selection_role": "Priority Paper A",
            },
            {
                "paper_id": "KANG2026", "title": "Robust control of aerial cable-suspended payload transportation via fully actuated system approach",
                "authors": "Kang and Shan", "venue": "Control Engineering Practice 170:106837", "year": 2026,
                "doi": "10.1016/j.conengprac.2026.106837", "peer_reviewed": True,
                "primary_abstract": "https://www.sciencedirect.com/science/article/pii/S096706612600081X",
                "full_equations_available": False, "selected": False,
                "reason": "Only abstract-level primary content was legally and reliably retrievable before suite freeze; formula guessing is forbidden.",
            },
            {
                "paper_id": "SEP2026", "title": "SEP-NMPC: Safety Enhanced Passivity-Based Nonlinear Model Predictive Control for a UAV Slung Payload System",
                "authors": "Rezaei, Kang, Haridevan, and Shan", "venue": "ICRA 2026 accepted preprint", "year": 2026,
                "arxiv": "2603.08860v1", "primary_full_text": "https://arxiv.org/html/2603.08860v1",
                "tex_source_sha256": "379d9a1734fdf179a48d1ad49a9ee52321eba6eb2238da9eb0c99bc68e2aed6e",
                "html_sha256": "fb283983e1a002aec703069a9f655f415ecbff4bba274fcf7a60137eb8fb51b4",
                "full_equations_available": True, "selected": True, "selection_role": "predeclared fallback for unavailable Priority Paper B",
            },
            {
                "paper_id": "ACTUATORS2025", "title": "Composite Perturbation-Rejection Trajectory-Tracking Control for a Quadrotor-Slung Load System",
                "venue": "Actuators 14(7):335", "year": 2025, "doi": "10.3390/act14070335",
                "full_equations_available": True, "selected": False,
                "reason": "Complete 2026 fallback SEP-NMPC ranked higher on recency and direct constrained-MPC authority compatibility.",
            },
        ],
    })
    write("suite_freeze.json", {
        "status": "V6_PAPER_SUITE_FROZEN_BEFORE_PERFORMANCE", "suite_size": 2,
        "methods": ["YU2026-FT-CFO-ADAPTED-5LINK", "SEP2026-PASSIVITY-NMPC-ADAPTED-5LINK"],
        "priority_b_exclusion": "KANG2026_FULL_TEXT_UNAVAILABLE_NO_FORMULA_GUESSING",
        "replacement_after_freeze_allowed": False, "paper_performance_executed": False,
        "holdout_accessed": False, "holdout_executed": False,
    })
    write("yu2026_adaptation_contract.json", {
        "method": "YU2026-FT-CFO-ADAPTED-5LINK", "label": "adaptation, not exact reproduction",
        "PRESERVED": [
            "Eq. (15) compensation-function observer structure with position and velocity innovations and intermediate z0",
            "Eq. (28) finite-time signed-power virtual velocity structure", "Eq. (39)-(45) payload energy/swing terms",
            "Eq. (49) finite-time position/velocity feedback plus disturbance compensation and swing rejection",
        ],
        "ADAPTED": [
            "single rigid cable and point load mapped to measured five-link cutter-tip relative displacement and velocity",
            "force command U1 mapped to world-frame acceleration [ax, ay, az]",
            "continuous observer integrated at frozen 20 Hz and bounded by the common physical limiter",
        ],
        "OMITTED": ["quadrotor attitude/torque loop because the same frozen geometric inner loop is mandatory"],
        "UNAVAILABLE": ["executable simulation source code; the PLOS supporting package is manuscript source only"],
        "core_innovation_retained": True, "faithfulness": "FAITHFUL_MATHEMATICAL_ADAPTATION",
        "authority": {"output": "world acceleration", "absolute_limit": 2.0, "slew_per_update": 0.25},
    })
    write("sep2026_adaptation_contract.json", {
        "method": "SEP2026-PASSIVITY-NMPC-ADAPTED-5LINK", "label": "adaptation, not exact reproduction",
        "PRESERVED": [
            "Eq. (7a)-(7d) predictive objective, model constraints, authority set, and strict passivity inequality",
            "Eq. (8) shaped storage: kinetic/task velocity, multi-link swing energy proxy, and position shaping",
            "Eq. (10) passivity filter with positive rho and epsilon", "finite-horizon constrained optimization at every 20 Hz update",
        ],
        "ADAPTED": [
            "nonlinear single-cable state mapped to frozen linearized 20-state five-link model and measured task outputs",
            "force/shaped input mapped to world-frame acceleration and common geometric inner loop",
            "payload swing energy mapped to frozen orientation and angular task outputs",
        ],
        "OMITTED": ["obstacle HOCBF inequalities because the frozen benchmark contains no obstacle geometry or obstacle task"],
        "UNAVAILABLE": [],
        "core_innovation_retained": True, "faithfulness": "FAITHFUL_PASSIVITY_NMPC_ADAPTATION_WITH_INAPPLICABLE_OBSTACLE_MODULE_OMITTED",
        "authority": {"output": "world acceleration", "absolute_limit": 2.0, "slew_per_update": 0.25},
    })
    write("development_protocol.json", {
        "contract_head": START_HEAD, "development_manifest": "reproducibility/v6/r0/development_manifest.json",
        "holdout_manifest_loaded": False, "stage_a_sample_ids": stage_a_ids,
        "papers": {
            "YU2026": {"stage_a": yu_a, "stage_b": yu_b, "unique_configuration_count": 32, "maximum": 48},
            "SEP2026": {"stage_a": sep_a, "stage_b": sep_b, "unique_configuration_count": 32, "maximum": 48},
        },
        "selection_rule": read(R0 / "paper_search_budget.json")["selection_rule"],
        "qualification_contract": read(R0 / "win_contract.json")["development_qualification"],
        "paper_performance_executed": False, "holdout_executed": False,
    })
    print(json.dumps({"suite": ["YU2026", "SEP2026"], "stage_a_each": 8, "stage_b_each": 24, "holdout_accessed": False}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
