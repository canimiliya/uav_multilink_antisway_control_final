"""Freeze the Xu2025 adaptation and its deterministic Development search."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v8/r0"
PAPER = ROOT / "reproducibility/v8/paper"
IMPLEMENTATION = ROOT / "src/uav_sway/v8/xu2025_cbs_ftdo.py"
SEARCH_SEED = 20260827


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def lhs(count: int, dimensions: int, seed: int) -> np.ndarray:
    rng = np.random.Generator(np.random.PCG64(seed))
    result = np.empty((count, dimensions), dtype=float)
    for column in range(dimensions):
        result[:, column] = (rng.permutation(count) + rng.random(count)) / count
    return result


def scaled(value: float, low: float, high: float) -> float:
    return round(low + value * (high - low), 8)


def candidates() -> list[dict]:
    modal = read(R0 / "modal_selection_contract.json")
    fixed = {
        "architecture": "XU2025-CBS-FTDO-ADAPTED-5LINK",
        "modal_basis": modal["modal_basis_mass_normalized"],
        "modal_projection": modal["joint_to_modal_projection"],
        "effective_length_m": 2.81,
    }
    anchors = [
        (0.8, 2.4, 5.0, 20.0, 15.0, 15.0, 1.0, 2.0, 0.03, 1.5, 0.20, 0.40, 1.0),
        (1.2, 3.0, 3.0, 6.0, 8.0, 8.0, 0.5, 1.0, 0.04, 1.0, 0.00, 0.20, 1.0),
        (1.5, 3.5, 2.5, 4.0, 6.0, 6.0, 0.4, 0.8, 0.05, 0.8, 0.00, 0.10, 0.9),
        (2.0, 4.0, 6.0, 10.0, 12.0, 12.0, 0.8, 1.5, 0.03, 1.5, 0.25, 0.30, 1.1),
    ]
    values = []
    for row in anchors:
        values.append(row)
    for row in lhs(124, 13, SEARCH_SEED):
        values.append(
            (
                scaled(row[0], 0.55, 2.40),
                scaled(row[1], 2.05, 5.00),
                scaled(row[2], 2.05, 10.0),
                scaled(row[3], 1.60, 20.0),
                scaled(row[4], 4.0, 18.0),
                scaled(row[5], 4.0, 18.0),
                scaled(row[6], 0.20, 4.0),
                scaled(row[7], 0.20, 6.0),
                scaled(row[8], 0.01, 0.08),
                scaled(row[9], 0.50, 3.0),
                scaled(row[10], 0.0, 0.60),
                scaled(row[11], 0.05, 0.80),
                scaled(row[12], 0.65, 1.35),
            )
        )
    result = []
    for index, row in enumerate(values):
        (
            kp, kv, kq, kw, kf, kfg, kd1, kd2, boundary, clip,
            compensation, coupling, z_scale,
        ) = row
        result.append(
            {
                "candidate_id": f"xu_v8_{index:03d}",
                **fixed,
                "axis_scale": [1.0, 1.0, z_scale],
                "position_gain": kp,
                "velocity_gain": kv,
                "direction_gain": kq,
                "angular_gain": kw,
                "direction_filter_gain": kf,
                "angular_filter_gain": kfg,
                "force_observer_k1": kd1,
                "force_observer_k2": kd2,
                "angular_observer_k1": kd1,
                "angular_observer_k2": kd2,
                "observer_boundary": boundary,
                "observer_clip": clip,
                "observer_compensation_scale": compensation,
                "angular_coupling_scale": coupling,
            }
        )
    assert len(result) == 128
    assert len({row["candidate_id"] for row in result}) == 128
    return result


def main() -> int:
    source = read(R0 / "paper_source_freeze.json")
    if source["adaptation_name"] != "XU2025-CBS-FTDO-ADAPTED-5LINK":
        raise RuntimeError("Paper source drift")
    manifest = read(R0 / "development_manifest.json")
    core_ids = []
    by_cohort: dict[str, list[str]] = {}
    for sample in manifest["samples"]:
        by_cohort.setdefault(sample["cohort"], []).append(sample["sample_id"])
    for cohort in sorted(by_cohort):
        core_ids.extend(by_cohort[cohort][:4])
    if len(core_ids) != 24:
        raise RuntimeError("expected four core cases from six frozen cohorts")
    rows = candidates()
    protocol = {
        "paper": source,
        "implementation_sha256_before_performance": sha256(IMPLEMENTATION),
        "search_seed": SEARCH_SEED,
        "max_unique_configurations": 128,
        "candidate_registry": "reproducibility/v8/paper/candidate_registry.json",
        "parameter_ranges": {
            "position_gain": [0.55, 2.40],
            "velocity_gain": [2.05, 5.00],
            "direction_gain": [2.05, 10.0],
            "angular_gain": [1.60, 20.0],
            "direction_filter_gain": [4.0, 18.0],
            "angular_filter_gain": [4.0, 18.0],
            "observer_k1": [0.20, 4.0],
            "observer_k2": [0.20, 6.0],
            "observer_boundary": [0.01, 0.08],
            "observer_clip": [0.50, 3.0],
            "observer_compensation_scale": [0.0, 0.60],
            "angular_coupling_scale": [0.05, 0.80],
            "z_axis_dimensional_scale": [0.65, 1.35],
        },
        "stages": {
            "stage_0": {
                "purpose": "formula, unit, frame, reset, and limiter validation",
                "plant_performance": False,
            },
            "stage_a": {
                "candidate_ids": [f"xu_v8_{value:03d}" for value in range(0, 128, 8)],
                "sample_ids": core_ids,
                "sample_count_each": 24,
                "selection_authority": False,
            },
            "stage_b": {
                "candidate_ids": [row["candidate_id"] for row in rows],
                "sample_count_each": 144,
                "selection_authority": True,
            },
            "stage_c": {
                "trigger": "one or more Stage-B candidates pass every hard gate",
                "candidate_count": 1,
                "candidate_rule": "preregistered lexicographic winner only",
                "sample_count_each": 144,
                "purpose": "fresh-cache deterministic confirmation; no parameter change",
            },
        },
        "selection_rule": [
            "all hard gates pass",
            "highest success rate",
            "lowest strong-position P90",
            "lowest overall position RMSE",
            "lowest acquisition median",
            "lowest orientation RMSE",
            "lowest effort",
            "lowest runtime",
            "candidate_id",
        ],
        "development_manifest_sha256": sha256(R0 / "development_manifest.json"),
        "holdout_manifest_sha256": sha256(R0 / "holdout_manifest.json"),
        "holdout_execution_allowed": False,
        "bootstrap": {"resamples": 10000, "seed": 20260825, "pairing": "exact sample_id"},
        "paper_native_parameters_only": True,
        "satc_used_for_selection": False,
        "performance_accessed": False,
    }
    write(PAPER / "candidate_registry.json", {"unique_candidates": 128, "candidates": rows})
    write(PAPER / "development_protocol.json", protocol)
    write(
        PAPER / "implementation_audit.json",
        {
            "adaptation": "XU2025-CBS-FTDO-ADAPTED-5LINK",
            "paper_core_retained": [
                "load-position virtual velocity loop",
                "load-velocity virtual force loop",
                "desired cable-direction command filter",
                "cable-direction virtual angular-rate loop",
                "angular-rate command filter and control loop",
                "finite-time force and angular disturbance observers",
                "parallel/perpendicular force synthesis",
            ],
            "five_link_mapping": "plant-only modes 1 and 2 reconstruct the equivalent payload-center direction",
            "common_output": "world-frame acceleration [ax, ay, az]",
            "paper_thrust_attitude_inner_loop_used": False,
            "common_geometric_inner_loop_used": True,
            "satc_components_present": False,
            "unfaithful_pd_plus_sign_reduction": False,
            "performance_accessed": False,
        },
    )
    write(
        PAPER / "protocol_sha256.json",
        {
            "candidate_registry": sha256(PAPER / "candidate_registry.json"),
            "development_protocol": sha256(PAPER / "development_protocol.json"),
            "implementation_audit": sha256(PAPER / "implementation_audit.json"),
            "equation_mapping": sha256(PAPER / "equation_mapping.md"),
            "implementation": sha256(IMPLEMENTATION),
        },
    )
    print(json.dumps({"unique_candidates": len(rows), "stage_a_cases": len(core_ids), "implementation_sha256": sha256(IMPLEMENTATION)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
