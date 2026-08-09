"""Freeze Kang-2026 adaptation parameters and all 96 search configurations."""

from __future__ import annotations

import hashlib
import json
import subprocess
from itertools import product
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v7/r0"
OUT = ROOT / "reproducibility/v7/paper"
CONTRACT_HEAD = "79fa0ce058823f2a09d3d6cdda24105a5a9591d5"


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(name: str, payload: object) -> None:
    path = OUT / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes().replace(b"\r\n", b"\n")).hexdigest()


def candidate(candidate_id: str, k1: float, k2: float, virtual: float, observer_scale: float, *, length: float, l1: float, l2: float, rho: float, clip: float) -> dict:
    return {
        "method": "KANG2026-FAS-DOB-ADAPTED-5LINK", "candidate_id": candidate_id,
        "position_gain": k1, "velocity_gain": k2,
        "virtual_constraint_gain": virtual, "virtual_length_scale": length,
        "observer_lambda1": l1, "observer_lambda2": l2, "observer_rho": rho,
        "observer_clip_m_s2": clip, "observer_compensation_scale": observer_scale,
        "observer_substeps": 5, "axis_scale": [1.0, 1.0, 0.85],
    }


def main() -> int:
    if git("rev-parse", "HEAD") != CONTRACT_HEAD:
        raise RuntimeError("Paper protocol must immediately follow the V7 contract freeze")
    manifest = read(R0 / "development_manifest.json")
    samples = manifest["samples"]
    stage_a_ids = []
    for cohort in ("NORMAL_CALM", "NORMAL_CONSTANT", "STRONG_NEAR_SIMULTANEOUS", "NORMAL_STOCHASTIC"):
        matching = [row["sample_id"] for row in samples if row["cohort"] == cohort]
        stage_a_ids.extend(matching[:6])
    if len(stage_a_ids) != 24 or len(set(stage_a_ids)) != 24:
        raise RuntimeError("Stage-A sample selection drift")

    stage_a = [
        candidate(f"kang_a_{i:03d}", k1, k2, virtual, 0.25, length=0.80, l1=0.8, l2=0.6, rho=0.8, clip=0.6)
        for i, (k1, k2, virtual) in enumerate(product((0.7, 1.1, 1.6), (0.9, 1.4, 2.0, 2.8), (0.55, 0.90)), 1)
    ]
    stage_b = [
        candidate(f"kang_b_{i:03d}", k1, k2, virtual, observer, length=0.90, l1=1.2, l2=1.0, rho=1.0, clip=0.9)
        for i, (k1, k2, virtual, observer) in enumerate(product((0.6, 0.9, 1.3, 1.8), (1.0, 1.6, 2.3), (0.65, 1.0), (0.30, 0.70)), 1)
    ]
    stage_c = [
        candidate(f"kang_c_{i:03d}", k1, k2, virtual, observer, length=1.00, l1=1.7, l2=1.4, rho=1.2, clip=1.1)
        for i, (k1, k2, virtual, observer) in enumerate(product((0.8, 1.15, 1.5), (1.3, 1.9), (0.75, 0.90), (0.45, 0.90)), 1)
    ]
    serialized = [json.dumps({k: v for k, v in value.items() if k != "candidate_id"}, sort_keys=True) for value in stage_a + stage_b + stage_c]
    if (len(stage_a), len(stage_b), len(stage_c), len(set(serialized))) != (24, 48, 24, 96):
        raise RuntimeError("V7 unique-configuration budget drift")
    write("development_protocol.json", {
        "paper": "Kang and Shan 2026", "adaptation": "KANG2026-FAS-DOB-ADAPTED-5LINK",
        "stage_a_sample_ids": stage_a_ids,
        "stages": {
            "stage_a": {"samples_each": 24, "selection_authority": False, "candidates": stage_a},
            "stage_b": {"samples_each": 144, "selection_authority": True, "candidates": stage_b},
            "stage_c": {"samples_each": 144, "selection_authority": True, "candidates": stage_c},
        },
        "unique_configuration_count": 96, "maximum_unique_configurations": 96,
        "selection_rule": "all gates, then gate count, strong P90, position, acquisition, orientation, effort, runtime, candidate id",
        "holdout_accessed": False,
    })
    write("implementation_freeze.json", {
        "status": "V7_KANG2026_IMPLEMENTATION_FROZEN_BEFORE_PERFORMANCE",
        "contract_freeze_head": CONTRACT_HEAD,
        "implementation": "src/uav_sway/v7/kang2026_fas_dob.py::Kang2026FASDOB",
        "sha256": {
            name: sha(ROOT / name) for name in [
                "src/uav_sway/v7/kang2026_fas_dob.py", "scripts/run_v7_development.py",
                "scripts/evaluate_v7_development.py", "docs/v7/paper_equation_mapping.md",
            ]
        },
        "traditional_mutated": False, "satc_mutated": False,
        "paper_performance_executed": False, "holdout_accessed": False,
        "implementation_mutable_after_freeze": False,
    })
    print(json.dumps({"stage_a": 24, "stage_b": 48, "stage_c": 24, "unique": 96}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
