"""Build the P2-R0 audit, protocols, and unopened native data banks."""

from __future__ import annotations

import hashlib
import json
import subprocess
from collections import Counter
from pathlib import Path

import mujoco
import numpy as np

from uav_sway.native_stack.api import SensorPacket
from uav_sway.native_stack.scheduler import SUPPORTED_RATES_HZ

ROOT = Path(__file__).resolve().parents[1]
SOURCE = "v10-research-final-2026-08-10"
SOURCE_HEAD = "263e78859097592c4dfca5a86a6486606c1cb84f"
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"
OUT = ROOT / "reproducibility/native_stack/r0"
DOCS = ROOT / "docs/native_stack"


def dump(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def digest(value: object) -> str:
    raw = json.dumps(value, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(raw).hexdigest()


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def bank(split: str, count: int, seed_base: int) -> dict:
    winds = ("calm", "moderate", "strong_sustained", "strong_transient", "stochastic", "ramp")
    directions = ("aligned", "opposed", "cross")
    smooth_types = ("minimum_jerk", "approach_stop", "waypoint_3d")
    cases = []
    for index in range(count):
        family = "setpoint" if index % 2 == 0 else "smooth_trajectory"
        wind_kind = winds[(index // 2) % len(winds)]
        direction = directions[(index // (2 * len(winds))) % len(directions)]
        identity = seed_base + index
        cases.append({
            "sample_id": f"native_{split}_{index:03d}",
            "split": split,
            "task_family": family,
            "trajectory_type": "step" if family == "setpoint" else smooth_types[(index // 2) % 3],
            "target_id": f"{split}_target_{identity}",
            "target_seed": identity,
            "trajectory_seed": identity + 10000,
            "wind_kind": wind_kind,
            "wind_seed": identity + 20000,
            "wind_direction": direction,
            "timing_id": f"{split}_timing_{identity}",
            "issue_offset_s": round(0.25 + 0.05 * (index % 11), 3),
            "duration_s": 12.0,
            "reference_preview_allowed": False,
            "execution_allowed": split == "development",
        })
    payload = {
        "schema": "native_stack_bank_v1", "split": split, "case_count": count,
        "execution_allowed": split == "development", "executed": False,
        "cases": cases,
    }
    payload["manifest_sha256"] = digest(cases)
    return payload


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    names = ("rotor_motor_0", "rotor_motor_1", "rotor_motor_2", "rotor_motor_3", "thrust_motor", "mx_motor", "my_motor", "mz_motor")
    ids = {name: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)) for name in names}
    direct_names = ("thrust_motor", "mx_motor", "my_motor", "mz_motor")
    direct_ids = [ids[name] for name in direct_names]
    audit = {
        "source_tag": SOURCE, "source_head": SOURCE_HEAD,
        "model_path": "reproducibility/frozen/model/model_5link_controlled.xml",
        "model_sha256": hashlib.sha256(MODEL.read_bytes()).hexdigest(),
        "physics": {"timestep_s": float(model.opt.timestep), "rate_hz": int(round(1/model.opt.timestep)), "integrator": "RK4"},
        "all_physical_actuator_names": list(names),
        "historical_execution_actuators": list(direct_names),
        "historical_command_dimension": 4,
        "canonical_physical_api": "WrenchCommand[T,tau_x,tau_y,tau_z]",
        "ctrlrange": {name: model.actuator_ctrlrange[ids[name]].tolist() for name in names},
        "gear": {name: model.actuator_gear[ids[name]].tolist() for name in names},
        "dyntype": {name: int(model.actuator_dyntype[ids[name]]) for name in names},
        "direct_wrench_limits": {"thrust_N": [0.0, 285.74568], "torque_Nm": [[-25.0, 25.0], [-25.0, 25.0], [-12.0, 12.0]]},
        "actuator_dynamics_present": False, "actuator_lag_present": False,
        "actuator_rate_limits_present": False, "motor_dynamics_present": False,
        "commanded_vs_applied": "direct MuJoCo motor with dyntype=none; clipped ctrl is actual applied actuator command",
        "timing": {"physics_hz": 1000, "wind_hz": 200, "geometric_inner_hz": 200, "legacy_outer_hz": 20,
                   "hold": "zero-order hold", "update_order": "sample -> due controller updates -> apply command -> integrate physics"},
        "plant_changed": False,
        "audit_complete": bool(all(index >= 0 for index in ids.values()) and np.all(model.actuator_dyntype[direct_ids] == 0)),
    }
    dump(OUT / "physical_actuation_audit.json", audit)

    development = bank("development", 200, 310000)
    holdout = bank("holdout", 140, 710000)
    dump(OUT / "native_development_manifest.json", development)
    dump(OUT / "native_holdout_manifest.json", holdout)
    dev_cases, hold_cases = development["cases"], holdout["cases"]
    fields = ("target_id", "wind_seed", "trajectory_seed", "timing_id")
    intersections = {field: sorted(set(c[field] for c in dev_cases) & set(c[field] for c in hold_cases)) for field in fields}
    integrity = {
        "development_count": len(dev_cases), "holdout_count": len(hold_cases),
        "intersections": intersections,
        "all_identity_intersections_empty": all(not value for value in intersections.values()),
        "future_information_leakage": False,
        "holdout_executed": False, "holdout_execution_allowed": False,
        "balance": {
            "development_task": Counter(c["task_family"] for c in dev_cases),
            "holdout_task": Counter(c["task_family"] for c in hold_cases),
            "development_wind": Counter(c["wind_kind"] for c in dev_cases),
            "holdout_wind": Counter(c["wind_kind"] for c in hold_cases),
            "development_direction": Counter(c["wind_direction"] for c in dev_cases),
            "holdout_direction": Counter(c["wind_direction"] for c in hold_cases),
        },
    }
    dump(OUT / "split_integrity.json", integrity)
    dump(OUT / "benchmark_protocol.json", {
        "version": "native-stack-benchmark-v1", "plant": audit["model_path"], "plant_sha256": audit["model_sha256"],
        "physical_api": audit["canonical_physical_api"], "supported_rates_hz": list(SUPPORTED_RATES_HZ),
        "comparison_modes": {
            "equal_rate": "all compared controllers use one preregistered common rate",
            "native_rate": "each controller uses its preregistered native rate within the supported set",
        },
        "sensor_fields": list(SensorPacket.field_names()),
        "forbidden_runtime_fields": ["true_wind", "future_wind", "future_reference", "future_state", "hidden_mujoco_force", "holdout_metadata"],
        "reference_preview": {"v1_default": False, "future_protocol_must_grant_equally": True},
        "safety_v2": {
            "legacy_rules_preserved": True, "uav_horizontal_displacement_max_m": 8.0,
            "tip_horizontal_displacement_max_m": 10.0,
            "rationale": "The native mission bank is confined to targets within 2 m of trim; limits reserve at least 4x target radius plus suspended-tool reach and are frozen before controller performance.",
        },
        "performance_authority": "NO_SELECTION_AUTHORITY", "new_controller_performance": False,
        "old_holdout_accessed": False, "native_holdout_executed": False,
    })

    protected_paths = ["reproducibility/v2", "reproducibility/v3", "reproducibility/v4", "reproducibility/v5", "reproducibility/v6", "reproducibility/v7", "reproducibility/v8", "reproducibility/v9", "reproducibility/v10", "docs/v10"]
    trees = {}
    for path in protected_paths:
        trees[path] = {"source": git("rev-parse", f"{SOURCE}:{path}"), "current": git("rev-parse", f"HEAD:{path}")}
        trees[path]["unchanged"] = trees[path]["source"] == trees[path]["current"]
    tags = {tag: git("rev-parse", f"{tag}^{{}}") for tag in git("tag", "--list", "v*-research-final-2026-08-*").splitlines()}
    dump(OUT / "protected_evidence_audit.json", {"source": SOURCE, "paths": trees, "tags": tags, "all_unchanged": all(v["unchanged"] for v in trees.values())})

    DOCS.mkdir(parents=True, exist_ok=True)
    (DOCS / "CURRENT_CONTROL_STACK.md").write_text(f"""# Current frozen control stack\n\nThe audited V1-V10 execution path uses the byte-frozen MuJoCo model `{audit['model_path']}` (SHA-256 `{audit['model_sha256']}`). Physics integrates at 1000 Hz with RK4. Wind and the shared geometric inner loop update at 200 Hz, and the legacy acceleration outer loop updates at 20 Hz. Both controller levels use zero-order hold.\n\nThe formal runners write `thrust_motor`, `mx_motor`, `my_motor`, and `mz_motor`: direct body-z thrust plus body torque with limits `[0, 285.74568] N`, `±25`, `±25`, and `±12 N m`. Four rotor actuators exist in the XML but are not the historical formal execution path. No actuator dynamics, lag, rate limit, or motor model is present. Native v1 therefore standardizes the audited direct wrench and does not alter the plant.\n""", encoding="utf-8")


if __name__ == "__main__":
    main()
