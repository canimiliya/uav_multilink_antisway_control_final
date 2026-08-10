"""Freeze Development-informed local recovery candidates after initial Stage B."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / "reproducibility/native_stack/r1r3"
OUT = BASE / "recovery_protocol"


def write(name: str, payload: object) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / name).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def common() -> dict:
    return {
        "attitude_wn": 4.0, "attitude_zeta": 0.95,
        "integral_limit": 0.8, "acceleration_limit": 5.0,
        "acceleration_slew_per_s": 10.0,
        "reference_velocity_gain": 1.0, "reference_acceleration_gain": 1.0,
        "tip_correction_kp": 0.05, "tip_correction_kd": 0.02,
        "tip_correction_limit_m": 0.15,
        "swing_angle_gain": 0.01, "swing_rate_gain": 0.05,
        "constraint_margin": 0.88,
        "setpoint_velocity_threshold": 0.02, "setpoint_acceleration_threshold": 0.02,
        "setpoint_terminal_kp_scale": 1.0, "setpoint_terminal_kd_scale": 1.0,
    }


def pid_candidates() -> list[dict]:
    # The first point exactly reproduces native_pid_001 in the R1R3 controller.
    # Remaining coordinates target the observed moderate-wind endpoint speed
    # failure while retaining the incumbent's zero-catastrophe authority.
    designs = [
        (.12,.55,.03,.05,.02,.15,1.0,1.0,.8),
        (.12,.55,.04,.05,.02,.15,1.0,1.0,1.0),
        (.12,.55,.06,.05,.02,.15,1.0,1.0,1.2),
        (.12,.55,.08,.05,.02,.15,1.0,1.0,1.4),
        (.12,.65,.04,.05,.02,.15,1.0,1.0,1.0),
        (.12,.75,.04,.05,.02,.15,1.0,1.0,1.0),
        (.14,.60,.04,.05,.02,.15,1.0,1.0,1.0),
        (.16,.65,.04,.05,.02,.15,1.0,1.0,1.0),
        (.10,.60,.05,.05,.02,.15,1.0,1.0,1.2),
        (.14,.70,.06,.05,.02,.15,1.0,1.0,1.2),
        (.12,.55,.04,.08,.03,.20,1.0,1.0,1.0),
        (.12,.55,.05,.10,.04,.25,1.0,1.0,1.2),
        (.12,.60,.04,.05,.02,.15,1.25,1.25,1.0),
        (.12,.65,.05,.05,.02,.15,1.35,1.50,1.2),
        (.14,.65,.05,.05,.02,.15,1.40,1.60,1.2),
        (.16,.70,.06,.05,.02,.15,1.50,1.75,1.2),
        (.12,.70,.04,.08,.03,.20,1.25,1.50,1.0),
        (.14,.75,.05,.08,.03,.20,1.40,1.75,1.2),
        (.10,.65,.06,.05,.02,.15,1.25,1.75,1.4),
        (.10,.75,.08,.05,.02,.15,1.40,2.00,1.4),
        (.12,.80,.06,.05,.02,.15,1.50,2.00,1.2),
        (.14,.85,.08,.05,.02,.15,1.60,2.20,1.4),
        (.12,.70,.08,.08,.04,.20,1.50,2.00,1.4),
        (.14,.80,.10,.08,.04,.20,1.60,2.20,1.5),
    ]
    result = []
    for index, (kp, kd, ki, tip_kp, tip_kd, tip_limit, terminal_kp, terminal_kd, integral_limit) in enumerate(designs):
        p = common()
        p.update({
            "kp": [kp, kp, kp], "kd": [kd, kd, kd], "ki": [ki, ki, ki],
            "tip_correction_kp": tip_kp, "tip_correction_kd": tip_kd,
            "tip_correction_limit_m": tip_limit, "integral_limit": integral_limit,
            "setpoint_terminal_kp_scale": terminal_kp, "setpoint_terminal_kd_scale": terminal_kd,
        })
        result.append({"method_id": f"r1r3_pid_local_{index:03d}", "family": "native_pid",
                       "outer_rate_hz": 100, "inner_rate_hz": 500, "parameters": p})
    return result


def lqi_candidates() -> list[dict]:
    # Conservative CARE weights generate feedback gains around the safe
    # native_pid_001 neighbourhood rather than the unsafe initial LQI range.
    result = []
    index = 0
    for q_position in (0.0001, 0.001, 0.01, 0.03):
        for q_velocity in (0.05, 0.10, 0.30):
            for q_integral in (0.0001, 0.0004):
                p = common()
                p.update({
                    "kp": [0.0, 0.0, 0.0], "kd": [0.0, 0.0, 0.0], "ki": [0.0, 0.0, 0.0],
                    "q_position": [q_position, q_position, 1.2*q_position],
                    "q_velocity": [q_velocity, q_velocity, 1.1*q_velocity],
                    "q_integral": [q_integral, q_integral, 1.2*q_integral],
                    "r_acceleration": [1.0, 1.0, 1.0],
                    "integral_limit": 1.0 if q_integral == 0.0001 else 1.3,
                    "setpoint_terminal_kp_scale": (1.0, 1.35, 1.6)[index % 3],
                    "setpoint_terminal_kd_scale": (1.0, 1.5, 2.0)[index % 3],
                })
                result.append({"method_id": f"r1r3_lqi_local_{index:03d}", "family": "native_full_lqr",
                               "outer_rate_hz": 100, "inner_rate_hz": 500, "parameters": p})
                index += 1
    return result


def main() -> None:
    pid_summary = BASE / "traditional/pid/development_summary.csv"
    lqi_summary = BASE / "traditional/full_lqr/development_summary.csv"
    if not pid_summary.exists() or not lqi_summary.exists():
        raise RuntimeError("initial full-200 evidence must exist before local recovery freeze")
    pid_best = next(csv.DictReader(pid_summary.open(encoding="utf-8", newline="")))
    lqi_best = next(csv.DictReader(lqi_summary.open(encoding="utf-8", newline="")))
    write("local_recovery_diagnosis.json", {
        "evidence": {"pid_summary_sha256": sha256(pid_summary), "lqi_summary_sha256": sha256(lqi_summary)},
        "pid_observation": {
            "selected": pid_best["method_id"], "all_safety_rate": float(pid_best["all_safety_rate"]),
            "nominal_success_rate": float(pid_best["nominal_success_rate"]),
            "moderate_success_rate": float(pid_best["nominal_moderate_success_rate"]),
            "setpoint_success_rate": float(pid_best["nominal_setpoint_success_rate"]),
            "diagnosis": "safe but under-regulated; recover native_pid_001 exactly then target integral and terminal damping",
        },
        "lqi_observation": {
            "selected": lqi_best["method_id"], "all_safety_rate": float(lqi_best["all_safety_rate"]),
            "nominal_success_rate": float(lqi_best["nominal_success_rate"]),
            "diagnosis": "initial CARE weights were outside the safe low-authority neighbourhood; use conservative weights",
        },
        "benchmark_or_governance_change": False,
    })
    write("native_pid_local_candidate_registry.json", pid_candidates())
    write("native_full_lqr_local_candidate_registry.json", lqi_candidates())
    write("local_recovery_contract.json", {
        "stage_a_manifest": "reproducibility/native_stack/r1r3/protocol/stage_a_manifest.json",
        "configs_per_family": 24, "full_200_selected_per_family": 6,
        "selection_rule": "unchanged frozen P2-R1R3 Stage-A and qualification order",
        "holdout_execution_allowed": False, "satc_execution_allowed": False,
    })
    print(json.dumps({"pid_local": 24, "lqi_local": 24, "frozen": True}, sort_keys=True))


if __name__ == "__main__":
    main()
