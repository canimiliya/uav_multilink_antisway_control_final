"""Authoritative Governance-v2 evaluation for P2-R1R3 candidates."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from .case_semantics.authoritative import AUTHORITATIVE_EXECUTION, AuthoritativeNativeCaseRunner
from .case_semantics.resolver import NativeCaseResolver
from .r1r1_evaluation import AuditedController
from .r1r3_controllers import GovernanceV2LQI, GovernanceV2PID, GovernanceV2SATC


ROOT = Path(__file__).resolve().parents[3]
NOMINAL_WINDS = ("calm", "moderate", "stochastic")
CHALLENGE_WINDS = ("strong_sustained", "strong_transient", "ramp")


def build_controller(spec: dict[str, Any]):
    classes = {
        "native_pid": GovernanceV2PID,
        "native_full_lqr": GovernanceV2LQI,
        "satc_native": GovernanceV2SATC,
    }
    return classes[spec["family"]](spec["parameters"], spec["method_id"], int(spec["outer_rate_hz"]))


def evaluate_case(payload: tuple[dict[str, Any], dict[str, Any]]) -> dict[str, Any]:
    spec, identity = payload
    resolved = NativeCaseResolver().resolve(identity)
    audited = AuditedController(build_controller(spec))
    runner = AuthoritativeNativeCaseRunner(
        ROOT / "reproducibility/frozen/model/model_5link_controlled.xml",
        ROOT / "configs/model_5link.yaml",
        ROOT / "configs/aerodynamics.yaml",
        int(spec["outer_rate_hz"]),
        int(spec["inner_rate_hz"]),
    )
    result = runner.run_case(audited, resolved)
    if result.execution_authority != AUTHORITATIVE_EXECUTION:
        raise AssertionError("non-authoritative result")
    records = audited.records
    time = np.array([x[0] for x in records])
    pos = np.array([x[1] for x in records])
    vel = np.array([x[2] for x in records])
    ref = np.array([x[3] for x in records])
    ref_vel = np.array([x[4] for x in records])
    orient = np.array([x[5] for x in records])
    error = np.linalg.norm(pos - ref, axis=1)
    velocity_error = np.linalg.norm(vel - ref_vel, axis=1)
    mask = time >= resolved.issue_offset_s
    final_error = float(error[-1])
    final_speed = float(velocity_error[-1])
    acquired = np.inf
    required = max(1, int(round(0.5 / (audited.decimation * 0.001))))
    good = (error <= 0.5) & (velocity_error <= 0.5) & (time >= resolved.issue_offset_s)
    count = 0
    for index, ok in enumerate(good):
        count = count + 1 if ok else 0
        if count >= required:
            acquired = float(time[index - required + 1] - resolved.issue_offset_s)
            break
    commands = np.array([record.actual for record in result.command_records])
    rates = np.array([record.command_rate_per_s for record in result.command_records])
    thrust_sat = np.array([record.thrust_saturated for record in result.command_records])
    torque_sat = np.array([any(record.torque_saturated) for record in result.command_records])
    all_ms = np.asarray(audited.outer_ms + audited.inner_ms)
    deadlines = np.r_[
        np.full(len(audited.outer_ms), 1000 / spec["outer_rate_hz"]),
        np.full(len(audited.inner_ms), 1000 / spec["inner_rate_hz"]),
    ]
    return {
        "method_id": spec["method_id"], "family": spec["family"], "sample_id": resolved.sample_id,
        "task_family": resolved.identity["task_family"], "wind_kind": resolved.identity["wind_kind"],
        "wind_direction": resolved.identity["wind_direction"], "outer_rate_hz": spec["outer_rate_hz"],
        "inner_rate_hz": spec["inner_rate_hz"], "safe": bool(result.safe),
        "catastrophic": not bool(result.safe),
        "success": bool(result.safe and final_error <= 0.5 and final_speed <= 0.5),
        "position_rmse_m": float(np.sqrt(np.mean(error[mask] ** 2))),
        "position_p90_m": float(np.percentile(error[mask], 90)),
        "velocity_rmse_m_s": float(np.sqrt(np.mean(velocity_error[mask] ** 2))),
        "orientation_rmse_rad": float(np.sqrt(np.mean(orient[mask] ** 2))),
        "endpoint_position_error_m": final_error, "endpoint_velocity_error_m_s": final_speed,
        "acquisition_time_s": None if not np.isfinite(acquired) else acquired,
        "physical_effort": float(np.mean(np.sum(commands ** 2, axis=1))),
        "thrust_saturation_rate": float(np.mean(thrust_sat)),
        "torque_saturation_rate": float(np.mean(torque_sat)),
        "wrench_rate_rms": float(np.sqrt(np.mean(rates ** 2))),
        "runtime_mean_ms": float(np.mean(all_ms)), "runtime_p95_ms": float(np.percentile(all_ms, 95)),
        "runtime_p99_ms": float(np.percentile(all_ms, 99)), "runtime_max_ms": float(np.max(all_ms)),
        "deadline_miss_rate": float(np.mean(all_ms > deadlines)),
        "execution_authority": result.execution_authority,
        "semantic_fingerprint": result.case_semantic_fingerprint,
    }


def aggregate_v2(rows: list[dict[str, Any]]) -> dict[str, Any]:
    if not rows:
        raise ValueError("cannot aggregate an empty result set")

    def subset(*, winds: tuple[str, ...] | None = None, task: str | None = None) -> list[dict[str, Any]]:
        return [row for row in rows if (winds is None or row["wind_kind"] in winds)
                and (task is None or row["task_family"] == task)]

    def mean(key: str, values: list[dict[str, Any]]) -> float:
        return float(np.mean([row[key] for row in values]))

    nominal = subset(winds=NOMINAL_WINDS)
    challenge = subset(winds=CHALLENGE_WINDS)
    nominal_setpoint = subset(winds=NOMINAL_WINDS, task="setpoint")
    nominal_trajectory = subset(winds=NOMINAL_WINDS, task="smooth_trajectory")
    challenge_setpoint = subset(winds=CHALLENGE_WINDS, task="setpoint")
    challenge_trajectory = subset(winds=CHALLENGE_WINDS, task="smooth_trajectory")
    strong = subset(winds=("strong_sustained", "strong_transient"))
    acquisitions = [row["acquisition_time_s"] for row in rows if row["task_family"] == "setpoint"
                    and row["acquisition_time_s"] is not None]
    result: dict[str, Any] = {
        "method_id": rows[0]["method_id"], "family": rows[0]["family"], "case_count": len(rows),
        "all_safety_rate": mean("safe", rows),
        "all_catastrophic_count": int(sum(row["catastrophic"] for row in rows)),
        "all_deadline_miss_rate": mean("deadline_miss_rate", rows),
        "overall_success_rate": mean("success", rows),
        "nominal_case_count": len(nominal), "nominal_success_rate": mean("success", nominal),
        "nominal_setpoint_success_rate": mean("success", nominal_setpoint),
        "nominal_trajectory_success_rate": mean("success", nominal_trajectory),
        "nominal_setpoint_rmse_m": mean("position_rmse_m", nominal_setpoint),
        "nominal_trajectory_rmse_m": mean("position_rmse_m", nominal_trajectory),
        "nominal_endpoint_position_error_m": mean("endpoint_position_error_m", nominal),
        "nominal_endpoint_velocity_error_m_s": mean("endpoint_velocity_error_m_s", nominal),
        "challenge_case_count": len(challenge), "challenge_success_rate": mean("success", challenge),
        "challenge_setpoint_rmse_m": mean("position_rmse_m", challenge_setpoint),
        "challenge_trajectory_rmse_m": mean("position_rmse_m", challenge_trajectory),
        "challenge_rmse_m": mean("position_rmse_m", challenge),
        "challenge_endpoint_position_error_m": mean("endpoint_position_error_m", challenge),
        "challenge_endpoint_velocity_error_m_s": mean("endpoint_velocity_error_m_s", challenge),
        "strong_mean_m": mean("position_rmse_m", strong),
        "strong_p90_m": float(np.percentile([row["position_rmse_m"] for row in strong], 90)),
        "orientation_rmse_rad": mean("orientation_rmse_rad", rows),
        "acquisition_time_s": float(np.median(acquisitions)) if acquisitions else None,
        "physical_effort": mean("physical_effort", rows),
        "thrust_saturation_rate": mean("thrust_saturation_rate", rows),
        "torque_saturation_rate": mean("torque_saturation_rate", rows),
        "wrench_rate_rms": mean("wrench_rate_rms", rows),
        "runtime_mean_ms": mean("runtime_mean_ms", rows),
        "runtime_p95_ms": float(np.percentile([row["runtime_p95_ms"] for row in rows], 95)),
        "runtime_p99_ms": float(np.percentile([row["runtime_p99_ms"] for row in rows], 99)),
        "runtime_max_ms": max(row["runtime_max_ms"] for row in rows),
    }
    for wind in NOMINAL_WINDS:
        values = subset(winds=(wind,))
        result[f"nominal_{wind}_success_rate"] = mean("success", values)
    return result


def competence_v2(summary: dict[str, Any]) -> bool:
    return bool(
        summary["case_count"] == 200
        and summary["nominal_case_count"] == 100
        and summary["challenge_case_count"] == 100
        and summary["all_safety_rate"] >= 0.98
        and summary["all_catastrophic_count"] <= 4
        and summary["all_deadline_miss_rate"] <= 0.01
        and summary["nominal_success_rate"] >= 0.70
        and all(summary[f"nominal_{wind}_success_rate"] >= 0.50 for wind in NOMINAL_WINDS)
        and summary["nominal_setpoint_success_rate"] >= 0.60
        and summary["nominal_trajectory_success_rate"] >= 0.60
        and summary["nominal_setpoint_rmse_m"] <= 1.25
        and summary["nominal_trajectory_rmse_m"] <= 1.50
    )


__all__ = ["aggregate_v2", "build_controller", "competence_v2", "evaluate_case"]
