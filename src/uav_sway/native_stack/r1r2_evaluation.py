"""Metric-identical authoritative evaluation for frozen P2-R1R2 candidates."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from .case_semantics.authoritative import AUTHORITATIVE_EXECUTION, AuthoritativeNativeCaseRunner
from .case_semantics.resolver import NativeCaseResolver
from .r1r1_evaluation import AuditedController, aggregate
from .r1r2_controllers import R1R2FullLQR, R1R2NativePID, R1R2TaskLQR


ROOT = Path(__file__).resolve().parents[3]


def build_controller(spec: dict[str, Any]):
    cls = {
        "native_pid": R1R2NativePID,
        "native_full_lqr": R1R2FullLQR,
        "native_task_lqr": R1R2TaskLQR,
    }[spec["family"]]
    return cls(spec["parameters"], spec["method_id"], int(spec["outer_rate_hz"]))


def evaluate_case(payload: tuple[dict[str, Any], dict[str, Any]]) -> dict[str, Any]:
    spec, identity = payload
    resolved = NativeCaseResolver().resolve(identity)
    audited = AuditedController(build_controller(spec))
    runner = AuthoritativeNativeCaseRunner(
        ROOT / "reproducibility/frozen/model/model_5link_controlled.xml", ROOT / "configs/model_5link.yaml",
        ROOT / "configs/aerodynamics.yaml", int(spec["outer_rate_hz"]), int(spec["inner_rate_hz"]),
    )
    result = runner.run_case(audited, resolved)
    if result.execution_authority != AUTHORITATIVE_EXECUTION:
        raise AssertionError("non-authoritative result")
    records = audited.records
    time = np.array([x[0] for x in records]); pos = np.array([x[1] for x in records]); vel = np.array([x[2] for x in records])
    ref = np.array([x[3] for x in records]); ref_vel = np.array([x[4] for x in records]); orient = np.array([x[5] for x in records])
    error = np.linalg.norm(pos - ref, axis=1); velocity_error = np.linalg.norm(vel - ref_vel, axis=1)
    mask = time >= resolved.issue_offset_s; error_eval = error[mask]; velocity_eval = velocity_error[mask]
    final_error = float(error[-1]); final_speed = float(velocity_error[-1])
    acquired = np.inf; required = max(1, int(round(.5 / (audited.decimation * .001))))
    good = (error <= .5) & (velocity_error <= .5) & (time >= resolved.issue_offset_s); count = 0
    for i, ok in enumerate(good):
        count = count + 1 if ok else 0
        if count >= required:
            acquired = float(time[i - required + 1] - resolved.issue_offset_s); break
    commands = np.array([r.actual for r in result.command_records]); rates = np.array([r.command_rate_per_s for r in result.command_records])
    thrust_sat = np.array([r.thrust_saturated for r in result.command_records]); torque_sat = np.array([any(r.torque_saturated) for r in result.command_records])
    all_ms = np.asarray(audited.outer_ms + audited.inner_ms)
    deadlines = np.r_[np.full(len(audited.outer_ms), 1000 / spec["outer_rate_hz"]),
                      np.full(len(audited.inner_ms), 1000 / spec["inner_rate_hz"])]
    return {
        "method_id": spec["method_id"], "family": spec["family"], "sample_id": resolved.sample_id,
        "task_family": resolved.identity["task_family"], "wind_kind": resolved.identity["wind_kind"],
        "wind_direction": resolved.identity["wind_direction"], "outer_rate_hz": spec["outer_rate_hz"],
        "inner_rate_hz": spec["inner_rate_hz"], "safe": bool(result.safe), "catastrophic": not bool(result.safe),
        "success": bool(result.safe and final_error <= .5 and final_speed <= .5),
        "position_rmse_m": float(np.sqrt(np.mean(error_eval ** 2))), "position_p90_m": float(np.percentile(error_eval, 90)),
        "position_p95_m": float(np.percentile(error_eval, 95)), "velocity_rmse_m_s": float(np.sqrt(np.mean(velocity_eval ** 2))),
        "orientation_rmse_rad": float(np.sqrt(np.mean(orient[mask] ** 2))), "endpoint_position_error_m": final_error,
        "endpoint_velocity_error_m_s": final_speed, "acquisition_time_s": None if not np.isfinite(acquired) else acquired,
        "physical_effort": float(np.mean(np.sum(commands ** 2, axis=1))), "thrust_saturation_rate": float(np.mean(thrust_sat)),
        "torque_saturation_rate": float(np.mean(torque_sat)), "wrench_rate_rms": float(np.sqrt(np.mean(rates ** 2))),
        "runtime_mean_ms": float(np.mean(all_ms)), "runtime_p95_ms": float(np.percentile(all_ms, 95)),
        "runtime_p99_ms": float(np.percentile(all_ms, 99)), "runtime_max_ms": float(np.max(all_ms)),
        "deadline_miss_rate": float(np.mean(all_ms > deadlines)), "execution_authority": result.execution_authority,
        "semantic_fingerprint": result.case_semantic_fingerprint,
    }


def competence(summary: dict[str, Any]) -> bool:
    return bool(summary["safety_rate"] >= .98 and summary["success_rate"] >= .70
                and summary["setpoint_position_rmse_m"] <= 1.25
                and summary["trajectory_position_rmse_m"] <= 1.50
                and summary["catastrophic_count"] <= 4 and summary["deadline_miss_rate"] <= .01)


__all__ = ["aggregate", "build_controller", "competence", "evaluate_case"]

