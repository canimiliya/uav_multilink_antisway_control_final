"""Authoritative Development evaluation and metric reduction for P2-R1R1."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from time import perf_counter_ns
from typing import Any

import numpy as np

from .api import SensorPacket, WrenchCommand
from .case_semantics.authoritative import AUTHORITATIVE_EXECUTION, AuthoritativeNativeCaseRunner
from .case_semantics.resolver import NativeCaseResolver
from .controller import NativeStackController
from .r1r1_controllers import LegacyTaskLevelAdapter, NativeFullLQR, NativeGains, NativePID, NativeTaskLQR, SATCNative


ROOT = Path(__file__).resolve().parents[3]


class AuditedController(NativeStackController):
    """Transparent wrapper recording only causal packets seen by the controller."""
    def __init__(self, delegate: NativeStackController, decimation: int = 10) -> None:
        self.delegate = delegate; self.decimation = int(decimation); self._sensor_packet = None
        self.records: list[tuple] = []; self.outer_ms: list[float] = []; self.inner_ms: list[float] = []
    def reset(self) -> None:
        self.delegate.reset(); self.records.clear(); self.outer_ms.clear(); self.inner_ms.clear()
    def observe(self, packet: SensorPacket) -> None:
        self._sensor_packet = packet; self.delegate.observe(packet)
        if packet.tick % self.decimation == 0:
            r = packet.rotation_world_from_body
            attitude_angle = float(np.arccos(np.clip((np.trace(r)-1.0)*.5, -1., 1.)))
            cutter_orientation = attitude_angle + abs(float(np.sum(packet.joint_position)))
            self.records.append((packet.time_s, packet.cutter_tip_position_world.copy(), packet.cutter_tip_velocity_world.copy(), packet.reference.position_world.copy(), packet.reference.velocity_world.copy(), cutter_orientation))
    def update_high_level(self) -> None:
        start = perf_counter_ns(); self.delegate.update_high_level(); self.outer_ms.append((perf_counter_ns()-start)*1e-6)
    def update_inner(self) -> None:
        start = perf_counter_ns(); self.delegate.update_inner(); self.inner_ms.append((perf_counter_ns()-start)*1e-6)
    def physical_command(self) -> WrenchCommand: return self.delegate.physical_command()
    def diagnostics(self) -> dict[str, Any]: return self.delegate.diagnostics()


def build_controller(spec: dict[str, Any]) -> NativeStackController:
    family = spec["family"]
    if family == "incumbent": return LegacyTaskLevelAdapter(spec["method_id"], spec["kp"], spec["kd"], spec.get("tip_kp", .10), spec.get("tip_kd", .05), spec.get("historical_id"))
    cls = {"native_pid": NativePID, "native_full_lqr": NativeFullLQR, "native_task_lqr": NativeTaskLQR, "satc_native": SATCNative}[family]
    gains = NativeGains(**spec["gains"]) if isinstance(spec["gains"], dict) else spec["gains"]
    kwargs = {"gains": gains, "method_id": spec["method_id"]}
    if family in {"native_full_lqr", "native_task_lqr"}: kwargs["q"] = spec["q"]
    return cls(**kwargs)


def evaluate_case(payload: tuple[dict[str, Any], dict[str, Any], int, int]) -> dict[str, Any]:
    spec, identity, outer_rate, inner_rate = payload
    resolved = NativeCaseResolver().resolve(identity)
    audited = AuditedController(build_controller(spec))
    runner = AuthoritativeNativeCaseRunner(
        ROOT / "reproducibility/frozen/model/model_5link_controlled.xml", ROOT / "configs/model_5link.yaml",
        ROOT / "configs/aerodynamics.yaml", outer_rate, inner_rate,
    )
    result = runner.run_case(audited, resolved)
    if result.execution_authority != AUTHORITATIVE_EXECUTION: raise AssertionError("non-authoritative result")
    records = audited.records
    time = np.array([x[0] for x in records]); pos = np.array([x[1] for x in records]); vel = np.array([x[2] for x in records])
    ref = np.array([x[3] for x in records]); ref_vel = np.array([x[4] for x in records]); orient = np.array([x[5] for x in records])
    error = np.linalg.norm(pos-ref, axis=1); velocity_error = np.linalg.norm(vel-ref_vel, axis=1)
    mask = time >= resolved.issue_offset_s; error_eval = error[mask]; velocity_eval = velocity_error[mask]
    final_error = float(error[-1]); final_speed = float(velocity_error[-1])
    acquired = np.inf; required = max(1, int(round(.5/(audited.decimation*.001))))
    good = (error <= .5) & (velocity_error <= .5) & (time >= resolved.issue_offset_s)
    count = 0
    for i, ok in enumerate(good):
        count = count+1 if ok else 0
        if count >= required: acquired = float(time[i-required+1]-resolved.issue_offset_s); break
    commands = np.array([r.actual for r in result.command_records]); rates = np.array([r.command_rate_per_s for r in result.command_records])
    thrust_sat = np.array([r.thrust_saturated for r in result.command_records]); torque_sat = np.array([any(r.torque_saturated) for r in result.command_records])
    all_ms = np.asarray(audited.outer_ms + audited.inner_ms); deadlines = np.r_[np.full(len(audited.outer_ms),1000/outer_rate),np.full(len(audited.inner_ms),1000/inner_rate)]
    return {"method_id": spec["method_id"], "family": spec["family"], "sample_id": resolved.sample_id,
        "task_family": resolved.identity["task_family"], "wind_kind": resolved.identity["wind_kind"], "wind_direction": resolved.identity["wind_direction"],
        "safe": bool(result.safe), "catastrophic": not bool(result.safe), "success": bool(result.safe and final_error <= .5 and final_speed <= .5),
        "position_rmse_m": float(np.sqrt(np.mean(error_eval**2))), "position_p90_m": float(np.percentile(error_eval,90)), "position_p95_m": float(np.percentile(error_eval,95)),
        "velocity_rmse_m_s": float(np.sqrt(np.mean(velocity_eval**2))), "orientation_rmse_rad": float(np.sqrt(np.mean(orient[mask]**2))),
        "endpoint_position_error_m": final_error, "endpoint_velocity_error_m_s": final_speed, "acquisition_time_s": None if not np.isfinite(acquired) else acquired,
        "physical_effort": float(np.mean(np.sum(commands**2,axis=1))), "thrust_saturation_rate": float(np.mean(thrust_sat)), "torque_saturation_rate": float(np.mean(torque_sat)), "wrench_rate_rms": float(np.sqrt(np.mean(rates**2))),
        "runtime_mean_ms": float(np.mean(all_ms)), "runtime_p95_ms": float(np.percentile(all_ms,95)), "runtime_p99_ms": float(np.percentile(all_ms,99)), "runtime_max_ms": float(np.max(all_ms)), "deadline_miss_rate": float(np.mean(all_ms>deadlines)),
        "execution_authority": result.execution_authority, "semantic_fingerprint": result.case_semantic_fingerprint}


def aggregate(rows: list[dict[str, Any]]) -> dict[str, Any]:
    def mean(key: str, subset: list[dict[str, Any]]=rows) -> float: return float(np.mean([r[key] for r in subset]))
    setpoint = [r for r in rows if r["task_family"] == "setpoint"]; trajectory = [r for r in rows if r["task_family"] == "smooth_trajectory"]
    strong = [r for r in rows if r["wind_kind"] in {"strong_sustained","strong_transient"}]
    acquisitions = [r["acquisition_time_s"] for r in setpoint if r["acquisition_time_s"] is not None]
    return {"method_id": rows[0]["method_id"], "family": rows[0]["family"], "case_count": len(rows),
        "safety_rate": mean("safe"), "success_rate": mean("success"), "catastrophic_count": int(sum(r["catastrophic"] for r in rows)),
        "setpoint_position_rmse_m": mean("position_rmse_m",setpoint), "setpoint_position_p90_m": float(np.percentile([r["position_rmse_m"] for r in setpoint],90)), "setpoint_position_p95_m": float(np.percentile([r["position_rmse_m"] for r in setpoint],95)),
        "trajectory_position_rmse_m": mean("position_rmse_m",trajectory), "trajectory_position_p90_m": float(np.percentile([r["position_rmse_m"] for r in trajectory],90)), "trajectory_position_p95_m": float(np.percentile([r["position_rmse_m"] for r in trajectory],95)),
        "orientation_rmse_rad": mean("orientation_rmse_rad"), "tip_velocity_rmse_m_s": mean("velocity_rmse_m_s",trajectory), "endpoint_position_error_m": mean("endpoint_position_error_m",trajectory), "endpoint_velocity_error_m_s": mean("endpoint_velocity_error_m_s",trajectory),
        "acquisition_time_s": float(np.median(acquisitions)) if acquisitions else None, "strong_mean_m": mean("position_rmse_m",strong), "strong_p90_m": float(np.percentile([r["position_rmse_m"] for r in strong],90)),
        "physical_effort": mean("physical_effort"), "thrust_saturation_rate": mean("thrust_saturation_rate"), "torque_saturation_rate": mean("torque_saturation_rate"), "wrench_rate_rms": mean("wrench_rate_rms"),
        "runtime_mean_ms": mean("runtime_mean_ms"), "runtime_p95_ms": float(np.percentile([r["runtime_p95_ms"] for r in rows],95)), "runtime_p99_ms": float(np.percentile([r["runtime_p99_ms"] for r in rows],99)), "runtime_max_ms": max(r["runtime_max_ms"] for r in rows), "deadline_miss_rate": mean("deadline_miss_rate")}
