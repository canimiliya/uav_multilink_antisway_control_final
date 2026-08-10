"""End-to-end runner for Native-Stack Benchmark diagnostic/Development use."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Protocol

import mujoco
import numpy as np

from .actuation import CanonicalWrenchActuator
from .api import ReferenceSample, WrenchCommand
from .controller import NativeStackController
from .logging import NativeCommandLogger
from .safety import SafetyV2Limits, evaluate_safety_v2
from .scheduler import DeterministicMultiRateScheduler
from .sensors import NativeSensorReader


class ReferenceGenerator(Protocol):
    def sample(self, time_s: float) -> ReferenceSample: ...


@dataclass(frozen=True, slots=True)
class NativeRunResult:
    ticks: int
    safe: bool
    safety_reasons: tuple[str, ...]
    command_records: tuple
    outer_update_ticks: tuple[int, ...]
    inner_update_ticks: tuple[int, ...]
    final_qpos: np.ndarray
    final_qvel: np.ndarray
    execution_authority: str = "DIAGNOSTIC_NON_AUTHORITATIVE"
    sample_id: str | None = None
    case_semantics_version: str | None = None
    case_semantic_fingerprint: str | None = None


class NativeStackRunner:
    """Runs the frozen plant without any split-specific knowledge.

    A caller may provide a disturbance callback for a manifest-approved wind
    realization. The callback receives MuJoCo objects but is never exposed to
    the controller or SensorPacket.
    """

    execution_authority = "DIAGNOSTIC_NON_AUTHORITATIVE"

    def __init__(self, model_path: str | Path) -> None:
        self.model_path = Path(model_path)

    def run(
        self, controller: NativeStackController, reference: ReferenceGenerator,
        duration_s: float, outer_rate_hz: int, inner_rate_hz: int,
        disturbance_callback: Callable[[object, object, int, float], None] | None = None,
        safety_limits: SafetyV2Limits = SafetyV2Limits(),
    ) -> NativeRunResult:
        model = mujoco.MjModel.from_xml_path(str(self.model_path))
        data = mujoco.MjData(model)
        data.qpos[:] = 0.0; data.qpos[:7] = [0, 0, 3.2, 1, 0, 0, 0]
        data.qvel[:] = 0.0; data.ctrl[:] = 0.0; data.eq_active[:] = 0
        mujoco.mj_forward(model, data)
        scheduler = DeterministicMultiRateScheduler(int(round(1.0 / model.opt.timestep)))
        scheduler.register("outer", int(outer_rate_hz)); scheduler.register("inner", int(inner_rate_hz))
        sensor_reader = NativeSensorReader(model); actuator = CanonicalWrenchActuator(model)
        logger = NativeCommandLogger(float(model.opt.timestep))
        controller.reset()
        previous = WrenchCommand(0.0, np.zeros(3))
        quad = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
        tip = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))
        origin_uav = np.asarray(data.xpos[quad]).copy(); origin_tip = np.asarray(data.site_xpos[tip]).copy()
        outer_ticks: list[int] = []; inner_ticks: list[int] = []; reasons: set[str] = set()
        ticks = int(round(float(duration_s) / float(model.opt.timestep)))
        for tick in range(ticks):
            time_s = scheduler.timestamp(tick)
            if disturbance_callback is not None:
                disturbance_callback(model, data, tick, time_s)
            due = scheduler.due_components(tick)
            packet = sensor_reader.read(model, data, reference.sample(time_s), previous, tick, float(model.opt.timestep))
            controller.observe(packet)
            if "outer" in due:
                controller.update_high_level(); outer_ticks.append(tick)
            if "inner" in due:
                controller.update_inner(); inner_ticks.append(tick)
            applied = actuator.apply(data, controller.physical_command(), tick, float(model.opt.timestep))
            logger.append(applied, due)
            safe, tick_reasons = evaluate_safety_v2(packet, applied, origin_uav, origin_tip, safety_limits)
            if not safe: reasons.update(tick_reasons)
            previous = applied.actual
            mujoco.mj_step(model, data)
        return NativeRunResult(
            ticks, not reasons, tuple(sorted(reasons)), tuple(logger.records), tuple(outer_ticks), tuple(inner_ticks),
            np.asarray(data.qpos).copy(), np.asarray(data.qvel).copy(),
        )
