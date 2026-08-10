"""Authoritative Native-Stack v1.1 execution entry point."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind_world
from uav_sway.models.model_config import load_model_config

from ..controller import NativeStackController
from ..runner import NativeRunResult, NativeStackRunner
from .resolver import NativeCaseResolver, ResolvedNativeCase, ResolvedReference

AUTHORITATIVE_EXECUTION = "NATIVE_STACK_BENCHMARK_V1_1_AUTHORITATIVE"


class AuthoritativeNativeCaseRunner:
    """Execute only a frozen resolved case; caller-defined semantics are absent."""

    execution_authority = AUTHORITATIVE_EXECUTION

    def __init__(
        self,
        model_path: str | Path,
        model_config_path: str | Path,
        aerodynamic_config_path: str | Path,
        outer_rate_hz: int,
        inner_rate_hz: int,
    ) -> None:
        self.model_path = Path(model_path)
        self.model_config = load_model_config(model_config_path)
        self.aerodynamic_config = load_aerodynamic_config(aerodynamic_config_path)
        self.outer_rate_hz = int(outer_rate_hz)
        self.inner_rate_hz = int(inner_rate_hz)
        self._resolver = NativeCaseResolver()
        self._generic = NativeStackRunner(self.model_path)

    def _frozen_inputs(self, resolved_case: ResolvedNativeCase):
        if not self._resolver.verify_fingerprint(resolved_case):
            raise ValueError("resolved case semantic fingerprint mismatch")
        reference = ResolvedReference(resolved_case)
        wind_timeline = self._resolver.canonical_signals(resolved_case)["wind_world"]

        def disturbance(model, data, tick: int, time_s: float) -> None:
            del time_s
            index = min(int(tick), len(wind_timeline) - 1)
            clear_and_apply_wind_world(
                model, data, self.model_config, self.aerodynamic_config,
                wind_timeline[index],
            )

        return reference, disturbance

    def run_case(
        self, controller: NativeStackController, resolved_case: ResolvedNativeCase,
    ) -> NativeRunResult:
        if resolved_case.split == "holdout" or not bool(resolved_case.execution["execution_allowed"]):
            raise PermissionError("Native Holdout execution is frozen and not authorized")
        reference, disturbance = self._frozen_inputs(resolved_case)
        result = self._generic.run(
            controller=controller,
            reference=reference,
            duration_s=resolved_case.duration_s,
            outer_rate_hz=self.outer_rate_hz,
            inner_rate_hz=self.inner_rate_hz,
            disturbance_callback=disturbance,
        )
        return replace(
            result,
            execution_authority=AUTHORITATIVE_EXECUTION,
            sample_id=resolved_case.sample_id,
            case_semantics_version=resolved_case.resolver_version,
            case_semantic_fingerprint=resolved_case.case_semantic_fingerprint,
        )
