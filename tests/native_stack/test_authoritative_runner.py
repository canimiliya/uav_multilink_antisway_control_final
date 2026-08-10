from __future__ import annotations

import inspect
import json
from dataclasses import replace
from pathlib import Path

import pytest

from uav_sway.native_stack.case_semantics import AuthoritativeNativeCaseRunner, NativeCaseResolver
from uav_sway.native_stack.runner import NativeStackRunner

ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/native_stack/r0"


def records(split: str):
    return json.loads((R0 / f"native_{split}_manifest.json").read_text(encoding="utf-8"))["cases"]


def runner() -> AuthoritativeNativeCaseRunner:
    return AuthoritativeNativeCaseRunner(
        ROOT / "reproducibility/frozen/model/model_5link_controlled.xml",
        ROOT / "configs/model_5link.yaml",
        ROOT / "configs/aerodynamics.yaml",
        outer_rate_hz=50,
        inner_rate_hz=200,
    )


def test_authoritative_signature_has_no_custom_semantics() -> None:
    parameters = list(inspect.signature(AuthoritativeNativeCaseRunner.run_case).parameters)
    assert parameters == ["self", "controller", "resolved_case"]
    assert NativeStackRunner.execution_authority == "DIAGNOSTIC_NON_AUTHORITATIVE"


def test_authoritative_plumbing_builds_reference_and_distributed_wind_without_execution() -> None:
    record = next(case for case in records("development") if case["wind_kind"] == "moderate")
    resolved = NativeCaseResolver().resolve(record)
    reference, disturbance = runner()._frozen_inputs(resolved)
    assert reference.sample(0.0).time_s == 0.0
    assert callable(disturbance)


def test_holdout_execution_is_hard_rejected() -> None:
    resolved = NativeCaseResolver().resolve(records("holdout")[0])
    with pytest.raises(PermissionError, match="Holdout"):
        runner().run_case(object(), resolved)


def test_fingerprint_mutation_is_rejected_before_execution() -> None:
    resolved = NativeCaseResolver().resolve(records("development")[0])
    mutated = replace(resolved, case_semantic_fingerprint="0" * 64)
    with pytest.raises(ValueError, match="fingerprint"):
        runner()._frozen_inputs(mutated)
