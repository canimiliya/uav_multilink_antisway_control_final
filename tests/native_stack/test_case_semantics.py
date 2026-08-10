from __future__ import annotations

import json
import os
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import mujoco
import numpy as np

from uav_sway.disturbances.aerodynamics import load_aerodynamic_config
from uav_sway.disturbances.wind_applier import clear_and_apply_wind, clear_and_apply_wind_world
from uav_sway.models.model_config import load_model_config
from uav_sway.native_stack.case_semantics import NativeCaseResolver, ResolvedReference

ROOT = Path(__file__).resolve().parents[2]
R0 = ROOT / "reproducibility/native_stack/r0"
MODEL = ROOT / "reproducibility/frozen/model/model_5link_controlled.xml"


def manifest(split: str) -> dict:
    return json.loads((R0 / f"native_{split}_manifest.json").read_text(encoding="utf-8"))


def case_for(**fields):
    cases = manifest("development")["cases"]
    return next(case for case in cases if all(case[key] == value for key, value in fields.items()))


def test_same_case_and_cross_process_determinism() -> None:
    record = case_for(wind_kind="stochastic")
    resolver = NativeCaseResolver()
    first = resolver.resolve(record)
    assert first.to_dict() == resolver.resolve(record).to_dict()
    code = (
        "import json; from pathlib import Path; "
        "from uav_sway.native_stack.case_semantics import NativeCaseResolver; "
        f"r=json.loads(Path({str(R0 / 'native_development_manifest.json')!r}).read_text(encoding='utf-8'))['cases'][{manifest('development')['cases'].index(record)}]; "
        "print(NativeCaseResolver().resolve(r).case_semantic_fingerprint)"
    )
    env = dict(os.environ)
    env["PYTHONPATH"] = os.pathsep.join([str(ROOT / "src"), str(ROOT / "third_party/udaan")])
    observed = subprocess.check_output([sys.executable, "-c", code], cwd=ROOT, env=env, text=True).strip()
    assert observed == first.case_semantic_fingerprint


def test_target_mapping_envelope_and_split_independent_algorithm() -> None:
    resolver = NativeCaseResolver()
    for record in manifest("development")["cases"][:20]:
        resolved = resolver.resolve(record)
        displacement = np.asarray(resolved.target["displacement_world_m"])
        assert np.linalg.norm(displacement) < 2.0
        assert np.asarray(resolved.target["final_cutter_target_world_m"])[2] > 0.0
    dev = manifest("development")["cases"][7]
    synthetic_holdout = {**dev, "sample_id": "synthetic_holdout", "split": "holdout", "execution_allowed": False}
    np.testing.assert_array_equal(
        resolver.resolve(dev).target["final_cutter_target_world_m"],
        resolver.resolve(synthetic_holdout).target["final_cutter_target_world_m"],
    )


def test_direction_mapping_is_geometric_and_handles_degenerate_displacement() -> None:
    resolver = NativeCaseResolver()
    records = [case_for(wind_kind="moderate", wind_direction=label) for label in ("aligned", "opposed", "cross")]
    for record in records:
        resolved = resolver.resolve(record)
        direction = np.asarray(resolved.wind["direction_world"])
        displacement = np.asarray(resolved.target["displacement_world_m"]); horizontal = displacement.copy(); horizontal[2] = 0
        dot = float(direction @ horizontal / np.linalg.norm(horizontal))
        if record["wind_direction"] == "aligned": assert np.isclose(dot, 1.0)
        elif record["wind_direction"] == "opposed": assert np.isclose(dot, -1.0)
        else: assert abs(dot) < 1e-12
        assert np.isclose(np.linalg.norm(direction), 1.0)


def test_trajectory_endpoints_and_required_continuity() -> None:
    resolver = NativeCaseResolver()
    for kind in ("step", "minimum_jerk", "approach_stop", "waypoint_3d"):
        resolved = resolver.resolve(case_for(trajectory_type=kind))
        reference = ResolvedReference(resolved)
        start = np.asarray(resolved.target["initial_cutter_target_world_m"])
        goal = np.asarray(resolved.target["final_cutter_target_world_m"])
        np.testing.assert_allclose(reference.sample(0.0).position_world, start)
        np.testing.assert_allclose(reference.sample(resolved.duration_s).position_world, goal)
        if kind != "step":
            end = float(resolved.trajectory["end_time_s"])
            np.testing.assert_allclose(reference.sample(end).velocity_world, 0.0, atol=1e-12)
            np.testing.assert_allclose(reference.sample(end).acceleration_world, 0.0, atol=1e-12)
        if kind in {"approach_stop", "waypoint_3d"}:
            for time_s in resolved.trajectory["waypoint_times_s"]:
                sample = reference.sample(float(time_s))
                np.testing.assert_allclose(sample.velocity_world, 0.0, atol=1e-10)
                np.testing.assert_allclose(sample.acceleration_world, 0.0, atol=1e-10)
                np.testing.assert_allclose(sample.jerk_world, 0.0, atol=1e-10)


def test_wind_categories_stochastic_reproducibility_and_zoh() -> None:
    resolver = NativeCaseResolver()
    peaks = {}
    for kind in ("calm", "moderate", "strong_sustained", "strong_transient", "stochastic", "ramp"):
        resolved = resolver.resolve(case_for(wind_kind=kind))
        wind = resolver.canonical_signals(resolved)["wind_world"]
        assert np.isfinite(wind).all()
        peaks[kind] = float(np.max(np.linalg.norm(wind, axis=1)))
        np.testing.assert_array_equal(wind, resolver.canonical_signals(resolved)["wind_world"])
        assert np.array_equal(wind[4000], wind[4004])
    assert peaks["calm"] == 0.0
    assert peaks["moderate"] == 1.5
    assert peaks["strong_sustained"] == 3.0
    assert peaks["strong_transient"] == 3.0
    assert peaks["stochastic"] <= 3.0
    assert np.isclose(peaks["ramp"], 3.0, rtol=0.0, atol=1e-12)


def test_world_wind_extension_preserves_frozen_x_axis_physics() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    model_config = load_model_config(ROOT / "configs/model_5link.yaml")
    aero = load_aerodynamic_config(ROOT / "configs/aerodynamics.yaml")
    old_data, new_data = mujoco.MjData(model), mujoco.MjData(model)
    for data in (old_data, new_data):
        data.qpos[:] = 0.0; data.qpos[:7] = [0, 0, 3.2, 1, 0, 0, 0]
        mujoco.mj_forward(model, data)
    old = clear_and_apply_wind(model, old_data, model_config, aero, 1.5)
    new = clear_and_apply_wind_world(model, new_data, model_config, aero, np.array([1.5, 0.0, 0.0]))
    np.testing.assert_array_equal(old_data.xfrc_applied, new_data.xfrc_applied)
    assert np.isclose(old["total_x"], np.asarray(new["total_world"])[0])
    assert set(new) == {"quadrotor", "link_1", "link_2", "link_3", "link_4", "link_5", "cutter", "total_world"}


def test_fingerprint_detects_semantic_mutation() -> None:
    resolver = NativeCaseResolver()
    resolved = resolver.resolve(manifest("development")["cases"][0])
    assert resolver.verify_fingerprint(resolved)
    mutated_target = {**resolved.target, "final_cutter_target_world_m": [9.0, 9.0, 9.0]}
    assert not resolver.verify_fingerprint(replace(resolved, target=mutated_target))
