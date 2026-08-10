"""Pure deterministic resolver for Native-Stack Benchmark v1.1.

The mapping in this module is a benchmark definition.  It must never inspect a
controller, a controller result, or split-specific performance data.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np

from ..api import ReferenceSample

NATIVE_CASE_SEMANTICS_VERSION = "native-case-semantics-v1"
RESOLVER_RNG = "numpy.PCG64"
PHYSICS_DT_S = 0.001
WIND_DT_S = 0.005
INITIAL_UAV_QPOS = (0.0, 0.0, 3.2, 1.0, 0.0, 0.0, 0.0)
INITIAL_CUTTER_TARGET_WORLD_M = (0.225, 0.0, 0.39)
APPLICATION_BODIES = ("quadrotor", "link_1", "link_2", "link_3", "link_4", "link_5", "cutter")


def _canonical_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("ascii")


def _hash_json(value: Any) -> str:
    return hashlib.sha256(_canonical_json(value)).hexdigest()


def _hash_signal(value: np.ndarray) -> str:
    array = np.ascontiguousarray(np.asarray(value, dtype="<f8"))
    return hashlib.sha256(array.tobytes(order="C")).hexdigest()


def _unit_horizontal_from_seed(seed: int) -> np.ndarray:
    rng = np.random.Generator(np.random.PCG64(int(seed)))
    angle = float(rng.uniform(-np.pi, np.pi))
    return np.array([np.cos(angle), np.sin(angle), 0.0], dtype=float)


def _direction_vector(label: str, displacement: np.ndarray, seed: int) -> np.ndarray:
    horizontal = np.array([displacement[0], displacement[1], 0.0], dtype=float)
    norm = float(np.linalg.norm(horizontal))
    base = horizontal / norm if norm > 1e-12 else _unit_horizontal_from_seed(seed)
    if label == "aligned":
        result = base
    elif label == "opposed":
        result = -base
    elif label == "cross":
        sign = 1.0 if int(seed) % 2 == 0 else -1.0
        result = sign * np.array([-base[1], base[0], 0.0], dtype=float)
    else:
        raise ValueError(f"unsupported wind_direction: {label}")
    return result


def _target_from_seed(seed: int) -> np.ndarray:
    """Resolve a target in the frozen 2 m spherical mission envelope."""
    rng = np.random.Generator(np.random.PCG64(int(seed)))
    azimuth = float(rng.uniform(-np.pi, np.pi))
    radius_xy = float(rng.uniform(0.55, 1.75))
    delta_z = float(rng.uniform(-0.10, 0.55))
    delta = np.array([radius_xy * np.cos(azimuth), radius_xy * np.sin(azimuth), delta_z])
    if float(np.linalg.norm(delta)) >= 2.0:
        raise AssertionError("target generator exceeded frozen mission envelope")
    return np.asarray(INITIAL_CUTTER_TARGET_WORLD_M, dtype=float) + delta


def _smooth7(u: np.ndarray, duration: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """C3 endpoint-rest interpolation used for stop/waypoint trajectories."""
    u = np.clip(np.asarray(u, dtype=float), 0.0, 1.0)
    p = 35*u**4 - 84*u**5 + 70*u**6 - 20*u**7
    v = (140*u**3 - 420*u**4 + 420*u**5 - 140*u**6) / duration
    a = (420*u**2 - 1680*u**3 + 2100*u**4 - 840*u**5) / duration**2
    j = (840*u - 5040*u**2 + 8400*u**3 - 4200*u**4) / duration**3
    return p, v, a, j


def _minimum_jerk(u: np.ndarray, duration: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Classical quintic minimum-jerk polynomial with analytic derivatives."""
    u = np.clip(np.asarray(u, dtype=float), 0.0, 1.0)
    p = 10*u**3 - 15*u**4 + 6*u**5
    v = (30*u**2 - 60*u**3 + 30*u**4) / duration
    a = (60*u - 180*u**2 + 120*u**3) / duration**2
    j = (60 - 360*u + 360*u**2) / duration**3
    return p, v, a, j


def _segment_signal(
    times: np.ndarray, start: np.ndarray, goal: np.ndarray, start_s: float,
    end_s: float, profile: str,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    duration = float(end_s - start_s)
    if duration <= 0.0:
        raise ValueError("trajectory segment duration must be positive")
    u = (times - start_s) / duration
    scalar = _minimum_jerk(u, duration) if profile == "minimum_jerk" else _smooth7(u, duration)
    delta = goal - start
    position = start + scalar[0][:, None] * delta
    velocity = scalar[1][:, None] * delta
    acceleration = scalar[2][:, None] * delta
    jerk = scalar[3][:, None] * delta
    before = times < start_s
    after = times > end_s
    position[before] = start; position[after] = goal
    velocity[before | after] = 0.0
    acceleration[before | after] = 0.0
    jerk[before | after] = 0.0
    return position, velocity, acceleration, jerk


@dataclass(frozen=True, slots=True)
class ResolvedNativeCase:
    sample_id: str
    split: str
    duration_s: float
    issue_offset_s: float
    identity: Mapping[str, Any]
    initial_condition: Mapping[str, Any]
    target: Mapping[str, Any]
    trajectory: Mapping[str, Any]
    wind: Mapping[str, Any]
    wind_application: Mapping[str, Any]
    execution: Mapping[str, Any]
    signal_hashes: Mapping[str, str]
    case_semantic_fingerprint: str
    resolver_version: str = NATIVE_CASE_SEMANTICS_VERSION

    def to_dict(self) -> dict[str, Any]:
        return {
            "sample_id": self.sample_id,
            "split": self.split,
            "duration_s": self.duration_s,
            "issue_offset_s": self.issue_offset_s,
            "resolver_version": self.resolver_version,
            "identity": dict(self.identity),
            "initial_condition": dict(self.initial_condition),
            "target": dict(self.target),
            "trajectory": dict(self.trajectory),
            "wind": dict(self.wind),
            "wind_application": dict(self.wind_application),
            "execution": dict(self.execution),
            "signal_hashes": dict(self.signal_hashes),
            "case_semantic_fingerprint": self.case_semantic_fingerprint,
        }


class NativeCaseResolver:
    """One versioned mapping from an identity manifest record to physics."""

    version = NATIVE_CASE_SEMANTICS_VERSION

    def verify_fingerprint(self, case: ResolvedNativeCase) -> bool:
        """Detect any serialized semantic or canonical-signal drift."""
        expected = self.resolve(case.identity)
        return expected.to_dict() == case.to_dict()

    def resolve(self, record: Mapping[str, Any]) -> ResolvedNativeCase:
        required = {
            "sample_id", "split", "duration_s", "issue_offset_s", "target_id",
            "target_seed", "trajectory_seed", "task_family", "trajectory_type",
            "wind_kind", "wind_seed", "wind_direction", "timing_id",
            "reference_preview_allowed", "execution_allowed",
        }
        missing = sorted(required - set(record))
        if missing:
            raise KeyError(f"case record missing fields: {missing}")
        split = str(record["split"])
        if split not in {"development", "holdout"}:
            raise ValueError(f"unsupported split: {split}")
        duration_s = float(record["duration_s"])
        issue_s = float(record["issue_offset_s"])
        if duration_s <= 2.0 or not 0.0 <= issue_s < duration_s:
            raise ValueError("invalid case timing")

        start = np.asarray(INITIAL_CUTTER_TARGET_WORLD_M, dtype=float)
        goal = _target_from_seed(int(record["target_seed"]))
        trajectory = self._trajectory(record, start, goal, duration_s, issue_s)
        direction = _direction_vector(str(record["wind_direction"]), goal - start, int(record["wind_seed"]))
        wind = self._wind(record, direction, duration_s)
        application = {
            "affected_bodies": list(APPLICATION_BODIES),
            "application_point": "body_center_of_mass",
            "air_density_kg_m3": 1.225,
            "force_formula": "0.5*rho*Cd*A_projected*abs(v_rel_axis)*v_rel_axis*axis_world",
            "area_and_cd_source": "configs/aerodynamics.yaml",
            "area_model": {"quadrotor": "box", "link_1..link_5": "capsule", "cutter": "box"},
            "distribution_rule": "independent_per_body",
            "aerodynamic_torque": False,
            "implementation": "uav_sway.disturbances.aerodynamics.compute_body_wind_forces_world",
        }
        execution = {
            "physics_dt_s": PHYSICS_DT_S,
            "physics_rate_hz": 1000,
            "reference_sampling": "physics_tick_at_start_of_interval",
            "wind_sampling": "200_Hz_left_sample_then_zero_order_hold_at_1000_Hz",
            "reference_interpolation": "analytic",
            "reference_preview_allowed": bool(record["reference_preview_allowed"]),
            "execution_allowed": bool(record["execution_allowed"]),
            "units": "SI",
            "position_frame": "world",
            "wind_frame": "world",
        }
        initial = {
            "uav_qpos": list(INITIAL_UAV_QPOS),
            "uav_qvel": [0.0] * 6,
            "joint_positions_rad": [0.0] * 5,
            "joint_velocities_rad_s": [0.0] * 5,
            "cutter_configuration": "horizontal_positive_world_x_at_trim",
            "initial_cutter_target_world_m": start.tolist(),
        }
        target = {
            "initial_cutter_target_world_m": start.tolist(),
            "final_cutter_target_world_m": goal.tolist(),
            "displacement_world_m": (goal - start).tolist(),
            "generation_rule": "pcg64_uniform_azimuth_xy_radius_vertical_v1",
            "generation_version": self.version,
            "mission_envelope_radius_m": 2.0,
        }
        identity = {key: record[key] for key in (
            "sample_id", "split", "target_id", "target_seed", "trajectory_seed",
            "task_family", "trajectory_type", "wind_kind", "wind_seed",
            "wind_direction", "timing_id", "issue_offset_s", "duration_s",
            "reference_preview_allowed", "execution_allowed",
        )}

        provisional = ResolvedNativeCase(
            str(record["sample_id"]), split, duration_s, issue_s, identity, initial,
            target, trajectory, wind, application, execution, {}, "",
        )
        signals = self.canonical_signals(provisional)
        hashes = {
            "reference_position_sha256": _hash_signal(signals["position"]),
            "reference_velocity_sha256": _hash_signal(signals["velocity"]),
            "reference_acceleration_sha256": _hash_signal(signals["acceleration"]),
            "reference_jerk_sha256": _hash_signal(signals["jerk"]),
            "wind_world_sha256": _hash_signal(signals["wind_world"]),
            "wind_body_application_sha256": _hash_json({
                "wind_world_sha256": _hash_signal(signals["wind_world"]),
                "application": application,
            }),
        }
        fingerprint_payload = provisional.to_dict()
        fingerprint_payload.pop("signal_hashes")
        fingerprint_payload.pop("case_semantic_fingerprint")
        fingerprint_payload["signal_hashes"] = hashes
        fingerprint = _hash_json(fingerprint_payload)
        return ResolvedNativeCase(
            provisional.sample_id, split, duration_s, issue_s, identity, initial,
            target, trajectory, wind, application, execution, hashes, fingerprint,
        )

    def _trajectory(
        self, record: Mapping[str, Any], start: np.ndarray, goal: np.ndarray,
        duration_s: float, issue_s: float,
    ) -> dict[str, Any]:
        kind = str(record["trajectory_type"])
        if kind == "step":
            return {"type": kind, "start_time_s": issue_s, "end_time_s": issue_s, "waypoints_world_m": [start.tolist(), goal.tolist()], "waypoint_times_s": [0.0, issue_s], "interpolation": "causal_step"}
        end_s = duration_s - (1.0 if kind in {"approach_stop", "waypoint_3d"} else 0.5)
        if kind == "minimum_jerk":
            return {"type": kind, "start_time_s": issue_s, "end_time_s": end_s, "waypoints_world_m": [start.tolist(), goal.tolist()], "waypoint_times_s": [issue_s, end_s], "interpolation": "quintic_minimum_jerk"}
        if kind == "approach_stop":
            return {"type": kind, "start_time_s": issue_s, "end_time_s": end_s, "waypoints_world_m": [start.tolist(), goal.tolist()], "waypoint_times_s": [issue_s, end_s], "interpolation": "septic_C3_endpoint_rest"}
        if kind == "waypoint_3d":
            rng = np.random.Generator(np.random.PCG64(int(record["trajectory_seed"])))
            delta = goal - start
            horizontal = np.array([delta[0], delta[1], 0.0])
            norm = float(np.linalg.norm(horizontal))
            lateral = np.array([-horizontal[1], horizontal[0], 0.0]) / norm if norm > 1e-12 else _unit_horizontal_from_seed(int(record["trajectory_seed"]))
            sign = 1.0 if float(rng.uniform()) >= 0.5 else -1.0
            offset = sign * float(rng.uniform(0.08, 0.18)) * lateral
            offset[2] = float(rng.uniform(0.08, 0.18))
            waypoints = [start, start + 0.34*delta + offset, start + 0.68*delta - 0.5*offset, goal]
            times = np.linspace(issue_s, end_s, len(waypoints)).tolist()
            return {"type": kind, "start_time_s": issue_s, "end_time_s": end_s, "waypoints_world_m": [x.tolist() for x in waypoints], "waypoint_times_s": times, "interpolation": "piecewise_septic_C3_endpoint_rest", "trajectory_seed": int(record["trajectory_seed"])}
        raise ValueError(f"unsupported trajectory_type: {kind}")

    def _wind(self, record: Mapping[str, Any], direction: np.ndarray, duration_s: float) -> dict[str, Any]:
        kind = str(record["wind_kind"])
        common = {"kind": kind, "direction_class": str(record["wind_direction"]), "direction_world": direction.tolist(), "seed": int(record["wind_seed"]), "sample_dt_s": WIND_DT_S, "hold": "zero_order_hold", "duration_s": duration_s}
        specs = {
            "calm": {"magnitude_m_s": 0.0, "onset_s": 0.0, "active_duration_s": duration_s, "waveform": "zero"},
            "moderate": {"magnitude_m_s": 1.5, "onset_s": 4.0, "active_duration_s": duration_s-4.0, "waveform": "constant"},
            "strong_sustained": {"magnitude_m_s": 3.0, "onset_s": 4.0, "active_duration_s": duration_s-4.0, "waveform": "constant"},
            "strong_transient": {"magnitude_m_s": 3.0, "onset_s": 5.0, "active_duration_s": 2.0, "waveform": "one_cosine"},
            "stochastic": {"magnitude_m_s": 3.0, "onset_s": 4.0, "active_duration_s": duration_s-4.0, "waveform": "low_pass_gaussian", "rng": RESOLVER_RNG, "mean_m_s": 0.0, "sigma_m_s": 0.8, "time_constant_s": 1.0, "clip_abs_m_s": 3.0, "initial_value_m_s": 0.0},
            "ramp": {"magnitude_m_s": 3.0, "onset_s": 4.0, "active_duration_s": duration_s-4.0, "ramp_duration_s": 3.0, "waveform": "linear_ramp_then_hold"},
        }
        if kind not in specs:
            raise ValueError(f"unsupported wind_kind: {kind}")
        return {**common, **specs[kind]}

    def canonical_signals(self, case: ResolvedNativeCase) -> dict[str, np.ndarray]:
        count = int(round(case.duration_s / PHYSICS_DT_S))
        times = np.arange(count, dtype=float) * PHYSICS_DT_S
        position, velocity, acceleration, jerk = self._reference_timeline(case, times)
        wind_world = self._wind_timeline(case, times)
        return {"time": times, "position": position, "velocity": velocity, "acceleration": acceleration, "jerk": jerk, "wind_world": wind_world}

    def _reference_timeline(self, case: ResolvedNativeCase, times: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        trajectory = case.trajectory
        points = np.asarray(trajectory["waypoints_world_m"], dtype=float)
        kind = str(trajectory["type"])
        if kind == "step":
            position = np.repeat(points[0][None, :], len(times), axis=0)
            position[times >= float(trajectory["start_time_s"])] = points[-1]
            zero = np.zeros_like(position)
            return position, zero.copy(), zero.copy(), zero.copy()
        waypoint_times = np.asarray(trajectory["waypoint_times_s"], dtype=float)
        profile = "minimum_jerk" if kind == "minimum_jerk" else "smooth7"
        if len(points) == 2:
            return _segment_signal(times, points[0], points[1], waypoint_times[0], waypoint_times[1], profile)
        position = np.repeat(points[0][None, :], len(times), axis=0)
        velocity = np.zeros_like(position); acceleration = np.zeros_like(position); jerk = np.zeros_like(position)
        for index in range(len(points)-1):
            mask = (times >= waypoint_times[index]) & (times <= waypoint_times[index+1])
            values = _segment_signal(times[mask], points[index], points[index+1], waypoint_times[index], waypoint_times[index+1], profile)
            position[mask], velocity[mask], acceleration[mask], jerk[mask] = values
        position[times > waypoint_times[-1]] = points[-1]
        return position, velocity, acceleration, jerk

    def _wind_timeline(self, case: ResolvedNativeCase, physics_times: np.ndarray) -> np.ndarray:
        wind = case.wind
        sample_count = int(round(case.duration_s / WIND_DT_S)) + 1
        sample_times = np.arange(sample_count, dtype=float) * WIND_DT_S
        magnitude = np.zeros(sample_count, dtype=float)
        onset = float(wind["onset_s"]); kind = str(wind["kind"])
        active = sample_times >= onset
        if kind in {"moderate", "strong_sustained"}:
            magnitude[active] = float(wind["magnitude_m_s"])
        elif kind == "strong_transient":
            duration = float(wind["active_duration_s"])
            mask = active & (sample_times <= onset + duration)
            u = (sample_times[mask] - onset) / duration
            magnitude[mask] = float(wind["magnitude_m_s"]) * 0.5 * (1.0 - np.cos(2.0*np.pi*u))
        elif kind == "stochastic":
            rng = np.random.Generator(np.random.PCG64(int(wind["seed"])))
            tau = float(wind["time_constant_s"]); sigma = float(wind["sigma_m_s"])
            alpha = float(np.exp(-WIND_DT_S / tau)); previous = float(wind["initial_value_m_s"])
            for index in range(1, sample_count):
                if sample_times[index] < onset:
                    continue
                previous = alpha*previous + np.sqrt(1.0-alpha*alpha)*sigma*float(rng.standard_normal())
                magnitude[index] = previous
            magnitude = np.clip(magnitude, -float(wind["clip_abs_m_s"]), float(wind["clip_abs_m_s"]))
        elif kind == "ramp":
            ramp_duration = float(wind["ramp_duration_s"])
            magnitude[active] = float(wind["magnitude_m_s"]) * np.clip((sample_times[active]-onset)/ramp_duration, 0.0, 1.0)
        elif kind != "calm":
            raise ValueError(kind)
        indices = np.minimum((physics_times / WIND_DT_S).astype(int), sample_count-1)
        return magnitude[indices, None] * np.asarray(wind["direction_world"], dtype=float)[None, :]


class ResolvedReference:
    """Causal reference generated only from a frozen resolved case."""

    def __init__(self, case: ResolvedNativeCase) -> None:
        self.case = case
        self._resolver = NativeCaseResolver()

    def sample(self, time_s: float) -> ReferenceSample:
        times = np.asarray([float(time_s)], dtype=float)
        position, velocity, acceleration, jerk = self._resolver._reference_timeline(self.case, times)
        return ReferenceSample(position[0], velocity[0], acceleration[0], jerk[0], float(time_s))

    def wind_world_at(self, time_s: float) -> np.ndarray:
        return self._resolver._wind_timeline(self.case, np.asarray([float(time_s)], dtype=float))[0]
