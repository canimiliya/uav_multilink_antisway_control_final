"""Frozen S6T2 cutter-setpoint protocol and causal references."""

from __future__ import annotations

import csv
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .reference import EquilibriumTaskPose, build_equilibrium_task_pose


@dataclass(frozen=True)
class SetpointScene:
    name: str
    task_start_time_s: float | None
    wind_profile: str
    source_scenario: str
    performance_execution: bool = True


@dataclass(frozen=True)
class SetpointProtocol:
    duration_s: float
    sample_dt_s: float
    target_delta_x_m: float
    initial_tip_position_m: np.ndarray
    target_tip_position_m: np.ndarray
    target_cutter_axis: np.ndarray
    equilibrium: EquilibriumTaskPose
    scenes: tuple[SetpointScene, ...]

    def as_dict(self) -> dict:
        return {
            "protocol": "S6T2-cutter-setpoint-protocol-r1",
            "duration_s": self.duration_s,
            "sample_dt_s": self.sample_dt_s,
            "target_delta_x_m": self.target_delta_x_m,
            "initial_tip_position_m": self.initial_tip_position_m.tolist(),
            "target_tip_position_m": self.target_tip_position_m.tolist(),
            "target_cutter_axis": self.target_cutter_axis.tolist(),
            "equilibrium": self.equilibrium.as_dict(),
            "scenes": [scene.__dict__ for scene in self.scenes],
            "setpoint_is_constant_after_issue": True,
            "reference_preview_before_issue_forbidden": True,
        }


def _sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_setpoint_protocol(model, data, model_path: str | Path, config: dict) -> SetpointProtocol:
    equilibrium = build_equilibrium_task_pose(model, data, model_path)
    initial = np.asarray(data.site_xpos[int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cutter_tip"))], dtype=float)
    delta = np.array([float(config["target_delta_x_m"]), 0.0, 0.0], dtype=float)
    scenes = tuple(
        SetpointScene(
            name=name,
            task_start_time_s=None if values.get("task_start_time_s") is None else float(values["task_start_time_s"]),
            wind_profile=str(values["wind_profile"]),
            source_scenario=str(values["source_scenario"]),
            performance_execution=bool(values.get("performance_execution", True)),
        )
        for name, values in config["scenes"].items()
    )
    return SetpointProtocol(
        duration_s=float(config["duration_s"]),
        sample_dt_s=float(config["sample_dt_s"]),
        target_delta_x_m=float(config["target_delta_x_m"]),
        initial_tip_position_m=initial,
        target_tip_position_m=initial + delta,
        target_cutter_axis=np.asarray(equilibrium.cutter_axis_world, dtype=float),
        equilibrium=equilibrium,
        scenes=scenes,
    )


def write_setpoint_reference(path: str | Path, protocol: SetpointProtocol, scene: SetpointScene) -> None:
    if scene.task_start_time_s is None:
        raise ValueError("performance reference requires a task start time")
    time = np.arange(0.0, protocol.duration_s + 0.5 * protocol.sample_dt_s, protocol.sample_dt_s)
    issued = time >= scene.task_start_time_s
    x = np.where(issued, protocol.target_delta_x_m, 0.0)
    event = np.where(issued, "setpoint_issued", "hover")
    target = np.asarray(protocol.target_tip_position_m, dtype=float)
    y = np.full_like(time, 0.0)
    z = np.full_like(time, float(protocol.initial_tip_position_m[2] - protocol.equilibrium.tip_relative_position_m[2]))
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=["time", "x_ref", "vx_ref", "ax_ref", "y_ref", "z_ref", "yaw_ref", "event", "control_tick"], lineterminator="\n")
        writer.writeheader()
        for index, current_time in enumerate(time):
            writer.writerow({
                "time": format(float(current_time), ".17g"),
                "x_ref": format(float(x[index]), ".17g"), "vx_ref": "0", "ax_ref": "0",
                "y_ref": format(float(y[index]), ".17g"), "z_ref": format(float(z[index]), ".17g"), "yaw_ref": "0",
                "event": str(event[index]), "control_tick": index if index % 10 == 0 else -1,
            })


def protocol_reference_audit(path: str | Path, protocol: SetpointProtocol, scene: SetpointScene) -> dict:
    rows = list(csv.DictReader(Path(path).open("r", encoding="utf-8", newline="")))
    time = np.asarray([float(row["time"]) for row in rows])
    x = np.asarray([float(row["x_ref"]) for row in rows])
    vx = np.asarray([float(row["vx_ref"]) for row in rows])
    ax = np.asarray([float(row["ax_ref"]) for row in rows])
    before = time < float(scene.task_start_time_s)
    after = ~before
    checks = {
        "task_start_time_exact": bool(np.isclose(time[np.flatnonzero(after)[0]], scene.task_start_time_s)),
        "target_delta_x_exact": bool(np.all(x[before] == 0.0) and np.all(x[after] == protocol.target_delta_x_m)),
        "setpoint_constant_after_issue": bool(np.all(x[after] == protocol.target_delta_x_m)),
        "no_future_target_before_issue": bool(np.all(x[before] == 0.0)),
        "zero_velocity_reference": bool(np.all(vx == 0.0)),
        "zero_acceleration_reference": bool(np.all(ax == 0.0)),
        "target_tip_position_from_equilibrium": bool(np.allclose(protocol.target_tip_position_m, protocol.initial_tip_position_m + np.array([0.30, 0.0, 0.0]))),
    }
    return {"scene": scene.name, "reference_sha256": _sha256(path), "checks": checks, "pass": bool(all(checks.values()))}


def write_gust_protocol(path: str | Path, protocol: SetpointProtocol) -> None:
    gust = next(scene for scene in protocol.scenes if scene.name == "task_gust_recovery")
    payload = protocol.as_dict()
    payload["gust_recovery_protocol"] = {
        "scene": gust.name, "execution": False, "calm_hold_s": [0.0, 5.0],
        "frozen_gust_profile": gust.wind_profile, "recovery_window_s": [7.0, 12.0],
        "target_is_fixed_equilibrium_pose": True,
        "required_metrics": ["gust_peak_tip_position_error_m", "gust_peak_orientation_error_deg", "gust_recovery_time_s"],
    }
    Path(path).write_text(json.dumps(payload, indent=2, allow_nan=False) + "\n", encoding="utf-8", newline="\n")
