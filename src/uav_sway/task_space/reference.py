"""Equilibrium-derived cutter task-space references."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from .state import CutterTaskSpaceReader


@dataclass(frozen=True)
class EquilibriumTaskPose:
    tip_relative_position_m: np.ndarray
    cutter_axis_world: np.ndarray
    cutter_rotation: np.ndarray
    axis_norm: float
    model_sha256: str

    def as_dict(self) -> dict:
        return {
            "tip_relative_position_m": np.asarray(self.tip_relative_position_m, dtype=float).tolist(),
            "cutter_axis_world": np.asarray(self.cutter_axis_world, dtype=float).tolist(),
            "cutter_rotation": np.asarray(self.cutter_rotation, dtype=float).tolist(),
            "axis_norm": float(self.axis_norm),
            "model_sha256": self.model_sha256,
        }


@dataclass(frozen=True)
class CutterTaskReference:
    tip_position_world: np.ndarray
    cutter_axis_world: np.ndarray

    def __post_init__(self) -> None:
        position = np.asarray(self.tip_position_world, dtype=float).reshape(3)
        axis = np.asarray(self.cutter_axis_world, dtype=float).reshape(3)
        norm = float(np.linalg.norm(axis))
        if norm <= 1.0e-12 or not np.isfinite(norm):
            raise ValueError("reference cutter axis must be finite and non-zero")
        object.__setattr__(self, "tip_position_world", position.copy())
        object.__setattr__(self, "cutter_axis_world", (axis / norm).copy())


def _sha256(path: str | Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def build_equilibrium_task_pose(model, data, model_path: str | Path) -> EquilibriumTaskPose:
    """Measure the frozen plant equilibrium without hand-entered geometry."""

    reader = CutterTaskSpaceReader(model)
    state = reader.read(model, data)
    quad_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quadrotor"))
    if quad_id < 0:
        raise KeyError("quadrotor")
    relative = state.tip_position_world - np.asarray(data.xpos[quad_id], dtype=float)
    axis_norm = float(np.linalg.norm(state.cutter_axis_world))
    return EquilibriumTaskPose(relative, state.cutter_axis_world, state.cutter_rotation_world, axis_norm, _sha256(model_path))


def task_reference_at(old_reference: dict[str, float], equilibrium: EquilibriumTaskPose) -> CutterTaskReference:
    """Map the old UAV reference to a task-space reference by translation only."""

    uav_reference = np.asarray([old_reference["x_ref"], old_reference["y_ref"], old_reference["z_ref"]], dtype=float)
    return CutterTaskReference(uav_reference + equilibrium.tip_relative_position_m, equilibrium.cutter_axis_world)


def write_equilibrium_pose(path: str | Path, pose: EquilibriumTaskPose) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(pose.as_dict(), indent=2) + "\n", encoding="utf-8", newline="\n")
