"""Reference-window handling for the fixed 200 Hz S2 signal contract."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from uav_sway.control.base import ReferenceState


@dataclass(frozen=True)
class ReferenceHorizon:
    """H reference actions backed by H+1 boundary reference samples.

    Boundary ``j`` is the reference at the beginning of action ``j``.  The
    state produced by that action is evaluated against boundary ``j + 1``.
    Keeping both semantics explicit prevents an outer-step phase error.
    """

    boundary_samples: tuple[ReferenceState, ...]
    boundary_indices: tuple[int, ...]
    boundary_times: np.ndarray

    @property
    def samples(self) -> tuple[ReferenceState, ...]:
        """Compatibility view of all H+1 boundary samples."""
        return self.boundary_samples

    @property
    def indices(self) -> tuple[int, ...]:
        """Compatibility view of all H+1 boundary indices."""
        return self.boundary_indices

    @property
    def times(self) -> np.ndarray:
        """Compatibility view of all H+1 boundary times."""
        return self.boundary_times

    @property
    def action_count(self) -> int:
        return len(self.boundary_samples) - 1

    def __len__(self) -> int:
        return self.action_count

    def __getitem__(self, index: int) -> ReferenceState:
        return self.action_reference(index)

    def action_reference(self, action_index: int) -> ReferenceState:
        if not 0 <= action_index < self.action_count:
            raise IndexError("action index outside reference horizon")
        return self.boundary_samples[action_index]

    def state_reference(self, action_index: int) -> ReferenceState:
        if not 0 <= action_index < self.action_count:
            raise IndexError("action index outside reference horizon")
        return self.boundary_samples[action_index + 1]


def make_reference_horizon(reference: dict[str, np.ndarray], signal_index: int,
                           horizon_steps: int) -> ReferenceHorizon:
    if horizon_steps <= 0:
        raise ValueError("horizon_steps must be positive")
    count = len(reference["time"])
    if count == 0:
        raise ValueError("reference is empty")
    indices = tuple(min(count - 1, int(signal_index + 10 * j))
                    for j in range(horizon_steps + 1))
    samples = tuple(
        ReferenceState(*(float(reference[name][idx]) for name in
                         ("x_ref", "vx_ref", "ax_ref", "y_ref", "z_ref", "yaw_ref")))
        for idx in indices
    )
    times = np.asarray([float(reference["time"][idx]) for idx in indices], dtype=float)
    if not np.isfinite(times).all() or np.any(np.diff(times) < 0.0):
        raise ValueError("reference horizon times must be finite and nondecreasing")
    return ReferenceHorizon(samples, indices, times)
