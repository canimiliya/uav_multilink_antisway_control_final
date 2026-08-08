"""Full-state discrete LQR outer acceleration controller."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from .acceleration_limiter import AccelerationLimiter
from .base import ReferenceState


def lqr_candidate_score(
    tip_rms_ratios: Sequence[float],
    position_rmse_ratios: Sequence[float],
    control_rate_ratios: Sequence[float],
    saturation_rates: Sequence[float],
) -> float:
    """Return the frozen S4 candidate score.

    Every term is a penalty.  In particular, worse position tracking and
    higher saturation must increase the score rather than improve it.
    """
    arrays = [
        np.asarray(tip_rms_ratios, dtype=float),
        np.asarray(position_rmse_ratios, dtype=float),
        np.asarray(control_rate_ratios, dtype=float),
        np.asarray(saturation_rates, dtype=float),
    ]
    if any(array.size == 0 or not np.isfinite(array).all() for array in arrays):
        raise ValueError("LQR score inputs must be non-empty and finite")
    return float(
        np.mean(arrays[0])
        + 0.25 * np.mean(arrays[1])
        + 0.05 * np.mean(arrays[2])
        + 0.05 * np.mean(arrays[3])
    )


@dataclass(frozen=True)
class LQRDiagnostics:
    lqr_feedback_ax: float
    ax_reference_feedforward: float
    ax_cmd_raw: float
    ax_cmd_amplitude_limited: float
    ax_cmd_limited: float
    ax_saturated: bool
    ax_slew_limited: bool
    lqr_state_norm: float


class FullStateLQR:
    """Returns only x acceleration; all actuator conversion stays in S3's inner loop."""

    def __init__(self, gain: np.ndarray, ax_min: float = -2.0,
                 ax_max: float = 2.0, slew_limit: float = 0.25):
        gain = np.asarray(gain, dtype=float)
        if gain.shape != (1, 16):
            raise ValueError(f"expected K shape (1, 16), got {gain.shape}")
        if not np.isfinite(gain).all():
            raise ValueError("LQR gain contains non-finite values")
        self.gain = gain.copy()
        self.limiter = AccelerationLimiter(ax_min, ax_max, slew_limit)
        self.diagnostics = LQRDiagnostics(0.0, 0.0, 0.0, 0.0, 0.0, False, False, 0.0)

    def reset(self, state=None, reference=None) -> None:
        del state, reference
        self.limiter.reset(0.0)
        self.diagnostics = LQRDiagnostics(0.0, 0.0, 0.0, 0.0, 0.0, False, False, 0.0)

    def command(self, state: np.ndarray, reference: ReferenceState, dt: float = 0.05) -> float:
        del dt
        x = np.asarray(state, dtype=float)
        if x.shape != (16,) or not np.isfinite(x).all():
            raise ValueError("LQR state must be a finite 16-vector")
        feedback = float((-self.gain @ x.reshape(-1, 1))[0, 0])
        raw = float(reference.ax_ref + feedback)
        limited = self.limiter.limit(raw)
        d = self.limiter.diagnostics
        self.diagnostics = LQRDiagnostics(
            feedback, float(reference.ax_ref), raw, d.amplitude_limited, limited,
            d.saturated, d.slew_limited, float(np.linalg.norm(x)),
        )
        return limited
