"""Matched scalar disturbance observer using state and command history only."""

from __future__ import annotations

import numpy as np


class MatchedDisturbanceObserver:
    """17-state observer for x[k+1]=A x+B(u+d).

    The measured 16-state vector is accepted directly.  No wind, force, or
    profile object is part of this API; d_hat is updated from prediction error.
    """

    def __init__(self, A, B, gain: float = 0.15, limit: float = 2.0):
        self.A = np.asarray(A, dtype=float).reshape(16, 16)
        self.B = np.asarray(B, dtype=float).reshape(16)
        self.gain = float(gain); self.limit = float(limit)
        self.x_hat = np.zeros(16)
        self._previous_state = np.zeros(16)
        self.d_hat = 0.0
        self._initialized = False

    @property
    def dimension(self): return 17

    def reset(self, state=None, disturbance: float = 0.0, command: float = 0.0):
        del command
        self.x_hat = np.zeros(16) if state is None else np.asarray(state, dtype=float).copy()
        self._previous_state = self.x_hat.copy()
        self.d_hat = float(disturbance)
        self._initialized = False

    def update(self, measured_state, applied_previous_command: float,
               reference_shift=None) -> float:
        """Update from the applied command that produced this sample.

        The first measurement only initializes the observer.  Thereafter the
        prediction uses the immediately preceding measured state and the
        command supplied by the caller, before the caller solves the new QP.
        ``reference_shift`` is the same error-coordinate shift used by
        :class:`PreviewModel`.
        """
        y = np.asarray(measured_state, dtype=float).reshape(16)
        shift = np.zeros(16, dtype=float) if reference_shift is None else np.asarray(reference_shift, dtype=float).reshape(16)
        if not self._initialized:
            self.x_hat = y.copy()
            self._previous_state = y.copy()
            self._initialized = True
            return self.d_hat
        prediction = (self.A @ self._previous_state
                      + self.B * (float(applied_previous_command) + self.d_hat)
                      - shift)
        innovation = y - prediction
        scale = float(self.B @ self.B) + 1.0e-12
        self.d_hat = float(np.clip(self.d_hat + self.gain * float(self.B @ innovation) / scale,
                                   -self.limit, self.limit))
        self.x_hat = y.copy()
        self._previous_state = y.copy()
        return self.d_hat

    def augmented_state(self) -> np.ndarray:
        return np.r_[self.x_hat, self.d_hat]
