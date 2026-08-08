"""Fixed-horizon affine preview model used by the DA-PMPC pilot."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def reference_vector(reference) -> np.ndarray:
    """Return the affine 16-state reference vector."""
    result = np.zeros(16, dtype=float)
    result[0] = float(reference.x_ref)
    result[1] = float(reference.vx_ref)
    result[2] = float(reference.z_ref) - 3.2
    return result


@dataclass(frozen=True)
class PreviewResult:
    states: np.ndarray
    input_matrix: np.ndarray


class PreviewModel:
    """Build the DA-PMPC affine preview model.

    ``B`` is the physical S4 input matrix, with no hidden sign inversion.  In
    this project the reduced-state input convention is
    ``x[k+1] = A x[k] + B (a[k] + d[k]) - c[k]``.  Keeping that convention
    explicit is important because the observer and QP use the same model.
    """

    def __init__(self, A: np.ndarray, B: np.ndarray, Q: np.ndarray,
                 P: np.ndarray, C_tip: np.ndarray, horizon_steps: int = 20,
                 control_weight: float = 1.0):
        self.A = np.asarray(A, dtype=float)
        self.B = np.asarray(B, dtype=float).reshape(16, 1)
        self.Q = np.asarray(Q, dtype=float)
        self.P = np.asarray(P, dtype=float)
        self.C_tip = np.asarray(C_tip, dtype=float).reshape(1, 16)
        self.horizon_steps = int(horizon_steps)
        self.control_weight = float(control_weight)
        if self.A.shape != (16, 16) or self.B.shape != (16, 1):
            raise ValueError("preview model requires A 16x16 and B 16x1")

    @staticmethod
    def reference_vector(reference) -> np.ndarray:
        """Return the affine reference state used by the S4 layout.

        The reduced state is an error state.  In particular, the altitude
        component is measured relative to the nominal 3.2 m hover height;
        joint and attitude reference components are zero.
        """
        return reference_vector(reference)

    def reference_shift(self, reference_i, reference_next) -> np.ndarray:
        """Return ``r_next - A @ r_i`` for error-coordinate dynamics."""
        return reference_vector(reference_next) - self.A @ reference_vector(reference_i)

    @staticmethod
    def static_reference_shift(A: np.ndarray, reference_i, reference_next) -> np.ndarray:
        """Functional form useful to observers and analytical tests."""
        A = np.asarray(A, dtype=float).reshape(16, 16)
        return reference_vector(reference_next) - A @ reference_vector(reference_i)

    def rollout(self, x0: np.ndarray, actions: np.ndarray, references,
                disturbance: float = 0.0) -> PreviewResult:
        x = np.asarray(x0, dtype=float).reshape(16)
        actions = np.asarray(actions, dtype=float).reshape(self.horizon_steps)
        if len(references) != self.horizon_steps + 1:
            raise ValueError("preview requires H+1 reference boundaries")
        states = np.zeros((self.horizon_steps + 1, 16), dtype=float)
        states[0] = x
        for i in range(self.horizon_steps):
            c = self.reference_shift(references[i], references[i + 1])
            states[i + 1] = self.A @ states[i] + self.B[:, 0] * (actions[i] + float(disturbance)) - c
        return PreviewResult(states=states, input_matrix=self.B.copy())
