"""Pure-NumPy runtime for the Jin et al. Neural Predictor adaptation."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def _relu(value: np.ndarray) -> np.ndarray:
    return np.maximum(np.asarray(value, dtype=float), 0.0)


def _stable_projection(matrix: np.ndarray, limit: float = 0.995) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=float)
    radius = float(np.max(np.abs(np.linalg.eigvals(matrix))))
    if not np.isfinite(radius):
        raise ValueError("lifted A has non-finite spectral radius")
    if radius > limit:
        matrix = matrix * (limit / radius)
    return matrix


@dataclass(frozen=True)
class NeuralPredictorArtifact:
    model_id: str
    chi_mean: np.ndarray
    chi_scale: np.ndarray
    zeta_mean: np.ndarray
    zeta_scale: np.ndarray
    weights: tuple[np.ndarray, ...]
    biases: tuple[np.ndarray, ...]
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    online_window: int
    online_ridge: float
    online_blend: float

    @classmethod
    def load(cls, path: str | Path) -> "NeuralPredictorArtifact":
        with np.load(Path(path), allow_pickle=False) as data:
            count = int(data["layer_count"])
            weights = tuple(np.asarray(data[f"weight_{i}"], dtype=float) for i in range(count))
            biases = tuple(np.asarray(data[f"bias_{i}"], dtype=float) for i in range(count))
            value = cls(
                model_id=str(data["model_id"]),
                chi_mean=np.asarray(data["chi_mean"], dtype=float).reshape(6),
                chi_scale=np.asarray(data["chi_scale"], dtype=float).reshape(6),
                zeta_mean=np.asarray(data["zeta_mean"], dtype=float).reshape(6),
                zeta_scale=np.asarray(data["zeta_scale"], dtype=float).reshape(6),
                weights=weights,
                biases=biases,
                A=np.asarray(data["A"], dtype=float),
                B=np.asarray(data["B"], dtype=float),
                C=np.asarray(data["C"], dtype=float),
                online_window=int(data["online_window"]),
                online_ridge=float(data["online_ridge"]),
                online_blend=float(data["online_blend"]),
            )
        value.validate()
        return value

    def validate(self) -> None:
        if len(self.weights) != len(self.biases) or not self.weights:
            raise ValueError("invalid embedding layers")
        width = 6
        for weight, bias in zip(self.weights, self.biases):
            if weight.ndim != 2 or weight.shape[1] != width:
                raise ValueError("embedding weight shape mismatch")
            if bias.shape != (weight.shape[0],):
                raise ValueError("embedding bias shape mismatch")
            width = weight.shape[0]
        lifted = 6 + width
        if self.A.shape != (lifted, lifted):
            raise ValueError("A shape mismatch")
        if self.B.shape != (lifted, 6):
            raise ValueError("B shape mismatch")
        if self.C.shape != (6, lifted):
            raise ValueError("C shape mismatch")
        arrays = (
            self.chi_mean,
            self.chi_scale,
            self.zeta_mean,
            self.zeta_scale,
            self.A,
            self.B,
            self.C,
            *self.weights,
            *self.biases,
        )
        if not all(np.isfinite(array).all() for array in arrays):
            raise ValueError("predictor artifact contains non-finite values")
        if np.any(self.chi_scale <= 0.0) or np.any(self.zeta_scale <= 0.0):
            raise ValueError("normalization scales must be positive")
        if self.online_window < 3 or self.online_ridge <= 0.0:
            raise ValueError("invalid online update contract")
        if not 0.0 <= self.online_blend <= 1.0:
            raise ValueError("invalid online blend")

    @property
    def lifted_dimension(self) -> int:
        return int(self.A.shape[0])

    def normalize_chi(self, chi: np.ndarray) -> np.ndarray:
        return (np.asarray(chi, dtype=float).reshape(6) - self.chi_mean) / self.chi_scale

    def normalize_zeta(self, zeta: np.ndarray) -> np.ndarray:
        return (np.asarray(zeta, dtype=float).reshape(6) - self.zeta_mean) / self.zeta_scale

    def denormalize_chi(self, chi: np.ndarray) -> np.ndarray:
        return np.asarray(chi, dtype=float).reshape(6) * self.chi_scale + self.chi_mean

    def embed_normalized(self, chi_normalized: np.ndarray) -> np.ndarray:
        original = np.asarray(chi_normalized, dtype=float).reshape(6)
        value = original
        for weight, bias in zip(self.weights, self.biases):
            value = _relu(weight @ value + bias)
        return np.r_[original, value]

    def embed(self, chi: np.ndarray) -> np.ndarray:
        return self.embed_normalized(self.normalize_chi(chi))

    def reconstruct(self, lifted: np.ndarray) -> np.ndarray:
        normalized = self.C @ np.asarray(lifted, dtype=float).reshape(self.lifted_dimension)
        return self.denormalize_chi(normalized)

    def predict_one(self, chi: np.ndarray, zeta: np.ndarray) -> np.ndarray:
        lifted = self.A @ self.embed(chi) + self.B @ self.normalize_zeta(zeta)
        return self.reconstruct(lifted)


class CausalLiftedPredictor:
    """Frozen embedding with causal rolling least-squares A/B updates."""

    def __init__(self, artifact: NeuralPredictorArtifact) -> None:
        self.artifact = artifact
        self.reset()

    def reset(self) -> None:
        self.A = self.artifact.A.copy()
        self.B = self.artifact.B.copy()
        self.chi_history: deque[np.ndarray] = deque(maxlen=self.artifact.online_window + 1)
        self.zeta_history: deque[np.ndarray] = deque(maxlen=self.artifact.online_window)
        self.pending_zeta: np.ndarray | None = None
        self.update_count = 0

    def observe(self, chi: np.ndarray, zeta: np.ndarray) -> None:
        chi = np.asarray(chi, dtype=float).reshape(6)
        zeta = np.asarray(zeta, dtype=float).reshape(6)
        if not np.isfinite(np.r_[chi, zeta]).all():
            raise ValueError("causal predictor history must be finite")
        if self.chi_history and self.pending_zeta is not None:
            self.zeta_history.append(self.pending_zeta.copy())
        self.chi_history.append(chi.copy())
        self.pending_zeta = self.artifact.normalize_zeta(zeta)
        if (
            len(self.zeta_history) == self.artifact.online_window
            and len(self.chi_history) == self.artifact.online_window + 1
        ):
            self._online_update()

    def _online_update(self) -> None:
        chi = list(self.chi_history)
        zeta = list(self.zeta_history)
        current = np.column_stack([self.artifact.embed(value) for value in chi[:-1]])
        following = np.column_stack([self.artifact.embed(value) for value in chi[1:]])
        controls = np.column_stack(zeta)
        design = np.vstack((current, controls))
        gram = design @ design.T + self.artifact.online_ridge * np.eye(design.shape[0])
        estimate = following @ design.T @ np.linalg.inv(gram)
        k = self.artifact.lifted_dimension
        candidate_a = _stable_projection(estimate[:, :k])
        candidate_b = estimate[:, k:]
        blend = self.artifact.online_blend
        self.A = _stable_projection((1.0 - blend) * self.A + blend * candidate_a)
        self.B = (1.0 - blend) * self.B + blend * candidate_b
        self.update_count += 1

    def predict_sequence(
        self,
        chi: np.ndarray,
        zeta_sequence: np.ndarray,
    ) -> np.ndarray:
        zeta_sequence = np.asarray(zeta_sequence, dtype=float)
        if zeta_sequence.ndim != 2 or zeta_sequence.shape[1] != 6:
            raise ValueError("zeta_sequence must have shape [horizon, 6]")
        lifted = self.artifact.embed(chi)
        result = np.zeros((len(zeta_sequence), 6))
        for index, zeta in enumerate(zeta_sequence):
            lifted = self.A @ lifted + self.B @ self.artifact.normalize_zeta(zeta)
            result[index] = self.artifact.reconstruct(lifted)
        return result
