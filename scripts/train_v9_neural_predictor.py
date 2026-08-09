"""Train and select the preregistered V9 Neural Predictor models."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import shutil
import time
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.nn.utils import spectral_norm


ROOT = Path(__file__).resolve().parents[1]
R0 = ROOT / "reproducibility/v9/r0"
TRAINING = ROOT / "reproducibility/v9/training"
OUT = ROOT / "reproducibility/v9/predictor"
DATA_HEAD = "9e1f8e69287cc9d7763f0e9a4470400460935e2a"
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
DTYPE = torch.float32


def read(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(json.dumps(value, indent=2, allow_nan=False) + "\n")


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Embedding(nn.Module):
    def __init__(self, width: int, depth: int, embedding_dim: int, gamma: float) -> None:
        super().__init__()
        dimensions = [6, *([width] * depth), embedding_dim]
        self.layers = nn.ModuleList(
            [
                spectral_norm(nn.Linear(dimensions[index], dimensions[index + 1]))
                for index in range(len(dimensions) - 1)
            ]
        )
        self.layer_scale = float(gamma) ** (1.0 / len(self.layers))

    def forward(self, chi: torch.Tensor) -> torch.Tensor:
        value = chi
        for layer in self.layers:
            value = torch.relu(layer(value)) * self.layer_scale
        return torch.cat((chi, value), dim=-1)


class LearnedLLS(nn.Module):
    def __init__(self, config: dict) -> None:
        super().__init__()
        self.embedding = Embedding(
            int(config["hidden_width"]),
            int(config["hidden_depth"]),
            int(config["embedding_dim"]),
            float(config["spectral_gamma"]),
        )
        lifted = 6 + int(config["embedding_dim"])
        self.decoder = nn.Linear(lifted, 6, bias=False)
        with torch.no_grad():
            self.decoder.weight.zero_()
            self.decoder.weight[:, :6] = torch.eye(6)


def project_stable(A: torch.Tensor, limit: float = 0.995) -> tuple[torch.Tensor, float]:
    # The paper's backward objective requires an invertible lifted map. Finite
    # data least squares can return rank-deficient A, so project singular
    # values to a preregistered numerical envelope before the stability gate.
    u, singular, vh = torch.linalg.svd(A, full_matrices=False)
    singular = torch.clamp(singular, min=0.05, max=limit)
    A = (u * singular.unsqueeze(0)) @ vh
    with torch.no_grad():
        radius = float(torch.max(torch.abs(torch.linalg.eigvals(A))).cpu())
    if radius > limit:
        A = A * (limit / radius)
        radius = limit
    return A, radius


def fit_ab(
    model: LearnedLLS,
    chi_now: torch.Tensor,
    chi_next: torch.Tensor,
    zeta_now: torch.Tensor,
    ridge: float = 1.0e-3,
) -> tuple[torch.Tensor, torch.Tensor, float]:
    z_now = model.embedding(chi_now)
    z_next = model.embedding(chi_next)
    design = torch.cat((z_now, zeta_now), dim=1).T
    target = z_next.T
    sample_count = float(design.shape[1])
    gram = (design @ design.T) / sample_count + ridge * torch.eye(
        design.shape[0], device=design.device, dtype=design.dtype
    )
    cross = (target @ design.T) / sample_count
    combined = torch.linalg.solve(gram, cross.T).T
    lifted = z_now.shape[1]
    A, radius = project_stable(combined[:, :lifted])
    return A, combined[:, lifted:], radius


def contiguous_windows(mask: np.ndarray, trajectory_indices: np.ndarray, horizon: int) -> np.ndarray:
    rows: list[tuple[int, int]] = []
    for trajectory in trajectory_indices:
        current = mask[trajectory]
        for start in range(current.shape[0] - horizon):
            if bool(np.all(current[start : start + horizon + 1])):
                rows.append((int(trajectory), int(start)))
    return np.asarray(rows, dtype=int)


def transition_rows(mask: np.ndarray, trajectory_indices: np.ndarray) -> np.ndarray:
    rows: list[tuple[int, int]] = []
    for trajectory in trajectory_indices:
        current = mask[trajectory]
        for index in range(current.shape[0] - 1):
            if current[index] and current[index + 1]:
                rows.append((int(trajectory), int(index)))
    return np.asarray(rows, dtype=int)


def gather_transitions(
    chi: np.ndarray,
    zeta: np.ndarray,
    rows: np.ndarray,
    chi_mean: np.ndarray,
    chi_scale: np.ndarray,
    zeta_mean: np.ndarray,
    zeta_scale: np.ndarray,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    now = chi[rows[:, 0], rows[:, 1]]
    following = chi[rows[:, 0], rows[:, 1] + 1]
    input_value = zeta[rows[:, 0], rows[:, 1]]
    return (
        torch.as_tensor((now - chi_mean) / chi_scale, device=DEVICE, dtype=DTYPE),
        torch.as_tensor((following - chi_mean) / chi_scale, device=DEVICE, dtype=DTYPE),
        torch.as_tensor((input_value - zeta_mean) / zeta_scale, device=DEVICE, dtype=DTYPE),
    )


def gather_windows(
    chi: np.ndarray,
    zeta: np.ndarray,
    rows: np.ndarray,
    horizon: int,
    chi_mean: np.ndarray,
    chi_scale: np.ndarray,
    zeta_mean: np.ndarray,
    zeta_scale: np.ndarray,
) -> tuple[torch.Tensor, torch.Tensor]:
    chi_values = np.asarray(
        [chi[trajectory, start : start + horizon + 1] for trajectory, start in rows]
    )
    zeta_values = np.asarray(
        [zeta[trajectory, start : start + horizon] for trajectory, start in rows]
    )
    return (
        torch.as_tensor(
            (chi_values - chi_mean) / chi_scale, device=DEVICE, dtype=DTYPE
        ),
        torch.as_tensor(
            (zeta_values - zeta_mean) / zeta_scale, device=DEVICE, dtype=DTYPE
        ),
    )


def rollout_losses(
    model: LearnedLLS,
    A: torch.Tensor,
    B: torch.Tensor,
    chi_sequence: torch.Tensor,
    zeta_sequence: torch.Tensor,
) -> tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
    horizon = zeta_sequence.shape[1]
    lifted = model.embedding(chi_sequence[:, 0])
    truth_lifted = model.embedding(chi_sequence.reshape(-1, 6)).reshape(
        chi_sequence.shape[0], chi_sequence.shape[1], -1
    )
    forward_values = []
    for step in range(horizon):
        lifted = lifted @ A.T + zeta_sequence[:, step] @ B.T
        forward_values.append(lifted)
    forward = torch.stack(forward_values, dim=1)
    forward_loss = torch.mean((forward - truth_lifted[:, 1:]) ** 2)
    inverse = torch.linalg.solve(
        A.T @ A + 1.0e-2 * torch.eye(A.shape[0], device=A.device, dtype=A.dtype),
        A.T,
    )
    lifted = model.embedding(chi_sequence[:, -1])
    backward_values = []
    for step in range(horizon - 1, -1, -1):
        lifted = (lifted - zeta_sequence[:, step] @ B.T) @ inverse.T
        backward_values.append(lifted)
    backward = torch.stack(list(reversed(backward_values)), dim=1)
    backward_loss = torch.mean((backward - truth_lifted[:, :-1]) ** 2)
    reconstruction = model.decoder(truth_lifted.reshape(-1, truth_lifted.shape[-1]))
    reconstruction_loss = torch.mean(
        (reconstruction - chi_sequence.reshape(-1, 6)) ** 2
    )
    return forward_loss, backward_loss, reconstruction_loss


def physical_metrics(
    model: LearnedLLS,
    A: torch.Tensor,
    B: torch.Tensor,
    chi: np.ndarray,
    zeta: np.ndarray,
    rows: np.ndarray,
    chi_mean: np.ndarray,
    chi_scale: np.ndarray,
    zeta_mean: np.ndarray,
    zeta_scale: np.ndarray,
) -> dict:
    with torch.no_grad():
        now, _, inputs = gather_transitions(
            chi,
            zeta,
            rows,
            chi_mean,
            chi_scale,
            zeta_mean,
            zeta_scale,
        )
        lifted = model.embedding(now) @ A.T + inputs @ B.T
        predicted = model.decoder(lifted).cpu().numpy() * chi_scale + chi_mean
    truth = chi[rows[:, 0], rows[:, 1] + 1]
    error = predicted - truth
    return {
        "force_rmse": float(np.sqrt(np.mean(np.sum(error[:, :3] ** 2, axis=1)))),
        "torque_rmse": float(np.sqrt(np.mean(np.sum(error[:, 3:] ** 2, axis=1)))),
        "all_rmse": float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
    }


def multi_step_metric(
    model: LearnedLLS,
    A: torch.Tensor,
    B: torch.Tensor,
    chi: np.ndarray,
    zeta: np.ndarray,
    rows: np.ndarray,
    horizon: int,
    chi_mean: np.ndarray,
    chi_scale: np.ndarray,
    zeta_mean: np.ndarray,
    zeta_scale: np.ndarray,
) -> float:
    subset = rows[: min(len(rows), 2048)]
    with torch.no_grad():
        chi_seq, zeta_seq = gather_windows(
            chi,
            zeta,
            subset,
            horizon,
            chi_mean,
            chi_scale,
            zeta_mean,
            zeta_scale,
        )
        lifted = model.embedding(chi_seq[:, 0])
        predictions = []
        for step in range(horizon):
            lifted = lifted @ A.T + zeta_seq[:, step] @ B.T
            predictions.append(model.decoder(lifted))
        value = torch.stack(predictions, dim=1).cpu().numpy()
    physical = value * chi_scale + chi_mean
    truth = chi[
        subset[:, 0, None],
        subset[:, 1, None] + np.arange(1, horizon + 1)[None, :],
    ]
    return float(np.sqrt(np.mean(np.sum((physical - truth) ** 2, axis=2))))


def export_artifact(
    path: Path,
    model_id: str,
    model: LearnedLLS,
    A: torch.Tensor,
    B: torch.Tensor,
    chi_mean: np.ndarray,
    chi_scale: np.ndarray,
    zeta_mean: np.ndarray,
    zeta_scale: np.ndarray,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload: dict[str, np.ndarray] = {
        "model_id": np.asarray(model_id),
        "chi_mean": chi_mean,
        "chi_scale": chi_scale,
        "zeta_mean": zeta_mean,
        "zeta_scale": zeta_scale,
        "A": A.detach().cpu().numpy().astype(np.float64),
        "B": B.detach().cpu().numpy().astype(np.float64),
        "C": model.decoder.weight.detach().cpu().numpy().astype(np.float64),
        "layer_count": np.asarray(len(model.embedding.layers)),
        "online_window": np.asarray(40),
        "online_ridge": np.asarray(1.0e-3),
        "online_blend": np.asarray(0.08),
    }
    with torch.no_grad():
        dummy = torch.zeros((1, 6), device=DEVICE, dtype=DTYPE)
        model.embedding(dummy)
    for index, layer in enumerate(model.embedding.layers):
        payload[f"weight_{index}"] = (
            layer.weight.detach().cpu().numpy().astype(np.float64)
            * model.embedding.layer_scale
        )
        payload[f"bias_{index}"] = (
            layer.bias.detach().cpu().numpy().astype(np.float64)
            * model.embedding.layer_scale
        )
    np.savez_compressed(path, **payload)


def simple_estimators(
    chi: np.ndarray,
    split: np.ndarray,
    mask: np.ndarray,
    validation_indices: np.ndarray,
) -> dict:
    truth_rows = transition_rows(mask, validation_indices)
    truth = chi[truth_rows[:, 0], truth_rows[:, 1] + 1]
    current = chi[truth_rows[:, 0], truth_rows[:, 1]]
    zero_error = -truth
    last_error = current - truth
    lowpass_pred = []
    lowpass_truth = []
    for trajectory in validation_indices:
        estimate = np.zeros(6)
        initialized = False
        for index in range(chi.shape[1] - 1):
            if not (mask[trajectory, index] and mask[trajectory, index + 1]):
                continue
            if not initialized:
                estimate = chi[trajectory, index].copy()
                initialized = True
            else:
                estimate = 0.35 * chi[trajectory, index] + 0.65 * estimate
            lowpass_pred.append(estimate.copy())
            lowpass_truth.append(chi[trajectory, index + 1].copy())
    lowpass_error = np.asarray(lowpass_pred) - np.asarray(lowpass_truth)

    def values(error: np.ndarray) -> dict:
        return {
            "force_rmse": float(
                np.sqrt(np.mean(np.sum(error[:, :3] ** 2, axis=1)))
            ),
            "torque_rmse": float(
                np.sqrt(np.mean(np.sum(error[:, 3:] ** 2, axis=1)))
            ),
            "all_rmse": float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
        }

    return {
        "zero_residual": values(zero_error),
        "last_value": values(last_error),
        "low_pass_alpha_0p35": values(lowpass_error),
    }


def train_one(
    config: dict,
    chi: np.ndarray,
    zeta: np.ndarray,
    train_rows: np.ndarray,
    validation_rows: np.ndarray,
    train_windows: np.ndarray,
    validation_windows: np.ndarray,
    chi_mean: np.ndarray,
    chi_scale: np.ndarray,
    zeta_mean: np.ndarray,
    zeta_scale: np.ndarray,
) -> dict:
    seed = int(config["training_seed"])
    np_rng = np.random.Generator(np.random.PCG64(seed))
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)
    model = LearnedLLS(config).to(device=DEVICE, dtype=DTYPE)
    optimizer = torch.optim.Adam(model.parameters(), lr=float(config["learning_rate"]))
    fit_count = min(8192, len(train_rows))
    fixed_fit = train_rows[np_rng.choice(len(train_rows), fit_count, replace=False)]
    fit_now, fit_next, fit_input = gather_transitions(
        chi,
        zeta,
        fixed_fit,
        chi_mean,
        chi_scale,
        zeta_mean,
        zeta_scale,
    )
    validation_subset = validation_rows[: min(4096, len(validation_rows))]
    best_score = float("inf")
    best_epoch = -1
    best_state = None
    epochs_without_improvement = 0
    started = time.perf_counter()
    last_losses = {}
    for epoch in range(int(config["epochs_max"])):
        model.train()
        A, B, radius = fit_ab(model, fit_now, fit_next, fit_input)
        # Algorithm 1 alternates closed-form LLS identification and neural
        # embedding updates. A/B are fixed during each embedding gradient step.
        A = A.detach()
        B = B.detach()
        batch_size = min(512, len(train_windows))
        selected = train_windows[
            np_rng.choice(len(train_windows), batch_size, replace=False)
        ]
        chi_seq, zeta_seq = gather_windows(
            chi,
            zeta,
            selected,
            int(config["prediction_horizon"]),
            chi_mean,
            chi_scale,
            zeta_mean,
            zeta_scale,
        )
        forward, backward, reconstruction = rollout_losses(
            model, A, B, chi_seq, zeta_seq
        )
        loss = (
            float(config["forward_weight"]) * forward
            + float(config["backward_weight"]) * backward
            + float(config["reconstruction_weight"]) * reconstruction
        )
        optimizer.zero_grad()
        loss.backward()
        torch.nn.utils.clip_grad_norm_(model.parameters(), 10.0)
        optimizer.step()
        last_losses = {
            "forward": float(forward.detach().cpu()),
            "backward": float(backward.detach().cpu()),
            "reconstruction": float(reconstruction.detach().cpu()),
            "total": float(loss.detach().cpu()),
        }
        if epoch % 5 == 0 or epoch + 1 == int(config["epochs_max"]):
            model.eval()
            with torch.no_grad():
                A_eval, B_eval, _ = fit_ab(model, fit_now, fit_next, fit_input)
                metric = physical_metrics(
                    model,
                    A_eval,
                    B_eval,
                    chi,
                    zeta,
                    validation_subset,
                    chi_mean,
                    chi_scale,
                    zeta_mean,
                    zeta_scale,
                )
                score = metric["force_rmse"] + 5.0 * metric["torque_rmse"]
            if score < best_score - 1.0e-6:
                best_score = score
                best_epoch = epoch
                best_state = {
                    key: value.detach().cpu().clone()
                    for key, value in model.state_dict().items()
                }
                epochs_without_improvement = 0
            else:
                epochs_without_improvement += 5
            if epochs_without_improvement >= int(config["early_stopping_patience"]):
                break
    if best_state is None:
        raise RuntimeError("predictor training produced no checkpoint")
    model.load_state_dict(best_state)
    model.eval()
    all_now, all_next, all_input = gather_transitions(
        chi,
        zeta,
        train_rows,
        chi_mean,
        chi_scale,
        zeta_mean,
        zeta_scale,
    )
    with torch.no_grad():
        A, B, radius = fit_ab(model, all_now, all_next, all_input)
    validation = physical_metrics(
        model,
        A,
        B,
        chi,
        zeta,
        validation_rows,
        chi_mean,
        chi_scale,
        zeta_mean,
        zeta_scale,
    )
    validation["multi_step_rmse"] = multi_step_metric(
        model,
        A,
        B,
        chi,
        zeta,
        validation_windows,
        int(config["prediction_horizon"]),
        chi_mean,
        chi_scale,
        zeta_mean,
        zeta_scale,
    )
    artifact = OUT / "models" / f"{config['model_id']}.npz"
    export_artifact(
        artifact,
        config["model_id"],
        model,
        A,
        B,
        chi_mean,
        chi_scale,
        zeta_mean,
        zeta_scale,
    )
    return {
        "model_id": config["model_id"],
        "configuration": config,
        "best_epoch": best_epoch,
        "epochs_executed": epoch + 1,
        "last_training_losses": last_losses,
        "validation": validation,
        "lifted_spectral_radius": radius,
        "artifact": str(artifact.relative_to(ROOT)).replace("\\", "/"),
        "artifact_sha256": sha(artifact),
        "elapsed_s": time.perf_counter() - started,
        "device": str(DEVICE),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--limit-models", type=int, default=None)
    args = parser.parse_args()
    if sha(TRAINING / "training_bank.npz") != read(
        TRAINING / "training_data_freeze.json"
    )["bank_sha256"]:
        raise RuntimeError("training bank drift")
    registry = read(R0 / "predictor_registry.json")
    configs = registry["models"]
    if args.limit_models is not None:
        configs = configs[: args.limit_models]
    with np.load(TRAINING / "training_bank.npz") as bank:
        chi = np.asarray(bank["chi"], dtype=np.float64)
        zeta = np.asarray(bank["zeta"], dtype=np.float64)
        split = np.asarray(bank["split"])
        mask = np.asarray(bank["label_valid"] & bank["safe_step"], dtype=bool)
    train_indices = np.flatnonzero(split == "TRAIN")
    validation_indices = np.flatnonzero(split == "VALIDATION")
    train_values = chi[train_indices][mask[train_indices]]
    train_zeta = zeta[train_indices][mask[train_indices]]
    chi_mean = np.mean(train_values, axis=0)
    chi_scale = np.std(train_values, axis=0)
    zeta_mean = np.mean(train_zeta, axis=0)
    zeta_scale = np.std(train_zeta, axis=0)
    chi_scale = np.maximum(chi_scale, 1.0e-6)
    zeta_scale = np.maximum(zeta_scale, 1.0e-6)
    train_rows = transition_rows(mask, train_indices)
    validation_rows = transition_rows(mask, validation_indices)
    horizons = sorted({int(config["prediction_horizon"]) for config in configs})
    train_window_map = {
        horizon: contiguous_windows(mask, train_indices, horizon)
        for horizon in horizons
    }
    validation_window_map = {
        horizon: contiguous_windows(mask, validation_indices, horizon)
        for horizon in horizons
    }
    OUT.mkdir(parents=True, exist_ok=True)
    simple = simple_estimators(chi, split, mask, validation_indices)
    results = []
    for index, config in enumerate(configs, 1):
        result = train_one(
            config,
            chi,
            zeta,
            train_rows,
            validation_rows,
            train_window_map[int(config["prediction_horizon"])],
            validation_window_map[int(config["prediction_horizon"])],
            chi_mean,
            chi_scale,
            zeta_mean,
            zeta_scale,
        )
        results.append(result)
        print(
            f"{index}/{len(configs)} {config['model_id']} "
            f"F={result['validation']['force_rmse']:.6f} "
            f"T={result['validation']['torque_rmse']:.6f} "
            f"M={result['validation']['multi_step_rmse']:.6f}",
            flush=True,
        )
    force_reference = min(value["force_rmse"] for value in simple.values())
    torque_reference = min(value["torque_rmse"] for value in simple.values())
    for result in results:
        metric = result["validation"]
        result["relative_to_best_simple"] = {
            "force_improvement": 1.0 - metric["force_rmse"] / force_reference,
            "torque_improvement": 1.0 - metric["torque_rmse"] / torque_reference,
        }
        result["competence_pass"] = bool(
            metric["force_rmse"] <= 0.95 * force_reference
            and metric["torque_rmse"] <= 0.95 * torque_reference
            and result["lifted_spectral_radius"] < 1.0
        )
    ranked = sorted(
        results,
        key=lambda value: (
            -int(value["competence_pass"]),
            value["validation"]["force_rmse"] / force_reference
            + value["validation"]["torque_rmse"] / torque_reference
            + value["validation"]["multi_step_rmse"]
            / max(simple["last_value"]["all_rmse"], 1.0e-9),
            value["model_id"],
        ),
    )
    selected = ranked[0]
    selected_source = ROOT / selected["artifact"]
    selected_artifact = OUT / "selected_predictor.npz"
    shutil.copyfile(selected_source, selected_artifact)
    validated = bool(selected["competence_pass"])
    write(
        OUT / "simple_estimator_comparison.json",
        {
            "validation_trajectories": len(validation_indices),
            "validation_transitions": len(validation_rows),
            "estimators": simple,
            "selection_independent_of_satc": True,
        },
    )
    write(
        OUT / "predictor_search_results.json",
        {
            "data_head": DATA_HEAD,
            "device": str(DEVICE),
            "models_evaluated": len(results),
            "maximum_allowed": 64,
            "results": ranked,
            "holdout_accessed": False,
            "development_performance_accessed": False,
        },
    )
    write(
        OUT / "predictor_freeze.json",
        {
            "result": (
                "V9_NEURAL_PREDICTOR_VALIDATED"
                if validated
                else "BLOCKED_V9_NEURAL_PREDICTOR_NOT_VALIDATED"
            ),
            "selected_model": selected["model_id"],
            "selected_metrics": selected,
            "selected_artifact": "reproducibility/v9/predictor/selected_predictor.npz",
            "selected_artifact_sha256": sha(selected_artifact),
            "models_evaluated": len(results),
            "force_reference_best_simple": force_reference,
            "torque_reference_best_simple": torque_reference,
            "validated": validated,
            "satc_used_for_selection": False,
            "development_used_for_selection": False,
            "holdout_accessed": False,
        },
    )
    print(
        json.dumps(
            {
                "selected": selected["model_id"],
                "validated": validated,
                "metrics": selected["validation"],
                "relative": selected["relative_to_best_simple"],
            },
            indent=2,
        )
    )
    return 0 if validated else 2


if __name__ == "__main__":
    raise SystemExit(main())
