"""Controllability, PBH and discrete DARE helpers."""

from __future__ import annotations

import numpy as np
from scipy.linalg import solve_discrete_are


def controllability_analysis(a: np.ndarray, b: np.ndarray, tolerance: float = 1e-10) -> dict:
    n = a.shape[0]
    blocks = [b]
    current = b.copy()
    for _ in range(1, n):
        current = a @ current
        blocks.append(current)
    matrix = np.hstack(blocks)
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    rank = int(np.linalg.matrix_rank(matrix, tol=tolerance))
    eigenvalues = np.linalg.eigvals(a)
    pbh = []
    for value in eigenvalues:
        if abs(value) >= 1.0 - 1e-10:
            pbh_rank = int(np.linalg.matrix_rank(np.hstack([value * np.eye(n) - a, b]), tol=tolerance))
            pbh.append({"eigenvalue_real": float(value.real), "eigenvalue_imag": float(value.imag), "abs": float(abs(value)), "rank": pbh_rank, "stabilizable": pbh_rank == n})
    return {"rank": rank, "singular_values": singular_values.tolist(), "tolerance": tolerance, "open_loop_eigenvalues": [{"real": float(v.real), "imag": float(v.imag), "abs": float(abs(v))} for v in eigenvalues], "unstable_modes": pbh, "pbh_stabilizable": bool(all(item["stabilizable"] for item in pbh))}


def solve_lqr(a: np.ndarray, b: np.ndarray, q: np.ndarray, r: np.ndarray) -> dict:
    p = solve_discrete_are(a, b, q, r)
    p = 0.5 * (p + p.T)
    k = np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a)
    acl = a - b @ k
    eig = np.linalg.eigvals(acl)
    residual = p - (a.T @ p @ a - a.T @ p @ b @ np.linalg.solve(r + b.T @ p @ b, b.T @ p @ a) + q)
    return {"P": p, "K": k, "Acl": acl, "eigenvalues": eig, "spectral_radius": float(np.max(np.abs(eig))), "dare_residual_norm": float(np.linalg.norm(residual, ord="fro")), "p_symmetry_error": float(np.max(np.abs(p - p.T))), "p_min_eigenvalue": float(np.min(np.linalg.eigvalsh(p)))}
