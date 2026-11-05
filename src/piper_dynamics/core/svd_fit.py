"""Truncated-SVD least squares in CAD-scaled parameter coordinates."""

from __future__ import annotations

import numpy as np


def solve_truncated_svd(
    design: np.ndarray,
    target: np.ndarray,
    nominal: np.ndarray,
    scales: np.ndarray,
    rank_tolerance: float = 1e-8,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Solve ``design @ theta ~= target`` with unresolved directions kept at nominal.

    Args:
        design: Stacked regressor of shape ``(N, 6, 78)`` or ``(6N, 78)``.
        target: Torque targets of shape ``(N, 6)`` or ``(6N,)``.
        nominal: Nominal (CAD) parameter vector of length 78.
        scales: Per-parameter characteristic scales, same length as ``nominal``.
        rank_tolerance: Relative singular-value threshold.

    Returns:
        ``(theta, singular_values, keep_mask)``.

    Raises:
        ValueError: for invalid input shapes or a rank tolerance outside (0, 1).
    """
    if not np.isfinite(rank_tolerance) or not 0 < rank_tolerance < 1:
        raise ValueError("rank_tolerance must be finite and between 0 and 1")
    nominal = np.asarray(nominal, dtype=float)
    scales = np.asarray(scales, dtype=float)
    if nominal.shape != scales.shape or nominal.ndim != 1:
        raise ValueError("nominal and scales must be one-dimensional and equal length")
    matrix = np.asarray(design, dtype=float).reshape(-1, nominal.size)
    values = np.asarray(target, dtype=float).reshape(-1)
    if matrix.shape[0] != values.size:
        raise ValueError("design and target row counts differ")

    u, singular, vt = np.linalg.svd(matrix * scales, full_matrices=False)
    keep = singular > rank_tolerance * singular[0]
    if not np.any(keep):
        raise ValueError("regressor has no identifiable directions")
    residual = values - matrix @ nominal
    delta = (u[:, keep].T @ residual) / singular[keep]
    theta = nominal + scales * (vt[keep].T @ delta)
    return theta, singular, keep


def predict_torque(design: np.ndarray, theta: np.ndarray) -> np.ndarray:
    """Evaluate the torque model for every sample: shape ``(N, 6)``."""
    return np.einsum("nij,j->ni", np.asarray(design, dtype=float), np.asarray(theta, dtype=float))
