"""Shared offline regression features, parameter scales, and fit metrics."""

from __future__ import annotations

import hashlib
from pathlib import Path

import numpy as np


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def nominal_parameter_scales(theta_nominal: np.ndarray) -> np.ndarray:
    """Characteristic scales from nominal links, without amplifying near-zero columns."""
    scales = np.empty(78)
    for i in range(6):
        t = theta_nominal[10*i:10*(i+1)]
        m = float(t[0])
        h = t[1:4]
        I = np.array([[t[4], t[7], t[8]],
                      [t[7], t[5], t[9]],
                      [t[8], t[9], t[6]]])
        I_com = I - ((h @ h) * np.eye(3) - np.outer(h, h)) / m
        ell = float(np.sqrt(max(np.trace(I_com) / (2*m), 1e-10)))
        scales[10*i:10*(i+1)] = [m, *([m*ell]*3), *([m*ell*ell]*6)]
    scales[60:] = np.tile([1.0, 0.5, 1.0], 6)
    return scales


def make_friction(qd: np.ndarray) -> np.ndarray:
    """Six joints, each with viscous, smooth Coulomb and offset columns."""
    n = len(qd)
    features = np.zeros((n, 6, 18))
    for j in range(6):
        features[:, j, 3*j:3*j+3] = np.column_stack([
            qd[:, j], np.tanh(qd[:, j] / 0.02), np.ones(n),
        ])
    return features


def metrics(pred: np.ndarray, target: np.ndarray) -> dict:
    error = pred - target
    target_rms = np.sqrt(np.mean(target**2, axis=0))
    rmse = np.sqrt(np.mean(error**2, axis=0))
    denom = np.sum((target-target.mean(axis=0))**2, axis=0)
    return {
        "rmse_Nm": rmse.tolist(), "target_rms_Nm": target_rms.tolist(),
        "rmse_over_target_rms": (rmse / np.maximum(target_rms, 1e-12)).tolist(),
        "r2_per_joint": (1 - np.sum(error**2, axis=0) / np.maximum(denom, 1e-24)).tolist(),
        "relative_error_norm_ratio": float(np.linalg.norm(error) / max(np.linalg.norm(target), 1e-12)),
        "squared_error_sum_Nm2": float(np.sum(error**2)),
    }
