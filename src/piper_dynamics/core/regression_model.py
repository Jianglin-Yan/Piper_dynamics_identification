"""Regression features, CAD-based parameter scaling and prediction metrics."""

from __future__ import annotations

import numpy as np

#: Per-joint friction columns appended after the 60 rigid-body columns.
FRICTION_NAMES = ("viscous_friction", "smooth_coulomb_friction", "offset")


def nominal_parameter_scales(theta_nominal: np.ndarray) -> np.ndarray:
    """Characteristic scales of the 78 parameters, without amplifying zero columns.

    The length scale of each link is ``sqrt(trace(I_com) / (2 m))`` so that
    translational, rotational and friction columns become comparable before
    the truncated SVD decides the numerical rank.
    """
    scales = np.empty(78)
    for i in range(6):
        t = theta_nominal[10 * i : 10 * (i + 1)]
        m = float(t[0])
        h = t[1:4]
        inertia = np.array(
            [
                [t[4], t[7], t[8]],
                [t[7], t[5], t[9]],
                [t[8], t[9], t[6]],
            ]
        )
        inertia_com = inertia - ((h @ h) * np.eye(3) - np.outer(h, h)) / m
        ell = float(np.sqrt(max(np.trace(inertia_com) / (2 * m), 1e-10)))
        scales[10 * i : 10 * (i + 1)] = [m, *([m * ell] * 3), *([m * ell * ell] * 6)]
    scales[60:] = np.tile([1.0, 0.5, 1.0], 6)
    return scales


def make_friction(qd: np.ndarray) -> np.ndarray:
    """Six joints, each with viscous, smooth Coulomb and offset columns.

    Returns:
        Array of shape ``(N, 6, 18)``.
    """
    n = len(qd)
    features = np.zeros((n, 6, 18))
    for joint in range(6):
        features[:, joint, 3 * joint : 3 * joint + 3] = np.column_stack(
            [
                qd[:, joint],
                np.tanh(qd[:, joint] / 0.02),
                np.ones(n),
            ]
        )
    return features


def metrics(pred: np.ndarray, target: np.ndarray) -> dict:
    """Prediction metrics, both global and per joint."""
    error = pred - target
    target_rms = np.sqrt(np.mean(target**2, axis=0))
    rmse = np.sqrt(np.mean(error**2, axis=0))
    denominator = np.sum((target - target.mean(axis=0)) ** 2, axis=0)
    return {
        "rmse_Nm": rmse.tolist(),
        "target_rms_Nm": target_rms.tolist(),
        "rmse_over_target_rms": (rmse / np.maximum(target_rms, 1e-12)).tolist(),
        "r2_per_joint": (1 - np.sum(error**2, axis=0) / np.maximum(denominator, 1e-24)).tolist(),
        "relative_error_norm_ratio": float(
            np.linalg.norm(error) / max(np.linalg.norm(target), 1e-12)
        ),
        "squared_error_sum_Nm2": float(np.sum(error**2)),
    }
