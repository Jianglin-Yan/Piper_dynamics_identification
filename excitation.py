"""Persistently exciting joint trajectories (Swevers et al. multi-sine).

Each joint follows

    q_j(t) = q0_j + sum_k [ a_jk * sin(k*w*t) + b_jk * cos(k*w*t) ]

with ``q0_j`` the mid-range position, ``w = 2*pi*base_freq`` and ``harmonics``
harmonics per joint.  Derivatives are analytic, so the reference acceleration
is exact (no finite differences) which matters because the regressor is driven
by ``qdd``.

The coefficients are drawn at random but normalised so that

    sum_k (|a_jk| + |b_jk|) <= amplitude_frac * (hi_j - lo_j) / 2

which guarantees the trajectory stays inside the joint limits (worst case is
when every harmonic peaks simultaneously).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass
class Excitation:
    """Sampled excitation trajectory."""

    t: np.ndarray        # (N,)    time [s]
    q: np.ndarray        # (N, 6)  position [rad]
    qd: np.ndarray       # (N, 6)  velocity [rad/s]
    qdd: np.ndarray      # (N, 6)  acceleration [rad/s^2]
    q0: np.ndarray       # (6,)    mid-range offsets
    joint_names: Sequence[str]
    base_freq: float
    harmonics: int

    def __len__(self) -> int:  # pragma: no cover - convenience only
        return int(self.t.shape[0])


def swevers_trajectory(
    joint_lo: Sequence[float],
    joint_hi: Sequence[float],
    duration: float = 60.0,
    dt: float = 0.002,
    base_freq: float = 0.2,
    harmonics: int = 5,
    amplitude_frac: float = 0.6,
    seed: int = 0,
    joint_names: Sequence[str] = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6"),
) -> Excitation:
    """Build a multi-sine excitation trajectory.

    Parameters
    ----------
    joint_lo, joint_hi:
        Joint limits in radians (length 6).
    duration, dt:
        Trajectory length and sampling period (500 Hz by default).
    base_freq:
        Base frequency in Hz; harmonic ``k`` sits at ``k * base_freq``.
    harmonics:
        Number of harmonics per joint.
    amplitude_frac:
        Fraction of each joint's half-range used as the *total* amplitude.
    seed:
        RNG seed, so runs are reproducible.
    """
    lo = np.asarray(joint_lo, dtype=float)
    hi = np.asarray(joint_hi, dtype=float)
    if lo.shape != hi.shape or lo.ndim != 1:
        raise ValueError("joint_lo / joint_hi must be 1-D of equal length")
    nj = lo.size
    if harmonics < 1:
        raise ValueError("harmonics must be >= 1")
    if duration <= 0 or dt <= 0:
        raise ValueError("duration and dt must be positive")

    rng = np.random.default_rng(seed)
    w = 2.0 * np.pi * base_freq
    q0 = 0.5 * (lo + hi)
    half_range = 0.5 * (hi - lo)

    a = rng.uniform(-1.0, 1.0, size=(nj, harmonics))
    b = rng.uniform(-1.0, 1.0, size=(nj, harmonics))
    # Normalise so the worst-case peak equals amplitude_frac * half_range.
    peak = np.abs(a).sum(axis=1) + np.abs(b).sum(axis=1)
    budget = amplitude_frac * half_range
    scale = np.where(peak > 0, budget / np.maximum(peak, 1e-9), 0.0)
    a *= scale[:, None]
    b *= scale[:, None]

    n = int(round(duration / dt)) + 1
    t = np.arange(n) * dt

    k = np.arange(1, harmonics + 1)[None, :, None]        # (1,K,1)
    phase = k * w * t[None, None, :]                       # (1,K,N)
    sin_ph = np.sin(phase)                                 # (1,K,N)
    cos_ph = np.cos(phase)

    a3 = a[:, :, None]                                     # (J,K,1)
    b3 = b[:, :, None]
    kw = (k * w)                                           # (1,K,1)

    q = q0[:, None] + np.sum(a3 * sin_ph + b3 * cos_ph, axis=1)
    qd = np.sum(a3 * kw * cos_ph - b3 * kw * sin_ph, axis=1)
    qdd = np.sum(-a3 * kw**2 * sin_ph - b3 * kw**2 * cos_ph, axis=1)

    # Numerical safety: clip tiny overshoots caused by float round-off.
    eps = 1e-9
    q = np.clip(q, lo[:, None] + eps, hi[:, None] - eps)

    return Excitation(
        t=t,
        q=q.T.copy(),
        qd=qd.T.copy(),
        qdd=qdd.T.copy(),
        q0=q0,
        joint_names=tuple(joint_names),
        base_freq=float(base_freq),
        harmonics=int(harmonics),
    )


if __name__ == "__main__":  # pragma: no cover - manual smoke test
    lo = [-2.618, 0.0, -2.967, -1.745, -1.222, -2.967]
    hi = [2.618, 3.142, 0.0, 1.745, 1.222, 2.967]
    tr = swevers_trajectory(lo, hi, duration=60.0)
    print(f"samples={len(tr.t)} dt={tr.t[1]-tr.t[0]:.4f}")
    print("q range   :", np.round(tr.q.min(0), 3), np.round(tr.q.max(0), 3))
    print("|qdd| max :", np.round(np.abs(tr.qdd).max(0), 2), "rad/s^2")
