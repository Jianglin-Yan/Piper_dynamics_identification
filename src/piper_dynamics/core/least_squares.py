"""Column-pivoted QR and regularised least squares used by measured-run analysis.

Extracted without changing the existing solve, prior or return semantics.
The CAD-scaled truncated-SVD workflow remains in svd_fit.py.
"""
from typing import Dict, List, Optional, Tuple
import numpy as np


def select_base_parameters(Y_stack: np.ndarray, rel_tol: float = 1e-8
                           ) -> Tuple[np.ndarray, int, np.ndarray]:
    """Column-pivoted QR on the column-normalised regressor.

    Returns ``(base_indices, rank, column_norms)``.  Columns are normalised
    before the factorisation so that parameters with different physical units
    are compared on equal footing; the norms are returned so the caller can
    rescale the solution back.

    The pivoting order comes from ``scipy.linalg.qr``.  When SciPy is
    unavailable the pivot order is built greedily by repeatedly taking the
    column with the largest residual after projecting out the columns already
    chosen, which yields the same selection.
    """
    A = np.asarray(Y_stack, dtype=float)
    norms = np.linalg.norm(A, axis=0)
    norms[norms <= 0.0] = 1.0
    A_n = A / norms[None, :]

    diag = None
    try:
        from scipy.linalg import qr as _qr

        _, R, piv = _qr(A_n, mode="economic", pivoting=True)
        diag = np.abs(np.diag(R))
        piv = np.asarray(piv, dtype=int)
    except Exception:                                        # pragma: no cover
        piv, diag = _greedy_pivots(A_n)

    if diag is None or diag.size == 0 or diag[0] <= 0.0:
        return np.zeros(0, dtype=int), 0, norms
    rank = int(np.sum(diag > rel_tol * diag[0]))
    return piv[:rank], rank, norms


def _greedy_pivots(A: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:  # pragma: no cover
    """Fallback pivot order: largest residual norm at each step."""
    m, n = A.shape
    residual = A.copy()
    chosen: List[int] = []
    diag = np.zeros(n)
    remaining = list(range(n))
    for k in range(n):
        col_norms = np.linalg.norm(residual[:, remaining], axis=0)
        j = int(np.argmax(col_norms))
        idx = remaining.pop(j)
        chosen.append(idx)
        v = residual[:, idx]
        vnorm = float(np.linalg.norm(v))
        diag[k] = vnorm
        if vnorm <= 0.0:
            continue
        u = v / vnorm
        residual = residual - np.outer(u, u @ residual)
    return np.asarray(chosen, dtype=int), diag


def solve_parameters(Y_stack: np.ndarray, tau_stack: np.ndarray,
                     ridge: float = 0.0,
                     prior: Optional[np.ndarray] = None,
                     use_qr: bool = True,
                     rel_tol: float = 1e-8) -> Dict[str, object]:
    """Least-squares (optionally QR-reduced, optionally Tikhonov-regularised).

    ``ridge`` is relative to ``||Y||_F^2 / n_params`` so it is scale free.
    ``prior`` (flat, length = n_params) is the value pulled towards; it also
    fixes the unobservable directions to physically sensible values.
    """
    A = np.asarray(Y_stack, dtype=float)
    b = np.asarray(tau_stack, dtype=float)
    n_par = A.shape[1]
    use_qr = bool(use_qr) and ridge <= 0.0

    if use_qr:
        idx, rank, norms = select_base_parameters(A, rel_tol=rel_tol)
        if idx.size == 0:
            raise RuntimeError("regressor has no independent columns")
        A_sub = A[:, idx]
        theta_sub, *_ = np.linalg.lstsq(A_sub, b, rcond=None)
        theta = np.zeros(n_par)
        theta[idx] = theta_sub
    else:
        idx, rank, norms = select_base_parameters(A, rel_tol=rel_tol)
        if ridge > 0.0:
            lam = float(ridge) * float(np.sum(A * A)) / n_par
            prior_vec = np.zeros(n_par) if prior is None else np.asarray(prior, dtype=float)
            w = np.sqrt(lam)
            A_aug = np.vstack([A, w * np.eye(n_par)])
            b_aug = np.concatenate([b, w * prior_vec])
            theta, *_ = np.linalg.lstsq(A_aug, b_aug, rcond=None)
        else:
            theta, *_ = np.linalg.lstsq(A, b, rcond=None)

    residual = A @ theta - b
    return {
        "theta": theta,
        "base_idx": idx,
        "rank": int(rank),
        "column_norms": norms,
        "residual": residual,
        "used_qr": use_qr,
    }
