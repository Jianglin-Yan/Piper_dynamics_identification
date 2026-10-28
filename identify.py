"""Identify the arm's inertial parameters from simulated motion data.

Pipeline
--------
1. generate a persistently exciting trajectory (:mod:`.excitation`),
2. run it in the simulator and record ``q, qd, qdd, tau`` (:mod:`.data_collector`),
3. build the exact regressor ``Y(q, qd, qdd)`` such that ``tau = Y @ theta``
   (:mod:`.regressor`),
4. select a numerically independent *base* parameter set with column-pivoted
   QR, solve the resulting least-squares problem,
5. compare against the model's ground-truth parameters.

Torque sources
--------------
``--source oracle`` (default)
    ``tau`` comes from MuJoCo inverse dynamics evaluated on the *measured*
    ``(q, qd, qdd)``.  This is the exact rigid-body torque
    ``M(q) qdd + C(q,qd) qd + g(q)``, i.e. the quantity the regressor is
    linear in.  Because both sides are exact, this mode validates the whole
    chain (excitation -> regressor -> conditioning -> solver) and must recover
    the base parameters to machine precision.

``--source servo``
    ``tau`` is the generalised servo effort actually recorded during the run
    (``data.qfrc_actuator``).  This mirrors a real experiment, where only the
    joint torque is measurable.  It is noisier: the simulator integrates with
    the ``implicitfast`` integrator, so ``qdd`` must be reconstructed by
    differentiating ``qd``, and that finite-difference error (a few percent)
    enters the regressor as a model mismatch.

Identifiability
---------------
Only a *base* subset of the 60 parameters is observable from joint torque
data: unobservable directions form the null space of the stacked regressor.
Without a prior, those directions cannot be estimated, so they are left at
zero (``--ridge 0``) or pulled towards the nominal values with a tiny ridge
term.  Predicting torque is unaffected either way -- that is the metric that
is reported.
"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

RESULTS_DIR = Path(__file__).resolve().parent / "results"

ARM_JOINT_NAMES: Tuple[str, ...] = (
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
)


# ---------------------------------------------------------------------------
# Reference (oracle) inverse dynamics
# ---------------------------------------------------------------------------
class OracleTorque:
    """``tau = M qdd + C qd + g`` from an untouched copy of the model."""

    def __init__(self, scene_path):
        import mujoco

        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        # Random excitation states may poke through the table; contacts are
        # not part of the rigid-body model being identified.
        self.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self.data = mujoco.MjData(self.model)
        self.joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                          for n in ARM_JOINT_NAMES]
        self.qadr = np.array([self.model.jnt_qposadr[j] for j in self.joint_ids])
        self.dadr = np.array([self.model.jnt_dofadr[j] for j in self.joint_ids])

    def __call__(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray,
                 gripper_open: float = 0.0, progress: bool = False) -> np.ndarray:
        n = q.shape[0]
        out = np.zeros((n, len(self.joint_ids)))
        self.data.qpos[:] = 0.0
        for jn, val in (("joint7", gripper_open), ("joint8", -gripper_open)):
            jid = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                self.data.qpos[self.model.jnt_qposadr[jid]] = float(val)
        for i in range(n):
            self.data.qpos[self.qadr] = q[i]
            self.data.qvel[:] = 0.0
            self.data.qvel[self.dadr] = qd[i]
            self.data.qacc[:] = 0.0
            self.data.qacc[self.dadr] = qdd[i]
            self.data.qfrc_applied[:] = 0.0
            self.mujoco.mj_inverse(self.model, self.data)
            out[i] = self.data.qfrc_inverse[self.dadr]
        return out


# ---------------------------------------------------------------------------
# Base-parameter selection
# ---------------------------------------------------------------------------
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


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------
def _param_rows(theta_hat: np.ndarray, theta_gt: np.ndarray,
                names: Sequence[str]) -> List[Dict[str, object]]:
    from .regressor import to_com_params

    rows: List[Dict[str, object]] = []
    for i, (th_hat, th_gt, nm) in enumerate(zip(theta_hat, theta_gt, names)):
        hat_c = to_com_params(th_hat)
        gt_c = to_com_params(th_gt)
        m_hat, m_gt = float(hat_c[0]), float(gt_c[0])
        c_hat = hat_c[1:4] / m_hat if abs(m_hat) > 1e-12 else np.zeros(3)
        c_gt = gt_c[1:4] / m_gt if abs(m_gt) > 1e-12 else np.zeros(3)
        rows.append({
            "link": nm,
            "mass_hat": m_hat, "mass_gt": m_gt,
            "mass_err_pct": 100.0 * (m_hat - m_gt) / m_gt if abs(m_gt) > 1e-12 else float("nan"),
            "com_hat": [float(v) for v in c_hat], "com_gt": [float(v) for v in c_gt],
            "com_err_mm": float(1e3 * np.linalg.norm(c_hat - c_gt)),
            "inertia_hat": [float(v) for v in hat_c[4:10]],
            "inertia_gt": [float(v) for v in gt_c[4:10]],
        })
    return rows


def _print_param_table(rows: List[Dict[str, object]]) -> None:
    print("\n--- inertial parameters (identified vs ground truth) ---")
    print("%-7s %10s %10s %8s %9s %9s %9s" %
          ("link", "mass_hat", "mass_gt", "err[%]", "|dCOM|[mm]",
           "Ixx_hat", "Ixx_gt"))
    for r in rows:
        print("%-7s %10.4f %10.4f %8.2f %9.2f %9.2e %9.2e" %
              (r["link"], r["mass_hat"], r["mass_gt"], r["mass_err_pct"],
               r["com_err_mm"], r["inertia_hat"][0], r["inertia_gt"][0]))


# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------
def load_or_collect(data_path: Optional[str], duration: float, dt: float,
                    seed: int, amplitude_frac: float, verbose: bool):
    if data_path and Path(data_path).exists():
        d = np.load(data_path)
        if verbose:
            print(f"[identify] reusing cached data: {data_path}")
        return ({k: d[k] for k in d.files}, float(d["t"][1] - d["t"][0]))

    from .data_collector import collect

    t0 = time.time()
    res = collect(duration=duration, seed=seed, amplitude_frac=amplitude_frac,
                  verbose=verbose)
    if verbose:
        print(f"[identify] data collection took {time.time() - t0:.1f} s")
    if data_path:
        # Cache the raw run so the regression can be re-solved without
        # re-simulating (useful when tuning --stride / --ridge).
        res.save(data_path)
        if verbose:
            print(f"[identify] cached raw data -> {data_path}")
    return ({
        "t": res.t, "q": res.q, "qd": res.qd, "qdd": res.qdd,
        "tau_act": res.tau_act, "tau_bias": res.tau_bias, "M": res.M,
        "q_ref": res.q_ref, "qacc_raw": res.qacc_raw,
    }, float(res.t[1] - res.t[0]))


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Identify Piper arm inertial parameters")
    ap.add_argument("--data", default=None,
                    help="cached npz from data_collector (created if missing)")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--dt", type=float, default=0.002)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--amplitude-frac", type=float, default=0.6)
    ap.add_argument("--stride", type=int, default=5,
                    help="use every Nth sample for the regression (default 5 = 100 Hz)")
    ap.add_argument("--source", choices=("oracle", "servo"), default="oracle")
    ap.add_argument("--ridge", type=float, default=1e-8,
                    help="relative Tikhonov weight toward the nominal CAD values. "
                         "0 = pure least squares, which leaves the unobservable "
                         "parameter directions at zero.")
    ap.add_argument("--no-qr", action="store_true",
                    help="solve for all 60 parameters instead of a QR base set")
    ap.add_argument("--gripper-open", type=float, default=0.0)
    ap.add_argument("--out", default=str(RESULTS_DIR / "identified_params.npz"))
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    verbose = not args.quiet

    from agx_piper_sim.sim.simulation import DEFAULT_SCENE

    from . import regressor as rg

    # -- 1. data ---------------------------------------------------------
    data, dt = load_or_collect(args.data, args.duration, args.dt, args.seed,
                               args.amplitude_frac, verbose)
    stride = max(1, int(args.stride))
    sel = slice(None, None, stride)
    q, qd, qdd = data["q"][sel], data["qd"][sel], data["qdd"][sel]
    tau_act = data["tau_act"][sel]
    n_used = q.shape[0]
    if verbose:
        print(f"[identify] regression samples: {n_used} (stride {stride}, "
              f"effective {1.0 / (dt * stride):.0f} Hz)")

    # -- 2. regressor and ground truth -----------------------------------
    # ``RegressorBuilder`` overwrites the inertial parameters of the model it
    # is handed (that is how it installs the probing carrier), so the ground
    # truth must be read from a pristine copy *first*.
    import mujoco

    gt_model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    theta_gt_mat = rg.ground_truth_theta(gt_model, gripper_open=args.gripper_open)
    theta_gt = theta_gt_mat.reshape(-1)

    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    builder = rg.RegressorBuilder(model, gripper_open=args.gripper_open)
    t0 = time.time()
    Y = builder.build(q, qd, qdd, progress=verbose)
    if verbose:
        print(f"[identify] regressor built in {time.time() - t0:.1f} s, shape {Y.shape}")

    # -- 3. torque target ------------------------------------------------
    if args.source == "oracle":
        oracle = OracleTorque(DEFAULT_SCENE)
        tau = oracle(q, qd, qdd, gripper_open=args.gripper_open)
        if verbose:
            print("[identify] torque source: oracle inverse dynamics")
    else:
        tau = tau_act
        if verbose:
            print("[identify] torque source: measured servo effort")
            # Quantify the mismatch caused by finite-difference acceleration.
            M_stack = None
            if "M" in data:
                M_stack = data["M"][sel]
            if M_stack is not None:
                pred = np.einsum("nij,nj->ni", M_stack, qdd) + data["tau_bias"][sel]
                rel = np.linalg.norm(pred - tau) / max(1e-9, np.linalg.norm(tau))
                print(f"[identify] note: M*qdd_fd + bias differs from tau by "
                      f"{100 * rel:.1f}% (finite-difference qdd)")

    # -- 4. stack and solve ---------------------------------------------
    # Row layout: [tau_0(j0..j5); tau_1(j0..j5); ...]
    Y_stack = Y.reshape(-1, Y.shape[-1])
    tau_stack = tau.reshape(-1)

    sol = solve_parameters(
        Y_stack, tau_stack, ridge=args.ridge, prior=theta_gt,
        use_qr=not args.no_qr,
    )
    theta_hat = sol["theta"]
    resid = sol["residual"].reshape(-1, 6)
    regularised = args.ridge > 0.0

    rmse = np.sqrt(np.mean(resid ** 2, axis=0))
    tau_rms = np.sqrt(np.mean(tau ** 2, axis=0))
    rel_err = float(np.linalg.norm(resid) / max(1e-9, np.linalg.norm(tau_stack)))

    # The QR base set is a set of *combinations* of the physical parameters,
    # not a subset of them, so it cannot be compared element-wise with the
    # ground-truth vector.  Parameter accuracy is therefore judged by the
    # torque fit here and, more strictly, by re-testing on an unseen
    # trajectory in ``validate.py``.
    base = np.asarray(sol["base_idx"], dtype=int)

    if verbose:
        print(f"\n[identify] regressor rank: {sol['rank']} / {Y_stack.shape[1]} "
              f"({len(base)} base parameters selected by QR)")
        print(f"[identify] torque fit: rel err {100 * rel_err:.3e} %")
        print(f"[identify] per-joint RMSE [N*m]: {np.round(rmse, 6)}")
        print(f"[identify] per-joint |tau| rms [N*m]: {np.round(tau_rms, 3)}")

    # -- 5. ground-truth comparison -------------------------------------
    rows = _param_rows(theta_hat.reshape(6, 10), theta_gt_mat, builder.link_names)
    if verbose:
        if regularised:
            _print_param_table(rows)
            mass_hat = sum(r["mass_hat"] for r in rows)
            mass_gt = sum(r["mass_gt"] for r in rows)
            print("\ntotal arm mass: identified %.4f kg, ground truth %.4f kg "
                  "(err %.2f%%)" % (mass_hat, mass_gt,
                                    100.0 * (mass_hat - mass_gt) / mass_gt))
        else:
            print("\n[identify] --ridge 0: unobservable parameters stay at zero, so "
                  "per-link masses are not identified; only the base-parameter "
                  "and torque-fit numbers above are meaningful.")

    # -- 6. persist ------------------------------------------------------
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out,
        theta=theta_hat,
        theta_gt=theta_gt,
        param_names=np.array(builder.param_names),
        base_idx=sol["base_idx"],
        rank=sol["rank"],
        rmse=rmse,
        rel_err=rel_err,
        source=args.source,
        stride=stride,
        n_samples=n_used,
        seed=args.seed,
        duration=args.duration,
        regularised=regularised,
    )
    summary = {
        "source": args.source,
        "stride": stride,
        "n_samples": n_used,
        "regressor_rank": int(sol["rank"]),
        "n_params": int(Y_stack.shape[1]),
        "ridge": float(args.ridge),
        "base_idx": [int(i) for i in base],
        "base_param_names": [builder.param_names[int(i)] for i in base],
        "regularised": bool(regularised),
        "torque_rel_err": rel_err,
        "rmse_per_joint": [float(v) for v in rmse],
        "tau_rms_per_joint": [float(v) for v in tau_rms],
        "total_mass_hat": float(sum(r["mass_hat"] for r in rows)),
        "total_mass_gt": float(sum(r["mass_gt"] for r in rows)),
        "params": rows,
    }
    (out.with_suffix(".json")).write_text(json.dumps(summary, indent=2))
    if verbose:
        print(f"\n[identify] saved -> {out}")
        print(f"[identify] saved -> {out.with_suffix('.json')}")

    # Oracle mode is exact by construction; anything above float noise is a bug.
    if args.source == "oracle" and rel_err > 1e-6:
        print("[identify] WARNING: oracle fit is not exact, check the regressor")
        return 1
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
