"""Validate identified parameters on an unseen excitation trajectory.

The parameters are identified once (:mod:`.identify`) and then applied to a
*new* trajectory that was never used for fitting -- a different RNG seed, and
by default a different duration.  Two independent checks are reported:

1. **Torque prediction** -- ``tau_pred = Y(q,qd,qdd) @ theta`` against the
   reference rigid-body torque on the validation trajectory.  This is the
   quantity the identification actually fits, so it also measures how well the
   parameter set generalises.

2. **Forward simulation** -- the identified parameters are installed back into
   a copy of the MuJoCo model and the validation trajectory is replayed with
   ``mj_inverse``.  This catches sign/convention mistakes that a purely
   algebraic check can miss: the parameters must be interpretable by the
   physics engine itself.

A pass/fail threshold is applied to the torque RMSE relative to the RMS torque
on that joint.  Results are written next to the parameter file.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

RESULTS_DIR = Path(__file__).resolve().parent / "results"


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _load_params(path: str | Path) -> Tuple[np.ndarray, Dict[str, object]]:
    d = np.load(path, allow_pickle=True)
    theta = np.asarray(d["theta"], dtype=float)
    meta = {k: d[k].tolist() if hasattr(d[k], "tolist") else d[k] for k in d.files}
    return theta, meta


def _apply_params_to_model(model, theta: np.ndarray, link_bodies,
                           gripper_open: float = 0.0) -> None:
    """Write a ``(6, 10)`` ``[m, h, I_origin]`` parameter set into ``model``.

    Every body of a link group receives that link's parameters divided among
    its members: the owner body takes the whole lumped inertia (the other
    bodies, which are rigidly attached, are zeroed).  Forcing a single body is
    what makes the model reproduce the identified *lumped* link.
    """
    from .regressor import inertia_mat, mat_to_quat

    theta = np.asarray(theta, dtype=float).reshape(len(link_bodies), 10)
    for th, names in zip(theta, link_bodies):
        m = float(th[0])
        h = np.asarray(th[1:4], dtype=float)
        I_o = inertia_mat(th[4:10])
        if abs(m) > 1e-12:
            ipos = h / m
            I_c = I_o - (float(h @ h) / m * np.eye(3) - np.outer(h, h) / m)
        else:
            ipos = np.zeros(3)
            I_c = I_o
        w, V = np.linalg.eigh(I_c)
        if np.linalg.det(V) < 0.0:
            V[:, 0] = -V[:, 0]
        import mujoco

        for k, name in enumerate(names):
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid < 0:
                continue
            if k == 0:
                model.body_mass[bid] = m
                model.body_ipos[bid] = ipos
                model.body_inertia[bid] = w
                model.body_iquat[bid] = mat_to_quat(V)
            else:
                # Rigidly attached members are folded into the owner body.  A
                # numerically zero inertia makes the joint-space mass matrix
                # singular, so keep a negligible *non-zero* value: 1e-9 is far
                # below the smallest arm inertia (~1e-4) yet keeps the matrix
                # positive definite.
                model.body_mass[bid] = 1e-9
                model.body_ipos[bid] = 0.0
                model.body_inertia[bid] = [1e-9, 1e-9, 1e-9]
                model.body_iquat[bid] = [1.0, 0.0, 0.0, 0.0]


def _model_inverse_dynamics(scene_path, q, qd, qdd, gripper_open, model=None):
    import mujoco

    if model is None:
        model = mujoco.MjModel.from_xml_path(str(scene_path))
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    data = mujoco.MjData(model)
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"joint{i}")
           for i in range(1, 7)]
    qadr = np.array([model.jnt_qposadr[j] for j in ids])
    dadr = np.array([model.jnt_dofadr[j] for j in ids])
    for jn, val in (("joint7", gripper_open), ("joint8", -gripper_open)):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = float(val)
    out = np.zeros((q.shape[0], 6))
    for i in range(q.shape[0]):
        data.qpos[qadr] = q[i]
        data.qvel[:] = 0.0
        data.qvel[dadr] = qd[i]
        data.qacc[:] = 0.0
        data.qacc[dadr] = qdd[i]
        data.qfrc_applied[:] = 0.0
        mujoco.mj_inverse(model, data)
        out[i] = data.qfrc_inverse[dadr]
    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Validate identified parameters")
    ap.add_argument("--params", default=str(RESULTS_DIR / "identified_params.npz"))
    ap.add_argument("--duration", type=float, default=30.0,
                    help="validation trajectory length (seconds)")
    ap.add_argument("--seed", type=int, default=1234,
                    help="must differ from the identification seed")
    ap.add_argument("--stride", type=int, default=5)
    ap.add_argument("--gripper-open", type=float, default=0.0)
    ap.add_argument("--tol", type=float, default=0.10,
                    help="per-joint relative torque RMSE threshold")
    ap.add_argument("--abs-tol", type=float, default=0.30,
                    help="absolute torque RMSE floor [N*m]; the wrist joints carry "
                         "so little torque that a purely relative test is unfair")
    ap.add_argument("--source", choices=("oracle", "servo"), default=None,
                    help="defaults to the source recorded in the parameter file")
    ap.add_argument("--out", default=str(RESULTS_DIR / "validation.json"))
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args(argv)
    verbose = not args.quiet

    from agx_piper_sim.sim.simulation import DEFAULT_SCENE

    from . import regressor as rg
    from .data_collector import collect
    from .excitation import swevers_trajectory

    theta, meta = _load_params(args.params)
    source = args.source or str(meta.get("source", "oracle"))
    ident_seed = meta.get("seed", None)
    # With ``--ridge 0`` the stored vector only populates the QR base set; the
    # remaining directions are zero, so it is not a complete physical model and
    # cannot be written back into MuJoCo.
    regularised = bool(meta.get("regularised", True))

    if verbose:
        print(f"[validate] params   : {args.params}")
        print(f"[validate] source   : {source}")
        print(f"[validate] model    : {'complete parameter set' if regularised else 'base-parameter set (write-back skipped)'}")
        print(f"[validate] seed     : {args.seed} (identification seed was "
              f"{ident_seed})")
        if ident_seed is not None and int(ident_seed) == int(args.seed):
            print("[validate] WARNING: validation seed equals the identification "
                  "seed; the trajectory is not unseen")

    # -- validation data: new trajectory, new seed ------------------------
    gt_model = _fresh_model(DEFAULT_SCENE)
    jids = [gt_model.joint(f"joint{i}").id for i in range(1, 7)]
    lo = np.array([gt_model.jnt_range[j, 0] for j in jids])
    hi = np.array([gt_model.jnt_range[j, 1] for j in jids])
    traj = swevers_trajectory(lo, hi, duration=args.duration, seed=args.seed)
    res = collect(excitation=traj, seed=args.seed, verbose=False)

    stride = max(1, int(args.stride))
    sel = slice(None, None, stride)
    q, qd, qdd = res.q[sel], res.qd[sel], res.qdd[sel]

    # -- 1. regressor prediction -----------------------------------------
    model = _fresh_model(DEFAULT_SCENE)
    builder = rg.RegressorBuilder(model, gripper_open=args.gripper_open)
    Y = builder.build(q, qd, qdd, progress=False)
    Y_stack = Y.reshape(-1, Y.shape[1])
    tau_pred = (Y @ theta.reshape(-1)).reshape(-1, 6)

    tau_ref = _model_inverse_dynamics(DEFAULT_SCENE, q, qd, qdd,
                                      args.gripper_open)
    tau_servo = res.tau_act[sel]
    tau_target = tau_ref if source == "oracle" else tau_servo

    resid = tau_pred - tau_target
    rmse = np.sqrt(np.mean(resid ** 2, axis=0))
    rms = np.sqrt(np.mean(tau_target ** 2, axis=0))
    norm_rmse = rmse / np.maximum(rms, 1e-6)
    rel_err_total = float(np.linalg.norm(resid) / max(1e-12, np.linalg.norm(tau_target)))

    # -- 2. parameters installed back into MuJoCo ------------------------
    if regularised:
        rebuilt = _fresh_model(DEFAULT_SCENE)
        _apply_params_to_model(rebuilt, theta, rg.LINK_BODIES,
                               gripper_open=args.gripper_open)
        tau_mj = _model_inverse_dynamics(DEFAULT_SCENE, q, qd, qdd,
                                         args.gripper_open, model=rebuilt)
        resid_mj = tau_mj - tau_target
        rmse_mj = np.sqrt(np.mean(resid_mj ** 2, axis=0))
        norm_rmse_mj = rmse_mj / np.maximum(rms, 1e-6)
    else:
        rmse_mj = np.full(6, np.nan)
        norm_rmse_mj = np.full(6, np.nan)

    # A joint passes when its error is small either relative to its own torque
    # level or in absolute terms.
    limits = np.maximum(args.abs_tol, args.tol * rms)
    judged = rmse_mj if regularised else rmse
    ok = bool(np.all(judged <= limits))

    if verbose:
        print(f"\n[validate] samples: {q.shape[0]} "
              f"({args.duration:.0f} s, stride {stride})")
        print("\n--- torque prediction on unseen trajectory ---")
        print("%-7s %10s %10s %12s %12s %10s" %
              ("joint", "rms[N*m]", "rmse[N*m]", "norm_rmse", "rmse_mj", "limit"))
        for j in range(6):
            mj_txt = f"{rmse_mj[j]:12.4f}" if regularised else f"{'n/a':>12s}"
            print("%-7s %10.4f %10.4f %12.4f %s %10.4f" %
                  (f"joint{j + 1}", rms[j], rmse[j], norm_rmse[j], mj_txt,
                   limits[j]))
        print(f"\n[validate] overall relative torque error: {100 * rel_err_total:.3f} %")
        if regularised:
            print("[validate] rmse_mj is the error after the identified parameters "
                  "are written back into a MuJoCo model")
        else:
            print("[validate] base-parameter set: judged on regressor prediction only")
        print(f"[validate] result: {'PASS' if ok else 'FAIL'}")
        if source == "servo":
            print("[validate] note: the servo target carries finite-difference "
                  "acceleration error, so this is a lower bound on the true "
                  "prediction quality.")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "params": str(args.params),
        "source": source,
        "seed": args.seed,
        "duration": args.duration,
        "n_samples": int(q.shape[0]),
        "theta_norm": float(np.linalg.norm(theta)),
        "rel_err_total": rel_err_total,
        "rmse_per_joint": [float(v) for v in rmse],
        "rms_tau_per_joint": [float(v) for v in rms],
        "norm_rmse_per_joint": [float(v) for v in norm_rmse],
        "rmse_mj_per_joint": [float(v) for v in rmse_mj],
        "norm_rmse_mj_per_joint": [float(v) for v in norm_rmse_mj],
        "tol": args.tol,
        "abs_tol": args.abs_tol,
        "limits": [float(v) for v in limits],
        "regularised": regularised,
        "passed": ok,
    }
    out.write_text(json.dumps(payload, indent=2))
    if verbose:
        print(f"[validate] saved -> {out}")

    return 0 if ok else 1


def _fresh_model(scene_path):
    import mujoco

    return mujoco.MjModel.from_xml_path(str(scene_path))


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
