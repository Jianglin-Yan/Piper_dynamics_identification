"""Drive the simulator along an excitation trajectory and record data.

The collector intentionally runs with ``gravity_compensation=False``: the
simulator's default feed-forward cancels the inertial forces we are trying to
measure, which would make the regressor ill-conditioned (see the Phase-0 notes
in ``simulation.py``).

Recorded signals (one row per control tick, 500 Hz):

    t, q, qd, qdd, tau_act, tau_bias, M, q_ref

* ``q/qd/qdd`` come from ``data.qpos/qvel/qacc`` of the arm DOFs,
* ``tau_act``  is ``data.qfrc_actuator`` (generalised force, N*m),
* ``tau_bias`` is ``data.qfrc_bias``  (C(q,qd)*qd + g(q)),
* ``M``        is the 6x6 joint-space inertia matrix.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from agx_piper_sim.sim.simulation import Simulation

from .excitation import Excitation, swevers_trajectory

RESULTS_DIR = Path(__file__).resolve().parent / "results"


@dataclass
class CollectionResult:
    t: np.ndarray
    q: np.ndarray
    qd: np.ndarray
    qdd: np.ndarray
    tau_act: np.ndarray
    tau_bias: np.ndarray
    M: np.ndarray
    qacc_raw: np.ndarray
    q_ref: np.ndarray
    tracking_rmse: np.ndarray
    clipped: bool

    def save(self, path: str | os.PathLike) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(
            path,
            t=self.t, q=self.q, qd=self.qd, qdd=self.qdd, qacc_raw=self.qacc_raw,
            tau_act=self.tau_act, tau_bias=self.tau_bias, M=self.M,
            q_ref=self.q_ref,
        )


def _park_objects(sim: Simulation, z: float = -5.0) -> None:
    """Move the free-floating grasp objects far below the floor.

    Identification only concerns the arm; keeping the objects on the table
    would add contact impulses whenever the excitation trajectory sweeps over
    the workspace.
    """
    for spec in sim.object_specs:
        adr = spec.qpos_adr
        sim.data.qpos[adr] = 0.0
        sim.data.qpos[adr + 1] = 0.0
        sim.data.qpos[adr + 2] = z
        sim.data.qpos[adr + 3:adr + 7] = (1.0, 0.0, 0.0, 0.0)
        sim.data.qvel[spec.dof_adr:spec.dof_adr + 6] = 0.0
    sim.mujoco.mj_forward(sim.model, sim.data)


def collect(
    excitation: Optional[Excitation] = None,
    duration: float = 60.0,
    warmup: float = 1.0,
    seed: int = 0,
    amplitude_frac: float = 0.6,
    verbose: bool = True,
) -> CollectionResult:
    """Run the excitation trajectory and return the recorded signals."""
    sim = Simulation(render=False, gravity_compensation=False)
    try:
        sim.reset(randomize=False)
        sim._trajectory = None          # make sure no interpolation is active
        _park_objects(sim)

        if excitation is None:
            excitation = swevers_trajectory(
                sim.joint_lo, sim.joint_hi, duration=duration, seed=seed,
                amplitude_frac=amplitude_frac,
            )

        tr = excitation
        n = len(tr.t)
        dt = float(tr.t[1] - tr.t[0])

        # --- warm-up: hold q0 until the servos settle --------------------
        for _ in range(int(round(warmup / dt))):
            sim.set_arm_target(tr.q[0])
            sim.step(1)

        q = np.zeros((n, 6)); qd = np.zeros((n, 6)); qacc_raw = np.zeros((n, 6))
        tau = np.zeros((n, 6)); bias = np.zeros((n, 6)); mass = np.zeros((n, 6, 6))
        clipped = False

        lo = sim.joint_lo + 1e-6
        hi = sim.joint_hi - 1e-6

        for i in range(n):
            sim.set_arm_target(tr.q[i])
            sim.step(1)
            q[i] = sim.data.qpos[sim.joint_qadr]
            qd[i] = sim.data.qvel[sim.joint_dadr]
            qacc_raw[i] = sim.data.qacc[sim.joint_dadr]
            tau[i] = sim.data.qfrc_actuator[sim.joint_dadr]
            bias[i] = sim.data.qfrc_bias[sim.joint_dadr]
            mass[i] = sim.mass_matrix()
            if not clipped and (np.any(q[i] <= lo) or np.any(q[i] >= hi)):
                clipped = True

        # MuJoCo runs with the implicitfast integrator, where ``data.qacc`` is
        # the *explicit* acceleration of the unconstrained solve and not the
        # realised d(qd)/dt (stiff joint damping makes the two differ by orders
        # of magnitude).  Differentiate qd instead.
        qdd = np.gradient(qd, dt, axis=0) if n > 2 else np.zeros_like(qd)

        tracking = np.sqrt(np.mean((q - tr.q) ** 2, axis=0))
        if verbose:
            print(f"[collect] {n} samples @ {1/dt:.0f} Hz, warmup {warmup}s")
            print(f"[collect] tracking RMSE per joint [rad]: {np.round(tracking, 4)}")
            print(f"[collect] |qdd| max (finite diff) [rad/s^2]: {np.round(np.abs(qdd).max(0), 1)}")
            print(f"[collect] |tau| max [N*m]: {np.round(np.abs(tau).max(0), 2)}")
            print(f"[collect] joint-limit clipping: {clipped}")
    finally:
        sim.close()

    return CollectionResult(
        t=tr.t.copy(), q=q, qd=qd, qdd=qdd, qacc_raw=qacc_raw, tau_act=tau, tau_bias=bias,
        M=mass, q_ref=tr.q.copy(), tracking_rmse=tracking, clipped=clipped,
    )


def main() -> None:  # pragma: no cover - manual smoke test
    import argparse

    ap = argparse.ArgumentParser(description="Collect identification data")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--warmup", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", default=str(RESULTS_DIR / "raw_data.npz"))
    args = ap.parse_args()

    res = collect(duration=args.duration, warmup=args.warmup, seed=args.seed)
    res.save(args.out)
    print(f"[collect] saved -> {args.out}")


if __name__ == "__main__":  # pragma: no cover
    main()
