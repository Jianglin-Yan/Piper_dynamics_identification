#!/usr/bin/env python3
"""Build the 60-column rigid-body regressor Y for one compact dataset file.

Y has shape (n_samples, 6, 60) and maps the 60 standard inertial parameters
(link1..link6, each [m,hx,hy,hz,Ixx,Iyy,Izz,Ixy,Ixz,Iyz]) to six joint torques:

    tau = Y(q, qd, qdd) @ theta

The 18 friction/bias columns are added separately:
    per joint: qd, tanh(qd/0.02), 1
so the full regressor is 78 columns; friction parameters are not part of Y.

Usage:
    python build_regressor.py old_fast_0/sg41.npz
    python build_regressor.py old_fast_0/sg41.npz --out /tmp/Y.npz
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("dataset", type=Path, help="compact npz containing q, qd, qdd")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()
    root = Path(__file__).resolve().parent
    sys.path.insert(0, str(root / "model"))
    import mujoco
    import regressor

    model = mujoco.MjModel.from_xml_path(str(root / "model/piper.xml"))
    model.body_gravcomp[:] = 0
    model.dof_damping[:] = 0
    model.dof_armature[:] = 0
    model.dof_frictionloss[:] = 0
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    builder = regressor.RegressorBuilder(
        model, link_bodies=tuple((f"link{i}",) for i in range(1, 7)))

    z = np.load(args.dataset)
    q, qd, qdd = z["q"], z["qd"], z["qdd"]
    Y = builder.build(q, qd, qdd, progress=True)
    out = args.out or args.dataset.with_name(args.dataset.stem + "_Y.npz")
    np.savez_compressed(out, Y=Y, q=q, qd=qd, qdd=qdd,
                        t=z["t"], cycle=z["cycle"], target_Nm=z["target_Nm"])
    print(f"wrote {out} Y={Y.shape}")


if __name__ == "__main__":
    main()
