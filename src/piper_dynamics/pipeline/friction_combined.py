"""Configured offline friction analysis, retaining the original estimator."""

import json

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

from piper_dynamics.pipeline.friction_paths import parse_paths
from piper_dynamics.pipeline.measured_run import inertial_model, original_modules

KBC = np.array([4.725] * 3 + [0.7787701152] * 3)

DT = 0.01


def load_reference(config):
    basis = json.loads((config.revision / "basis.json").read_text())
    idx = np.array(basis["indices"])
    theta = np.load(config.revision / "training/main_balanced_v2.npz")["sg21_LS"]
    base = theta[idx]
    rg, _, _ = original_modules()
    builder = rg.RegressorBuilder(
        inertial_model(), link_bodies=tuple(((f"link{i}",) for i in range(1, 7)))
    )
    return (builder, idx, base)


def collect_records(config, builder, idx):
    records = []
    for joint in (1, 6, 4, 5, 2, 3):
        data = load_arrays(joint, config)
        rec = build(joint, data, builder, idx)
        np.savez(
            config.output / f"fit_j{joint}.npz",
            **{k: v for k, v in rec.items() if k != "direction"},
            direction=rec["direction"].astype("U8"),
        )
        records.append(rec)
        print(
            f"J{joint}: {len(rec['v'])} samples, moving {np.abs(rec['v'][:, joint - 1]).max():.3f} rad/s",
            flush=True,
        )
    return records


def fit_models(records, base):
    report = {}
    for w in (0.02, 0.005, 0.002):
        rep = {}
        for joint in (1, 6, 4, 5, 2, 3):
            rec = next((r for r in records if r["joint"] == joint))
            k = joint - 1
            resid = rec["tau"][:, k] - rec["YB"][:, k, :] @ base
            v = rec["v"][:, k]
            A = np.column_stack([v, np.tanh(v / w), np.ones_like(v)])
            p, *_ = np.linalg.lstsq(A, resid, rcond=None)
            pred = A @ p
            rep[joint] = dict(
                fv=float(p[0]),
                fc=float(p[1]),
                fo=float(p[2]),
                rmse=float(np.sqrt(np.mean((pred - resid) ** 2))),
                measured_rms=float(np.sqrt(np.mean(resid**2))),
            )
        report[f"w{w}"] = rep
    st = {}
    for joint in (1, 6, 4, 5, 2, 3):
        rec = next((r for r in records if r["joint"] == joint))
        k = joint - 1
        resid = rec["tau"][:, k] - rec["YB"][:, k, :] @ base
        v = rec["v"][:, k]
        p0 = [0.1, 0.1, 0.2, 0.01, 0.0]
        lower = [-10, 0, 0, 0.0001, -0.5]
        upper = [10, 2, 2, 0.5, 0.5]
        r = least_squares(
            lambda p: stribeck(v, p) - resid,
            p0,
            bounds=(lower, upper),
            loss="soft_l1",
            f_scale=0.05,
        )
        pred = stribeck(v, r.x)
        st[joint] = dict(params=r.x.tolist(), rmse=float(np.sqrt(np.mean((pred - resid) ** 2))))
    report["stribeck"] = st
    return (report, st)


def load_arrays(joint, config):
    z = np.load(config.run_root / config.runs[joint] / "test_signals.npz")
    t = z["t_test_s"]
    grid = np.arange(t[0], t[-1] + DT * 0.01, DT)
    q = np.column_stack([np.interp(grid, t, z["q"][:, k]) for k in range(6)])
    cur = np.column_stack([np.interp(grid, t, z["current_A"][:, k]) for k in range(6)])
    qd = savgol_filter(q, 21, 3, deriv=1, delta=DT, axis=0)
    qdd = savgol_filter(q, 51, 3, deriv=2, delta=DT, axis=0)
    tau = cur * KBC
    gap = np.interp(grid, t, np.r_[0.0, np.diff(t)])
    good = gap <= 0.03
    idx = np.clip(np.searchsorted(t, grid), 0, len(t) - 1)
    repeat = z["trial_repeat"][idx]
    speed = z["trial_speed_bound_rad_s"][idx]
    direction = z["trial_direction"][idx]
    return dict(
        grid=grid,
        q=q,
        qd=qd,
        qdd=qdd,
        tau=tau,
        good=good,
        repeat=repeat,
        speed=speed,
        direction=direction,
    )


def build(joint, data, builder, idx):
    mov = data["good"] & (np.abs(data["qd"][:, joint - 1]) > 0.002)
    sel = np.flatnonzero(mov)[::5]
    q, v, a = (data["q"][sel], data["qd"][sel], data["qdd"][sel])
    Y = builder.build(q, v, a)
    return dict(
        YB=Y[:, :, idx],
        q=q,
        v=v,
        tau=data["tau"][sel],
        joint=joint,
        repeat=data["repeat"][sel],
        speed=data["speed"][sel],
        direction=data["direction"][sel],
        t=data["grid"][sel],
    )


def friction_matrix(v, width):
    return np.stack([v, np.tanh(v / width), np.ones_like(v)], axis=2)


def stribeck(v, p):
    fv, fc, fs, vs, fo = p
    return fv * v + np.tanh(v / 0.002) * (fc + (fs - fc) * np.exp(-((v / vs) ** 2))) + fo


def fit(config):
    builder, idx, base = load_reference(config)
    records = collect_records(config, builder, idx)
    report, st = fit_models(records, base)
    (config.output / "report.json").write_text(json.dumps(report, indent=2))
    for name in ("w0.02", "w0.005", "w0.002"):
        print(name, {j: round(report[name][j]["rmse"], 4) for j in (1, 2, 3, 4, 5, 6)})
    print("stribeck", {j: round(st[j]["rmse"], 4) for j in (1, 2, 3, 4, 5, 6)})
    print(
        "stribeck params", {j: [round(x, 4) for x in st[j]["params"]] for j in (1, 2, 3, 4, 5, 6)}
    )


def main(argv=None):
    """Run this analysis with explicit acquisition, revision and output selection."""
    fit(parse_paths(__doc__, argv))


if __name__ == "__main__":
    main()
