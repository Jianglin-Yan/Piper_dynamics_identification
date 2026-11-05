"""Fit low-speed friction from the six friction runs and compare models.

Loads the completed J1-J6 runs, derives velocity/acceleration from position on a
uniform grid (the drive velocity is unusable at these speeds), builds the base
inertial regressor, and compares Coulomb widths and a Stribeck model.  The
inertial parameters are first fixed to revision 001 / balanced_v2 / sg21_LS so
the friction comparison is not confounded by a joint refit.
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from real_robot.identify_offline import original_modules, inertial_model  # noqa: E402

IDENT = Path('/home/shin/Documents/41Piper/identification')
REV = IDENT / 'revisions/PIPER-20261102-001'
OUT = Path('/tmp/friction_fit')
OUT.mkdir(exist_ok=True)
RUNS = {1: '20261104_170052_844284', 6: '20261104_174524_247680',
        4: '20261104_181228_158215', 5: '20261105_090806_321000',
        2: '20261105_092108_681630', 3: '20261105_093410_716869'}
KBC = np.array([4.725] * 3 + [0.7787701152] * 3)
DT = 0.01


def load_arrays(joint):
    z = np.load(ROOT / 'runs' / RUNS[joint] / 'test_signals.npz')
    t = z['t_test_s']
    grid = np.arange(t[0], t[-1] + DT * .01, DT)
    q = np.column_stack([np.interp(grid, t, z['q'][:, k]) for k in range(6)])
    cur = np.column_stack([np.interp(grid, t, z['current_A'][:, k]) for k in range(6)])
    qd = savgol_filter(q, 21, 3, deriv=1, delta=DT, axis=0)
    qdd = savgol_filter(q, 51, 3, deriv=2, delta=DT, axis=0)
    tau = cur * KBC
    # gap guard: ignore samples whose nearest raw sample is far away
    gap = np.interp(grid, t, np.r_[0., np.diff(t)])
    good = gap <= .03
    # nearest label index
    idx = np.clip(np.searchsorted(t, grid), 0, len(t) - 1)
    repeat = z['trial_repeat'][idx]
    speed = z['trial_speed_bound_rad_s'][idx]
    direction = z['trial_direction'][idx]
    return dict(grid=grid, q=q, qd=qd, qdd=qdd, tau=tau, good=good,
                repeat=repeat, speed=speed, direction=direction)


def build(joint, data, builder, idx):
    mov = data['good'] & (np.abs(data['qd'][:, joint - 1]) > .002)
    sel = np.flatnonzero(mov)[::5]  # 20 Hz
    q, v, a = data['q'][sel], data['qd'][sel], data['qdd'][sel]
    Y = builder.build(q, v, a)
    return dict(YB=Y[:, :, idx], q=q, v=v, tau=data['tau'][sel], joint=joint,
                repeat=data['repeat'][sel], speed=data['speed'][sel],
                direction=data['direction'][sel], t=data['grid'][sel])


def friction_matrix(v, width):
    return np.stack([v, np.tanh(v / width), np.ones_like(v)], axis=2)


def stribeck(v, p):
    fv, fc, fs, vs, fo = p
    return fv * v + np.tanh(v / .002) * (fc + (fs - fc) * np.exp(-(v / vs) ** 2)) + fo


def main():
    basis = json.loads((REV / 'basis.json').read_text())
    idx = np.array(basis['indices'])
    theta = np.load(REV / 'training/main_balanced_v2.npz')['sg21_LS']
    base = theta[idx]
    ds = np.load(REV / 'datasets/balanced_v2/sg21.npz')
    rg, _, _ = original_modules()
    builder = rg.RegressorBuilder(inertial_model(), link_bodies=tuple((f'link{i}',) for i in range(1, 7)))
    records = []
    for joint in (1, 6, 4, 5, 2, 3):
        data = load_arrays(joint)
        rec = build(joint, data, builder, idx)
        np.savez(OUT / f'fit_j{joint}.npz', **{k: v for k, v in rec.items() if k != 'direction'},
                 direction=rec['direction'].astype('U8'))
        records.append(rec)
        print(f'J{joint}: {len(rec["v"])} samples, '
              f'moving {np.abs(rec["v"][:, joint-1]).max():.3f} rad/s', flush=True)

    # fixed-base friction fit per joint, width comparison
    report = {}
    for w in (.02, .005, .002):
        rep = {}
        for joint in (1, 6, 4, 5, 2, 3):
            rec = next(r for r in records if r['joint'] == joint)
            k = joint - 1
            resid = rec['tau'][:, k] - rec['YB'][:, k, :] @ base
            v = rec['v'][:, k]
            A = np.column_stack([v, np.tanh(v / w), np.ones_like(v)])
            p, *_ = np.linalg.lstsq(A, resid, rcond=None)
            pred = A @ p
            rep[joint] = dict(fv=float(p[0]), fc=float(p[1]), fo=float(p[2]),
                              rmse=float(np.sqrt(np.mean((pred - resid) ** 2))),
                              measured_rms=float(np.sqrt(np.mean(resid ** 2))))
        report[f'w{w}'] = rep
    # Stribeck per joint
    st = {}
    for joint in (1, 6, 4, 5, 2, 3):
        rec = next(r for r in records if r['joint'] == joint)
        k = joint - 1
        resid = rec['tau'][:, k] - rec['YB'][:, k, :] @ base
        v = rec['v'][:, k]
        p0 = [.1, .1, .2, .01, 0.]
        lower = [-10, 0, 0, 1e-4, -.5]
        upper = [10, 2, 2, .5, .5]
        r = least_squares(lambda p: stribeck(v, p) - resid, p0, bounds=(lower, upper), loss='soft_l1', f_scale=.05)
        pred = stribeck(v, r.x)
        st[joint] = dict(params=r.x.tolist(), rmse=float(np.sqrt(np.mean((pred - resid) ** 2))))
    report['stribeck'] = st
    (OUT / 'report.json').write_text(json.dumps(report, indent=2))
    for name in ('w0.02', 'w0.005', 'w0.002'):
        print(name, {j: round(report[name][j]['rmse'], 4) for j in (1, 2, 3, 4, 5, 6)})
    print('stribeck', {j: round(st[j]['rmse'], 4) for j in (1, 2, 3, 4, 5, 6)})
    print('stribeck params', {j: [round(x, 4) for x in st[j]['params']] for j in (1, 2, 3, 4, 5, 6)})


if __name__ == '__main__':
    main()
