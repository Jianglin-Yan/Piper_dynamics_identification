"""Bin-mean friction fit on the six low-speed runs.

Uses the per-joint records from fit_low_speed_combined.py, subtracts the nominal
URDF gravity, and fits friction to the plateau of each (speed, direction,
repeat) segment.  Plateau means suppress the quintic start/stop transients and
the drive's coarse velocity estimate.
"""
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from real_robot.gravity import GravityModel  # noqa: E402

SRC = Path('/tmp/friction_fit')
OUT = Path('/tmp/friction_fit')
KBC = np.array([4.725] * 3 + [0.7787701152] * 3)


def stribeck(v, p):
    fv, fc, fs, vs, fo = p
    return fv * v + np.tanh(v / .002) * (fc + (fs - fc) * np.exp(-(v / vs) ** 2)) + fo


def main():
    G = GravityModel({'gravity_model': 'nominal', 'gravity_fit': 'sg21_LS'})
    report = {'points': {}, 'fits': {}}
    for joint in (1, 6, 4, 5, 2, 3):
        z = np.load(SRC / f'fit_j{joint}.npz')
        q, v, tau = z['q'], z['v'], z['tau']
        k = joint - 1
        g = np.array([G(x) for x in q])
        r = tau[:, k] - g[:, k]
        vj = v[:, k]
        speed = z['speed']
        direction = z['direction'].astype(str)
        repeat = z['repeat']
        points = []
        for sp in sorted(set(np.round(speed, 4))):
            for dr in sorted(set(direction)):
                for rp in sorted(set(repeat)):
                    m = (np.round(speed, 4) == sp) & (direction == dr) & (repeat == rp)
                    if m.sum() < 20:
                        continue
                    mm = m & (np.abs(vj) >= .5 * np.abs(vj[m]).max())
                    points.append((float(vj[mm].mean()), float(r[mm].mean()), int(mm.sum())))
        points = np.array(points)
        report['points'][joint] = points.tolist()
        out = {}
        for name, width in (('w0.02', .02), ('w0.005', .005), ('w0.002', .002)):
            A = np.column_stack([points[:, 0], np.tanh(points[:, 0] / width), np.ones(len(points))])
            w = np.sqrt(points[:, 2])
            A2 = A * w[:, None]
            y2 = points[:, 1] * w
            p, *_ = np.linalg.lstsq(A2, y2, rcond=None)
            rmse = float(np.sqrt(np.mean((A @ p - points[:, 1]) ** 2)))
            out[name] = dict(fv=float(p[0]), fc=float(p[1]), fo=float(p[2]), rmse=rmse)
        # Stribeck on the segment points
        p0 = [.1, .1, .3, .01, 0.]
        res = least_squares(lambda p: stribeck(points[:, 0], p) - points[:, 1], p0,
                            bounds=([-10, 0, 0, 1e-4, -.5], [10, 2, 2, .5, .5]), loss='soft_l1', f_scale=.02)
        out['stribeck'] = dict(params=res.x.tolist(),
                               rmse=float(np.sqrt(np.mean((stribeck(points[:, 0], res.x) - points[:, 1]) ** 2))))
        report['fits'][joint] = out
        print(f'J{joint}: ' + ' '.join(f'{n} rmse={out[n]["rmse"]:.4f}' for n in ('w0.02', 'w0.005', 'w0.002', 'stribeck')))
        print(f'   points {len(points)}  stribeck {np.round(res.x, 4)}')
    (OUT / 'bin_report.json').write_text(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
