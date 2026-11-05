"""Configured offline friction analysis, retaining the original estimator."""
import json
import numpy as np
from scipy.signal import savgol_filter
from scipy.optimize import least_squares
from piper_dynamics.core.gravity_model import GravityModel
from piper_dynamics.pipeline.measured_run import original_modules, inertial_model
from piper_dynamics.pipeline.friction_paths import parse_paths
WIDTH = 0.005

def fit_joint(config, G, joint, result, theta):
    """Fit and record one joint using the existing plateau and loss definitions."""
    z = np.load(config.source / f'fit_j{joint}.npz')
    k = joint - 1
    g = np.array([G(x) for x in z['q']])
    r = z['tau'][:, k] - g[:, k]
    v = z['v'][:, k]
    speed = z['speed']
    direction = z['direction'].astype(str)
    repeat = z['repeat']
    groups = {}
    for sp in sorted(set(np.round(speed, 4))):
        for dr in sorted(set(direction)):
            for rp in sorted(set(repeat)):
                m = (np.round(speed, 4) == sp) & (direction == dr) & (repeat == rp)
                if m.sum() < 20:
                    continue
                mm = m & (np.abs(v) >= 0.5 * np.abs(v[m]).max())
                groups[sp, dr, rp] = (float(v[mm].mean()), float(r[mm].mean()), int(mm.sum()))
    pts = np.array(list(groups.values()))
    res = least_squares(lambda p: model(pts[:, 0], p) - pts[:, 1], [0.1, 0.3, 0.0], bounds=([0, 0, -0.5], [10, 2, 0.5]), loss='soft_l1', f_scale=0.03)
    p = res.x
    rmse = float(np.sqrt(np.mean((model(pts[:, 0], p) - pts[:, 1]) ** 2)))
    diffs = []
    for sp in sorted(set(np.round(speed, 4))):
        for dr in sorted(set(direction)):
            a = groups.get((sp, dr, 1))
            b = groups.get((sp, dr, 2))
            if a and b:
                diffs.append(abs(a[1] - b[1]))
    result['joints'][joint] = dict(fv=float(p[0]), fc=float(p[1]), fo=float(p[2]), rmse=rmse, points=[[float(x) for x in row] for row in pts])
    result['repeats'][joint] = dict(max_abs_diff=float(max(diffs)) if diffs else None, mean_abs_diff=float(np.mean(diffs)) if diffs else None, n_pairs=len(diffs))
    theta[60 + 3 * k:63 + 3 * k] = p
    print(f'J{joint}: fv={p[0]:.3f} fc={p[1]:.3f} fo={p[2]:.3f} rmse={rmse:.4f} repeat max/mean={max(diffs):.3f}/{np.mean(diffs):.3f} Nm')

def model(v, p):
    return p[0] * v + p[1] * np.tanh(v / WIDTH) + p[2]

def fit(config):
    G = GravityModel({'gravity_model': 'nominal', 'gravity_fit': 'sg21_LS'})
    old = np.load(config.revision / 'training/main_balanced_v2.npz')['sg21_LS']
    theta = old.copy()
    result = {'width': WIDTH, 'joints': {}, 'repeats': {}}
    for joint in (1, 6, 4, 5, 2, 3):
        fit_joint(config, G, joint, result, theta)
    np.savez(config.output / 'theta_new.npz', theta=theta, width=WIDTH, base=old[:60], friction=theta[60:].reshape(6, 3))
    (config.output / 'final_report.json').write_text(json.dumps(result, indent=2))
    (config.output / 'theta_new.json').write_text(json.dumps({'width': WIDTH, 'inertial_theta_60': old[:60].tolist(), 'friction_fv_fc_fo': theta[60:].reshape(6, 3).tolist()}, indent=2))

def main(argv=None):
    """Run this analysis with explicit acquisition, revision and output selection."""
    fit(parse_paths(__doc__,argv))

if __name__=='__main__':main()
