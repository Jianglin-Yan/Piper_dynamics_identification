"""Offline Fourier search. Fixed 54-column basis; never opens CAN or a gamepad.

Run with the project's Python environment; Pinocchio is an offline dependency.
Random screening and SciPy L-BFGS-B refine the same explicitly documented score.
"""
import argparse
import datetime
from zoneinfo import ZoneInfo
import hashlib
import json
import re
from pathlib import Path
import numpy as np
from scipy.optimize import minimize
from .designed_trajectory import coefficients, motion, motion_identity

BASE = Path(__file__).resolve().parent
REPO = BASE.parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Designer:
    def __init__(self, base_file):
        import pinocchio as pin
        self.pin = pin
        self.pm = pin.buildModelFromUrdf(str(BASE/'assets/piper.urdf'))
        self.pd = self.pm.createData()
        mapping = json.loads(Path(base_file).read_text())['mapping']
        self.indices = np.asarray(mapping['indices'], int)
        if len(self.indices) != 36:
            raise ValueError('Expected the audited common 36-column inertia basis')
        perm = np.array([0, 1, 2, 3, 4, 7, 5, 8, 9, 6])
        self.pin_columns = np.concatenate([10*i+np.argsort(perm) for i in range(6)])[self.indices]
        self.anchor = np.deg2rad([0, 45, -60, 0, 0, 0])
        cfg = json.loads((BASE/'multisine_profile.json').read_text())
        self.ab0 = coefficients(0, np.deg2rad(cfg['multisine_generation_lower_deg']),
                                np.deg2rad(cfg['multisine_generation_upper_deg']))
        # Exactly one full steady cycle, identical sample count for every candidate.
        self.set_execution(.5,.5)
        self.reference_caps = self.peaks(self.ab0, np.arange(0., self.duration+.0001, .002))
        self.fixed_scales = None
        _, self.baseline = self.evaluate(self.ab0)
        self.evaluations = 0

    def set_execution(self, scale, speed):
        self.scale=float(scale);self.speed=float(speed);self.duration=60/self.speed
        self.t_info=10/self.speed+np.arange(0,5/self.speed,.02)
        self.t_constraints=np.linspace(0,self.duration,int(np.ceil(self.duration/.025))+1)

    def design(self, t, ab):
        return motion(t, ab, self.anchor,scale=self.scale,speed=self.speed)

    def peaks(self, ab, t):
        q, v, a, j = self.design(t, ab)
        return np.array([np.abs(q-self.anchor).max(0), np.abs(v).max(0),
                         np.abs(a).max(0), np.abs(j).max(0)])

    def project(self, ab, caps):
        peaks = self.peaks(ab, self.t_constraints)
        # All four derivatives are linear in each joint's coefficients. One
        # joint-specific back-off enforces every peak without clipping the path.
        coefficient_budget=np.abs(self.ab0).sum((0,2))
        backoff = np.minimum(np.minimum(1., np.min(caps/np.maximum(peaks, 1e-12), axis=0)),
                             coefficient_budget/np.maximum(np.abs(ab).sum((0,2)),1e-12))
        return np.asarray(ab)*backoff[None, :, None]*.999

    def matrix(self, q, v, a):
        Y = np.array([self.pin.computeJointTorqueRegressor(self.pm, self.pd, qi, vi, ai)
                      .copy()[:, self.pin_columns] for qi, vi, ai in zip(q, v, a)])
        F = np.zeros((len(q), 6, 18))
        for axis in range(6):
            F[:, axis, 3*axis:3*axis+3] = np.column_stack(
                [v[:, axis], np.tanh(v[:, axis]/.02), np.ones(len(q))])
        return np.concatenate([Y, F], axis=2).reshape(-1, 54)

    def evaluate(self, ab):
        q, v, a, _ = self.design(self.t_info, ab)
        A = self.matrix(q, v, a)
        if self.fixed_scales is None:
            self.fixed_scales = np.linalg.norm(A, axis=0)/np.sqrt(len(A))
        S = np.linalg.svd(A/self.fixed_scales/np.sqrt(len(A)), compute_uv=False)
        rank = int(np.sum(S > S[0]*1e-8))
        log_info = float(2*np.log(np.maximum(S, 1e-30)).mean())
        metrics = dict(rank=rank, condition_fixed_scale=float(S[0]/S[-1]),
                       sigma_min_per_sample=float(S[-1]), log_D_information_per_sample=log_info)
        if not hasattr(self, 'baseline'):
            return 0., metrics
        b = self.baseline
        metrics['minimum_information_relative'] = float((S[-1]/b['sigma_min_per_sample'])**2)
        metrics['D_information_relative'] = float(np.exp(log_info-b['log_D_information_per_sample']))
        score = (np.log(metrics['condition_fixed_scale']/b['condition_fixed_scale'])
                 -.5*np.log(S[-1]/b['sigma_min_per_sample'])
                 -.5*(log_info-b['log_D_information_per_sample']))
        if rank != 54:
            score += 1000*(54-rank)
        metrics['score_lower_better'] = float(score)
        return float(score), metrics

    def geometry(self, ab, step=.1):
        """Explicit distances in the existing simplified MuJoCo preview geometry.

        These proxies do not include actual meshes, cables or the workbench.
        Adjacent bodies are excluded; floor is z=0 in this nominal model.
        """
        import mujoco
        from .simulation import model_xml
        model = mujoco.MjModel.from_xml_string(model_xml())
        data = mujoco.MjData(model)
        pairs = []
        for i in range(1, model.ngeom):
            bi = int(model.geom_bodyid[i])
            for k in range(i+1, model.ngeom):
                bk = int(model.geom_bodyid[k])
                if bi == bk or model.body_parentid[bi] == bk or model.body_parentid[bk] == bi:
                    continue
                # Two deliberately oversized joint proxies overlap at their
                # shared shoulder/wrist assembly for every joint configuration.
                # Exclude only those primitive pairs; keep the moving capsules
                # and gripper boxes in the distance check.
                if (i, k) in ((1, 3), (8, 11)):
                    continue
                pairs.append((i, k))
        minimum = 10.
        min_tip_z = 10.
        floor_clearance = 10.
        witness = None
        buffer = np.zeros(6)
        for t, q in zip(np.arange(0, self.duration+step/2, step), self.design(np.arange(0, self.duration+step/2, step), ab)[0]):
            data.qpos[:] = q
            mujoco.mj_forward(model, data)
            min_tip_z = min(min_tip_z, float(data.site_xpos[0, 2]))
            for i in range(2,model.ngeom):
                floor_clearance=min(floor_clearance,float(mujoco.mj_geomDistance(model,data,0,i,10.,buffer)))
            for i, k in pairs:
                distance = mujoco.mj_geomDistance(model, data, i, k, 10., buffer)
                if distance < minimum:
                    minimum = float(distance)
                    witness = dict(t_s=float(t), geom_ids=[i, k])
        return dict(simplified_nonadjacent_geom_min_distance_m=minimum, witness=witness,
                    minimum_tip_z_m=min_tip_z, moving_geom_min_floor_clearance_m=floor_clearance, sample_step_s=step,
                    excluded_shared_joint_geom_pairs=[[1,3],[8,11]],
                    scope='nominal simplified geometry only; not full-mesh/environment collision certification')

    def report(self, ab):
        _, metrics = self.evaluate(ab)
        p = self.peaks(ab, np.arange(0., self.duration+.0001, .002))
        t = np.arange(0., self.duration+.0001, .002)
        q, v, _, _ = self.design(t, ab)
        stop_time = np.maximum(.5, 2*np.abs(v).max(1)/6.)
        stop = q+v*stop_time[:, None]/2
        lower = np.deg2rad([-145, 5, -165, -84, -65, -115])
        upper = np.deg2rad([145, 175, -5, 84, 65, 115])
        metrics.update(q_min_per_axis_deg=np.rad2deg(q.min(0)).tolist(),
                       q_max_per_axis_deg=np.rad2deg(q.max(0)).tolist(),
                       angle_peak_per_axis_deg=np.rad2deg(p[0]).tolist(),
                       velocity_peak_per_axis_rad_s=p[1].tolist(),
                       acceleration_peak_per_axis_rad_s2=p[2].tolist(),
                       jerk_peak_per_axis_rad_s3=p[3].tolist(),
                       joint_position_bounds_ok=bool(np.all(q >= lower) and np.all(q <= upper)),
                       sampled_braking_endpoints_ok=bool(np.all(stop >= lower) and np.all(stop <= upper)),
                       low_speed_nonzero_fraction_per_axis=np.mean((abs(v) > .001) & (abs(v) < .02), axis=0).tolist(),
                       geometry=self.geometry(ab))
        return metrics


def grid_search(designer, count, iterations, seed, amplitudes, speeds, warm_start=None):
    """Joint waveform/scale/speed search under unchanged velocity/accel limits.

    Scale remains a coefficient-budget cap; actual angles are always reported,
    since a different Fourier waveform cannot be characterized by scale alone.
    """
    rng = np.random.default_rng(seed)
    warm = [] if warm_start is None else [np.array(c['coefficients_source_rad'])
                                           for c in warm_start['candidates'].values()]
    unit = np.linalg.norm(designer.ab0, axis=(0, 2))[None, :, None]
    output, log, grid = {}, [], []
    for name, jerk_fraction, description in [
            ('smooth-v1', .55, '优先平顺；J2加加速度上限为旧双0.5基线的55%'),
            ('balanced-v1', .8, '兼顾信息量与平顺；J2加加速度上限为旧双0.5基线的80%')]:
        pool = []
        for scale in amplitudes:
            for speed in speeds:
                designer.set_execution(scale, speed)
                caps = designer.reference_caps.copy()
                if getattr(designer, "search_velocity_caps", None) is not None:
                    caps[1] = np.minimum(caps[1], designer.search_velocity_caps)
                caps[0] *= scale/.5
                caps[0] = np.minimum(caps[0], np.deg2rad([145,40,55,84,65,115]))
                caps[3, 1] *= jerk_fraction
                starts = []
                for k in range(count):
                    if k == 0:
                        ab = designer.ab0.copy()
                    elif k <= len(warm):
                        ab = warm[k-1].copy()
                    else:
                        ab = rng.normal(size=(2, 6, 5))
                        ab /= np.arange(1, 6)[None, None, :]**rng.uniform(.2, 1.8)
                        peak = designer.peaks(ab, designer.t_constraints)[0]
                        ab *= (caps[0]/np.maximum(peak, 1e-12))[None, :, None]
                    ab = designer.project(ab, caps)
                    score, metrics = designer.evaluate(ab)
                    designer.evaluations += 1
                    starts.append((score, ab, metrics, scale, speed, caps))
                best = min(starts, key=lambda row: row[0])
                pool.extend(sorted(starts, key=lambda row: row[0])[:2])
                grid.append(dict(profile=name, scale=scale, speed_scale=speed,
                                 duration_s=designer.duration, **best[2]))
                print('GRID', name, scale, speed, 'kappa', round(best[2]['condition_fixed_scale'], 2),
                      'min-info', round(best[2]['minimum_information_relative'], 3),
                      'D-info', round(best[2]['D_information_relative'], 3), flush=True)
        pool.sort(key=lambda row: row[0])
        # Optimize the best two distinct scale/speed combinations.
        refined = set()
        for initial_row in list(pool):
            if len(refined) >= 2:
                break
            _, initial, _, scale, speed, caps = initial_row
            if (scale, speed) in refined:
                continue
            refined.add((scale, speed))
            designer.set_execution(scale, speed)
            best = [initial_row]
            def objective(x):
                ab = designer.project(np.asarray(x).reshape(2, 6, 5)*unit, caps)
                score, metrics = designer.evaluate(ab)
                designer.evaluations += 1
                if score < best[0][0]:
                    best[0] = (score, ab, metrics, scale, speed, caps)
                return score
            def callback(x):
                print('REFINE', name, scale, speed, 'best', round(best[0][0], 5), flush=True)
            result = minimize(objective, (initial/unit).reshape(-1), method='L-BFGS-B',
                              bounds=[(-2., 2.)]*60, callback=callback,
                              options=dict(maxiter=iterations, maxfun=5000, ftol=1e-7, maxls=12))
            pool.append(best[0])
            log.append(dict(candidate=name, scale=scale, speed_scale=speed,
                            success=bool(result.success), message=str(result.message),
                            function_evaluations=int(result.nfev), best_score=best[0][0]))
        pool.sort(key=lambda row: row[0])
        for _, ab, _, scale, speed, caps in pool[:12]:
            designer.set_execution(scale, speed)
            metrics = designer.report(ab)
            actual = np.array([np.deg2rad(metrics['angle_peak_per_axis_deg']),
                               metrics['velocity_peak_per_axis_rad_s'],
                               metrics['acceleration_peak_per_axis_rad_s2'],
                               metrics['jerk_peak_per_axis_rad_s3']])
            if (metrics['rank'] == 54 and metrics['minimum_information_relative'] >= 1
                    and metrics['D_information_relative'] >= getattr(designer, 'minimum_D_relative', 1.)
                    and metrics['joint_position_bounds_ok'] and metrics['sampled_braking_endpoints_ok']
                    and metrics['geometry']['simplified_nonadjacent_geom_min_distance_m'] >= .003
                    and metrics['geometry']['moving_geom_min_floor_clearance_m'] >= .01
                    and np.all(actual <= caps*(1+1e-6))):
                output[name] = dict(description_zh=description, execution=dict(scale=scale, speed_scale=speed),
                                    coefficients_source_rad=ab.tolist(), metrics=metrics,
                                    status='offline_screened_pending_operator_simulation_review',
                                    jerk_J2_cap_relative_to_baseline=jerk_fraction)
                print('SELECTED', name, scale, speed, json.dumps(metrics), flush=True)
                break
        if name not in output:
            raise RuntimeError('No grid candidate met all requirements: '+name)
        if getattr(designer, 'checkpoint_path', None):
            designer.checkpoint_path.write_text(json.dumps(dict(candidates=output,grid=grid,log=log),indent=2)+'\n')
    add_slower_control(designer,output,speeds,warm)
    return output, log, grid


def add_slower_control(designer,candidates,speeds,warm_coefficients=()):
    reference=min(candidates.values(),key=lambda c:c['metrics']['score_lower_better'])
    alternatives=[x for x in speeds if x<reference['execution']['speed_scale']]
    if not alternatives:return
    speed=max(alternatives);scale=reference['execution']['scale']
    designer.set_execution(scale,speed)
    caps=designer.reference_caps.copy();caps[0]*=scale/.5;caps[3,1]*=.55
    if getattr(designer, "search_velocity_caps", None) is not None:
        caps[1] = np.minimum(caps[1], designer.search_velocity_caps)
    waveforms=[np.asarray(reference['coefficients_source_rad']),*warm_coefficients,designer.ab0]
    pool=[]
    for ab in waveforms:
        ab=designer.project(ab,caps);score,metrics=designer.evaluate(ab)
        if metrics['rank']==54 and metrics['minimum_information_relative']>=1 and metrics['D_information_relative']>=getattr(designer,'minimum_D_relative',1.):
            pool.append((score,ab))
    for _,ab in sorted(pool,key=lambda row:row[0]):
        metrics=designer.report(ab)
        if (metrics['joint_position_bounds_ok'] and metrics['sampled_braking_endpoints_ok']
                and metrics['geometry']['simplified_nonadjacent_geom_min_distance_m']>=.003
                and metrics['geometry']['moving_geom_min_floor_clearance_m']>=.01):
            candidates['slow-v1']=dict(description_zh=f'低频大范围对照：速度系数{speed:g}，实际峰值另计；独立评分，不以采集更久占优',
                 execution=dict(scale=scale,speed_scale=speed),coefficients_source_rad=ab.tolist(),metrics=metrics,
                 status='offline_screened_pending_operator_simulation_review',jerk_J2_cap_relative_to_baseline=.55,
                 derivation='retimed/projected selected or warm-start waveform; independently rescored, no extra local refinement')
            break


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--base-file', type=Path, default=REPO.parent/'identification/revisions/PIPER-20261102-001/basis.json')
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--seed', type=int, default=20261030)
    ap.add_argument('--screen', type=int, default=120)
    ap.add_argument('--iterations', type=int, default=25)
    ap.add_argument('--amplitudes', default='.35,.5,.65,.8')
    ap.add_argument('--speeds', default='.25,.4,.5,.65')
    ap.add_argument('--warm-start', type=Path)
    ap.add_argument('--minimum-D-relative', type=float, default=1., help='Minimum total D information relative to old planned baseline; lower values explicitly allow an information/trackability tradeoff')
    ap.add_argument('--velocity-caps', help='Optional six comma-separated analytic reference speed caps in rad/s; screening only, not firmware settings')
    args = ap.parse_args()
    if args.output.exists():
        ap.error('Output exists; use a new filename to preserve previous iterations')
    if not 0 < args.minimum_D_relative <= 1:
        ap.error('minimum-D-relative must be in (0,1]')
    designer = Designer(args.base_file)
    designer.minimum_D_relative = args.minimum_D_relative
    designer.checkpoint_path = args.output.with_suffix('.checkpoint.json')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.velocity_caps:
        caps = np.asarray([float(v) for v in args.velocity_caps.split(',')])
        if caps.shape != (6,) or not np.all(np.isfinite(caps)) or not np.all(caps > 0):
            ap.error('velocity-caps must contain six positive finite rad/s values')
        designer.search_velocity_caps = caps
    baseline = designer.report(designer.ab0)
    print('BASELINE', json.dumps(baseline), flush=True)
    amplitudes = [float(x) for x in args.amplitudes.split(',')]
    speeds = [float(x) for x in args.speeds.split(',')]
    if not all(0 < x <= 1 for x in amplitudes+speeds) or args.screen < 2 or args.iterations < 1:
        ap.error('Scales must be in (0,1], screen>=2, iterations>=1')
    warm = json.loads(args.warm_start.read_text()) if args.warm_start else None
    candidates, search_log, grid = grid_search(designer, args.screen, args.iterations, args.seed,
                                              amplitudes, speeds, warm)
    version=re.search(r'v[0-9]+$',args.output.stem)
    if version:
        candidates={name.rsplit('-v',1)[0]+'-'+version.group():c for name,c in candidates.items()}
    designer.set_execution(.5,.5)
    comparisons = {}
    cfg = json.loads((BASE/'multisine_profile.json').read_text())
    for seed in (1234, 1111):
        ab = coefficients(seed, np.deg2rad(cfg['multisine_generation_lower_deg']), np.deg2rad(cfg['multisine_generation_upper_deg']))
        _, comparisons[str(seed)] = designer.evaluate(ab)
    bank = dict(schema_version=1, model='piper', iteration=args.output.stem, created_date=datetime.datetime.now(ZoneInfo('Asia/Shanghai')).date().isoformat(),
                execution=dict(anchor_deg=[0, 45, -60, 0, 0, 0],
                               source_duration_s=60., source_ramp_s=5., source_frequency_hz=.2, harmonics=5),
                model_sha256={str(p.relative_to(BASE)): sha(p) for p in [BASE/'assets/piper.urdf', BASE/'assets/nominal_piper.json']},
                provenance=dict(base_file=str(args.base_file), base_file_sha256=sha(args.base_file),
                                optimizer_sha256=sha(__file__), seed=args.seed,
                                reference_velocity_caps_rad_s=getattr(designer, "search_velocity_caps", np.array([])).tolist(),
                                minimum_D_information_relative=args.minimum_D_relative,
                                cap_interpretation="optional reference screening budget, not an identified firmware speed limit or hardware feasibility guarantee", random_candidates_per_scale_pair=args.screen,
                                amplitudes=amplitudes, speed_scales=speeds,
                                warm_start_sha256=sha(args.warm_start) if args.warm_start else None,
                                refinement='SciPy L-BFGS-B, two starts per profile',
                                iterations_per_start=args.iterations, total_evaluations=designer.evaluations,
                                pinocchio_version=designer.pin.__version__, hardware_access=False),
                evaluation=dict(inertial_base_indices=designer.indices.tolist(), linear_parameter_count=54,
                                friction='per axis: v, tanh(v/.02), constant offset',
                                common_column_RMS_scales=designer.fixed_scales.tolist(),
                                scope='planned analytic reference; not measured data or force accuracy',
                                information_window='one full steady cycle; source phase 10..15 seconds', information_sample_hz=50,
                                equal_noise_assumption=True, sample_count_normalized=True,
                                duration_policy='per-sample/per-time information; longer slow trials receive no duration bonus',
                                formulas=dict(score='log(kappa/kappa0)-0.5*log(smin/smin0)-0.5*(logD-logD0)',
                                              logD='2*mean(log(svd(A/fixed_column_RMS/sqrt(n_scalar_rows))))',
                                              minimum_information_relative='(smin/smin0)^2',
                                              D_information_relative='exp(logD-logD0)'),
                                limitations=['Current-to-torque calibration is pending.',
                                             '54 linear directions do not validate nonlinear Stribeck identifiability.',
                                             'Nominal simplified geometry is not full mesh/environment collision certification.',
                                             'Measured motion, filtering, colored noise and firmware smoothing change information quality.']),
                baseline=dict(id='legacy-seed0', execution=dict(scale=.5,speed_scale=.5),
                              coefficients_source_rad=designer.ab0.tolist(), metrics=baseline),
                scale_grid_screening=grid, legacy_seed_information=comparisons,
                candidates=candidates, optimization_log=search_log,
                references=['https://github.com/TUM-ICS/BIRDy/blob/main/Benchmark/Robot_Data_Generation/Trajectory_Data_Generation/Trajectory_Optimizer/trajectoryCriterion.m',
                            'https://github.com/kjyv/FloBaRoID',
                            'https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2020.571574/full'])
    for c in bank['candidates'].values():
        c['motion_sha256']=motion_identity(c['coefficients_source_rad'],{**bank['execution'],**c['execution']})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(bank, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print('SAVED', args.output, flush=True)


if __name__ == '__main__':
    main()
