"""Offline, exploratory hardware LS using the repository's original solver.

Never opens CAN or supplies simulator-generated torque as a measured target.
All parameter files are quarantined analysis artifacts, never controller inputs.
"""
import argparse
import hashlib
import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
from scipy.signal import savgol_filter


def load_npz(path):
    with np.load(path) as data:
        return {key:data[key] for key in data.files}


def original_modules():
    """Reuse main's regressor and retain the archived solver for existing trials."""
    from importlib import import_module
    import real_robot

    root = Path(real_robot.__file__).resolve().parent.parent
    package_root = root / 'src' / 'piper_dynamics'
    if (package_root / 'core' / 'rigid_body_regressor.py').is_file():
        # Existing real_robot entry points run from the repository root without
        # PYTHONPATH=src. Load the source package without changing sys.path.
        if 'piper_dynamics' not in sys.modules:
            spec = importlib.util.spec_from_file_location(
                'piper_dynamics', package_root / '__init__.py',
                submodule_search_locations=[str(package_root)])
            package = importlib.util.module_from_spec(spec)
            sys.modules['piper_dynamics'] = package
            spec.loader.exec_module(package)
        return (import_module('piper_dynamics.core.rigid_body_regressor'),
                import_module('piper_dynamics.legacy.snapshot.identify'),
                package_root / 'legacy' / 'snapshot')
    # The 2026-11-03 repository layout moved the frozen reference solver into
    # src/piper_dynamics/reference. Keep the legacy root layout working for
    # older checkouts, then fall back to the packaged reference snapshot.
    if not (root / '__init__.py').is_file():
        reference = root / 'src' / 'piper_dynamics' / 'reference'
        if not (reference / '__init__.py').is_file():
            raise RuntimeError('Dynamics reference solver is missing from both legacy root and src/piper_dynamics/reference')
        root = reference
    name = '_piper_original_identification'
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(
            name, root / '__init__.py', submodule_search_locations=[str(root)])
        package = importlib.util.module_from_spec(spec)
        sys.modules[name] = package
        spec.loader.exec_module(package)
    from importlib import import_module
    return (import_module(name + '.regressor'),
            import_module(name + '.identify'), root)


def inertial_model(model_name="piper"):
    """Use selected URDF kinematics, with preview-only compensation/passive terms off."""
    import mujoco
    from real_robot.simulation import model_xml
    model = mujoco.MjModel.from_xml_string(model_xml(model_name))
    model.body_gravcomp[:] = 0
    model.dof_damping[:] = 0
    model.dof_armature[:] = 0
    model.dof_frictionloss[:] = 0
    model.jnt_limited[:] = 0
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    return model


def interpolate_with_gaps(st, values, grid, max_gap=.03):
    st, values = np.asarray(st), np.asarray(values)
    order = np.argsort(st)
    st, indices = np.unique(st[order], return_index=True)
    values = values[order][indices]
    if len(st) < 2:
        raise ValueError('Not enough receive-timestamped measurements')
    result = np.interp(grid, st, values)
    right = np.searchsorted(st, grid, side='right')
    inside = (grid >= st[0]) & (grid <= st[-1])
    right = np.clip(right, 1, len(st)-1)
    valid = inside & ((st[right] - st[right-1]) <= max_gap)
    return result, valid


def load_measurements(run, dt=.01, window=21):
    """Decode only TESTING frames, then align by each CAN receive timestamp."""
    from real_robot.protocol_read import decode
    manifest = json.loads((run/'test_manifest.json').read_text())
    if len(manifest['windows']) != 1 or manifest['mode'] != 'hardware':
        raise ValueError('Exactly one real-hardware TESTING window is required')
    w = manifest['windows'][0]
    start, end = w['start_monotonic_s'], w['end_monotonic_s']
    qstreams, dstreams = [[] for _ in range(6)], [[] for _ in range(6)]
    with (run/'test_raw_can.jsonl').open() as raw:
        for line in raw:
            row = json.loads(line)
            ts = row['t_monotonic_s']
            if not start <= ts < end:
                raise ValueError('TESTING raw export contains an out-of-window frame')
            cid = int(row['can_id'], 16)
            if not (0x2a5 <= cid <= 0x2a7 or 0x251 <= cid <= 0x256):
                continue
            data = decode(cid, bytes.fromhex(row['data_hex']))
            if 0x2a5 <= cid <= 0x2a7:
                for j, q in zip(data['joint_pair'], data['position_deg']):
                    qstreams[j-1].append((ts-start, np.deg2rad(q)))
            else:
                dstreams[data['joint']-1].append(
                    (ts-start, data['velocity_sdk_rad_s'], data['current_signed_A']))
    if window != 1 and (window < 5 or window % 2 != 1):
        raise ValueError('Use window=1 for no filtering, or an odd SG window >=5')
    t = np.arange(dt, end-start-dt, dt)
    if len(t) <= window:
        raise ValueError('TESTING window is too short for filtering')
    q, qd, current = [np.zeros((len(t),6)) for _ in range(3)]
    good = np.ones((len(t),6), bool)
    for j in range(6):
        qp, dp = np.asarray(qstreams[j]), np.asarray(dstreams[j])
        for dest, st, val in ((q,qp[:,0],qp[:,1]),
                              (qd,dp[:,0],dp[:,1]),
                              (current,dp[:,0],dp[:,2])):
            dest[:,j], valid = interpolate_with_gaps(st, val, t)
            good[:,j] &= valid
    # A gap invalidates the entire symmetric filter support, not only its center.
    complete = np.all(good,axis=1)
    snapshots = load_npz(run/'test_signals.npz')
    ix = np.searchsorted(snapshots['t_test_s'],t,side='right')-1
    within_snapshots = (ix >= 0) & (ix < len(snapshots['t_test_s']))
    ix = np.clip(ix,0,len(snapshots['t_test_s'])-1)
    complete &= within_snapshots & snapshots['valid_measurement'][ix]
    support = 3 if window == 1 else window
    valid = np.convolve(complete.astype(int), np.ones(support,int), mode='same') == support
    filtered = {k:(a.copy() if window == 1 else savgol_filter(a,window,3,axis=0)) for k,a in
                [('q',q),('qd',qd),('current_A',current)]}
    filtered.update(t=t, valid=valid,
        qdd=np.gradient(qd,dt,axis=0,edge_order=2) if window == 1 else savgol_filter(qd,window,3,deriv=1,delta=dt,axis=0),
        qd_from_position=np.gradient(q,dt,axis=0,edge_order=2) if window == 1 else savgol_filter(q,window,3,deriv=1,delta=dt,axis=0),
        q_unfiltered=q, qd_unfiltered=qd, current_unfiltered_A=current)
    return filtered, manifest


def friction_matrix(qd):
    # Per axis: viscous + smoothed Coulomb + offset, all fitted together by LS.
    n = len(qd)
    out = np.zeros((n,6,18))
    for j in range(6):
        out[:,j,3*j:3*j+3] = np.column_stack(
            [qd[:,j], np.tanh(qd[:,j]/.02), np.ones(n)])
    return out


def metrics(pred, target):
    err = pred-target
    rmse = np.sqrt(np.mean(err**2,axis=0))
    rms = np.sqrt(np.mean(target**2,axis=0))
    spread = np.std(target,axis=0)
    return dict(rmse_Nm=rmse.tolist(), target_rms_Nm=rms.tolist(),
        rmse_over_rms=(rmse/np.maximum(rms,1e-12)).tolist(),
        r2_per_joint=(1-np.sum(err**2,axis=0)/np.maximum(
            np.sum((target-target.mean(0))**2,axis=0),1e-24)).tolist(),
        relative_error=float(np.linalg.norm(err)/max(np.linalg.norm(target),1e-12)),
        target_std_Nm=spread.tolist())


def fit_original(solver, Y, tau, train, validation):
    A = Y[train].reshape(-1,Y.shape[-1])
    norms = np.linalg.norm(A,axis=0)
    active = np.flatnonzero(norms > norms.max()*1e-10)
    if not len(active):raise ValueError('No nonzero regression columns')
    sol = solver(A[:,active],tau[train].reshape(-1),ridge=0.,use_qr=True)
    reduced_theta=sol['theta'];theta=np.zeros(A.shape[1]);theta[active]=reduced_theta
    sol['theta']=theta;sol['base_idx']=active[sol['base_idx']]
    pred = np.einsum('nij,j->ni',Y,sol['theta'])
    sv = np.linalg.svd(A[:,active]/norms[active],compute_uv=False)
    rank = sol['rank']
    return sol, pred, dict(rank=rank, n_parameters=Y.shape[-1],
        train_samples=int(train.sum()), validation_samples=int(validation.sum()),
        train=metrics(pred[train],tau[train]), validation=metrics(pred[validation],tau[validation]),
        normalized_singular_values=sv.tolist(),
        effective_condition_number=float(sv[0]/sv[rank-1]),
        base_idx=sol['base_idx'].tolist())


def run_analysis(run, output, window=21, fit_hz=20., torque_coefficients=None):
    output.mkdir(parents=True, exist_ok=True)
    rg, identification, root = original_modules()
    signals, manifest = load_measurements(run, window=window)
    t = signals['t']
    cfg = manifest['configuration']
    annotation = json.loads((run/'operator_annotations.json').read_text()) if (run/'operator_annotations.json').is_file() else {}
    recorded = load_npz(run/'test_signals.npz')
    nz = np.abs(recorded['current_A']) > .02
    recorded_factors = np.array([np.median(recorded['tau_candidate_Nm'][nz[:,j],j]/
                recorded['current_A'][nz[:,j],j]) for j in range(6)])
    coeff = recorded_factors if torque_coefficients is None else np.asarray(torque_coefficients,float)
    if coeff.shape != (6,) or not np.isfinite(coeff).all() or np.any(coeff == 0):
        raise ValueError('Six finite nonzero torque coefficients required')
    tau = signals['current_A'] * coeff
    # All source cycles are periodic after the ramp: retain whole cycles only.
    reference = manifest['windows'][0]['reference_plan']
    period = 1/reference['base_frequency_hz']
    ramp = reference['ramp_s']
    duration = reference['duration_s']
    # Keep filtering support away from held-out-cycle boundaries.
    phase_in_cycle = np.mod(t,period)
    guard = window//2*.01+.02
    plateau = signals['valid'] & (t >= ramp+1e-8) & (t < duration-ramp-1e-8)
    plateau &= (phase_in_cycle > guard) & (phase_in_cycle < period-guard)
    stride = max(1, round(100/fit_hz))
    selected = np.flatnonzero(plateau)[::stride]
    if len(selected) < 100:
        raise ValueError('Too few usable samples')
    print(f'Building original X-adapted regressor: {len(selected)} measured samples',flush=True)
    model = inertial_model()
    builder = rg.RegressorBuilder(model,link_bodies=tuple((f'link{i}',) for i in range(1,7)))
    Y = builder.build(signals['q'][selected],signals['qd'][selected],signals['qdd'][selected],progress=True)
    ts, vs, target = t[selected], signals['qd'][selected], tau[selected]
    cycle = np.floor((ts+1e-6)/period).astype(int)
    train = cycle % 2 == 0
    validation = ~train
    matrices = {'rigid_plus_friction_offset_LS':np.concatenate([Y,friction_matrix(vs)],axis=2)}
    fits, predictions, params = {}, {}, {}
    for name, matrix in matrices.items():
        sol, pred, report = fit_original(identification.solve_parameters,matrix,target,train,validation)
        all_sol, all_pred, all_report = fit_original(identification.solve_parameters,matrix,target,
            np.ones(len(ts),bool),np.ones(len(ts),bool))
        report['all_data_fit'] = all_report['train']
        report['early_validation'] = metrics(pred[validation & (ts<duration/2)],target[validation & (ts<duration/2)])
        report['late_validation'] = metrics(pred[validation & (ts>=duration/2)],target[validation & (ts>=duration/2)])
        # Transfer tests must compare predictions, not arbitrary null-space masses.
        early, late = ts<duration/2, ts>=duration/2
        _, _, report['fit_early_validate_late'] = fit_original(identification.solve_parameters,matrix,target,early,late)
        _, _, report['fit_late_validate_early'] = fit_original(identification.solve_parameters,matrix,target,late,early)
        fits[name] = report
        predictions[name] = pred
        params[name+'_theta_train'] = sol['theta']
        params[name+'_theta_all_exploratory'] = all_sol['theta']
        params[name+'_base_idx'] = sol['base_idx']
        params[name+'_prediction_train_fit'] = pred
        print(name,'rank',report['rank'],'validation relerr',report['validation']['relative_error'],flush=True)
    # Diagnostic nominal CAD reference only, never a target or claimed ground truth.
    import mujoco
    nominal = inertial_model()
    data = mujoco.MjData(nominal)
    nominal_tau=[]
    for q,v,a in zip(signals['q'][selected],vs,signals['qdd'][selected]):
        data.qpos[:]=q;data.qvel[:]=v;data.qacc[:]=a
        mujoco.mj_inverse(nominal,data)
        nominal_tau.append(data.qfrc_inverse.copy())
    nominal_tau=np.asarray(nominal_tau)
    # Separate an implementation error from hardware/model mismatch. CAD is
    # only used here as a mathematical self-check, never as the fitted target.
    nominal_for_theta = inertial_model()
    nominal_theta = rg.ground_truth_theta(nominal_for_theta).reshape(-1)
    self_check_max = float(np.max(np.abs(np.einsum('nij,j->ni',Y,nominal_theta)-nominal_tau)))
    if self_check_max > 1e-7:
        raise RuntimeError(f'X regressor self-check failed: {self_check_max}')
    # Compare matched phases of the repeated trajectory. Midpoint is only a proxy
    # for the unknown hand-support start, not a recovered intervention timestamp.
    available_cycles = np.unique(cycle).tolist()
    if len(available_cycles) < 8:
        raise ValueError('At least eight complete plateau cycles required for phase comparison')
    early_cycles = available_cycles[1:4]
    late_cycles = available_cycles[-4:-1]
    phase=np.arange(0.,period,.01)
    matched={}
    for key in ('q','qd','current_A'):
        values=signals[key]
        def cycles(ids):
            return np.stack([np.column_stack([np.interp(phase+c*period,t,values[:,j])
                                 for j in range(6)]) for c in ids])
        early,late=cycles(early_cycles),cycles(late_cycles)
        matched[key] = dict(early_cycle_std_rms=np.sqrt(np.mean(np.var(early,axis=0),axis=0)).tolist(),
            late_cycle_std_rms=np.sqrt(np.mean(np.var(late,axis=0),axis=0)).tolist(),
            mean_waveform_difference_rms=np.sqrt(np.mean((early.mean(0)-late.mean(0))**2,axis=0)).tolist())
    qder=signals['qd_from_position'][plateau]
    vel=signals['qd'][plateau]
    report=dict(source_run=str(run.resolve()), scope='TESTING only',
        trajectory_scale=cfg['trajectory_scale'],trajectory_speed_scale=cfg.get('trajectory_speed_scale',1.),
        operator_annotations=annotation, torque_calibrated=False,
        suitable_for_control=False, status='exploratory_pipeline_validation_only',
        torque_coefficients_Nm_per_A=coeff.tolist(),recorded_candidate_coefficients_Nm_per_A=recorded_factors.tolist(),
        preprocessing=dict(source='test_raw_can.jsonl; separate per-joint CAN receive timestamps',
            grid_hz=100.,SG_window_samples=window,SG_polynomial_order=None if window==1 else 3,
            SG_support_s=(window-1)*.01,phase='none' if window==1 else 'offline centered zero phase',
            qdd_source='central finite difference of unfiltered measured velocity' if window==1 else 'derivative of measured CAN velocity, not qdd_ref',
            max_interpolation_gap_s=.03,missing_support_masked=True,
            fit_hz=100/stride,selected_time_min_s=float(ts.min()),selected_time_max_s=float(ts.max()),
            cycle_boundary_guard_s=guard,
            train='even source cycles, filter-support boundary guard excluded',
            validation='odd source cycles; same trajectory, not independent motion'),
        source_hashes={name:hashlib.sha256((run/name).read_bytes()).hexdigest() for name in
            ('test_manifest.json','test_raw_can.jsonl','test_signals.npz')},
        original_code_hashes={name:hashlib.sha256((root/name).read_bytes()).hexdigest() for name in
            ('regressor.py','identify.py','real_robot/identify_offline.py','real_robot/assets/piper.urdf','real_robot/assets/nominal_piper.json')},
        model='Standard PiPER bundled URDF transforms, rigid gripper lumped link6; preview gravcomp/damping/armature/limits disabled',
        regressor_self_check_max_error_Nm=self_check_max,
        velocity_check=dict(correlation=[float(np.corrcoef(qder[:,j],vel[:,j])[0,1]) for j in range(6)],
            feedback_over_position_derivative_gain=(np.sum(qder*vel,axis=0)/np.sum(qder*qder,axis=0)).tolist()),
        fits=fits, nominal_CAD_prediction=metrics(nominal_tau,target),
        matched_phase_comparison=dict(early_windows_s=[[c*period,(c+1)*period] for c in early_cycles],
            late_windows_s=[[c*period,(c+1)*period] for c in late_cycles],signals=matched,
            caveat='Manual support timestamp and base pose/acceleration unmeasured; comparison cannot isolate or prove absence of base motion.'),
        limitations=['Base visibly moved; fixed-base assumption violated; cannot correct without base motion measurement.',
            'Current-to-output-torque coefficients, signs, offsets, gearbox efficiency and rotor inertia are not calibrated for this exact unit/firmware.',
            'Rigid-body plus friction and offset is the minimum baseline; this is not physical calibration.',
            'QR leaves unobservable parameter directions at zero; raw per-link mass/COM cannot be interpreted as physical estimates.',
            'Held-out cycles are from one repeated trajectory, not independent-trajectory validation.',
            'Nominal X wrist gravity allocation and measured current distribution disagree; exact geometry, joint mapping and current conventions require verification.',
            'Smoothing, receive-time alignment and numerical differentiation introduce uncertainty.'])
    snapshots_q, snapshots_ref = recorded['q'], recorded['q_ref']
    report['acquisition'] = json.loads((run/'test_quality.json').read_text())
    report['tracking'] = dict(rmse_deg=np.rad2deg(np.sqrt(np.mean((snapshots_q-snapshots_ref)**2,axis=0))).tolist(),
        max_error_deg=np.rad2deg(np.abs(snapshots_q-snapshots_ref).max(0)).tolist(),
        actual_peak_to_peak_deg=np.rad2deg(np.ptp(snapshots_q,axis=0)).tolist(),
        peak_feedback_speed_rad_s=np.abs(recorded['qd']).max(0).tolist(),
        peak_current_A=np.abs(recorded['current_A']).max(0).tolist())
    (output/'offline_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    (output/'operator_annotations.json').write_text(json.dumps(annotation,ensure_ascii=False,indent=2)+'\n')
    np.savez_compressed(output/'exploratory_parameters_NOT_FOR_CONTROL.npz',**params,
        t=ts,tau_candidate_Nm=target,nominal_CAD_tau=nominal_tau,
        param_names=np.array(builder.param_names),suitable_for_control=False,torque_calibrated=False)
    np.savez_compressed(output/'aligned_measured_signals.npz',**signals,tau_candidate_Nm=tau)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,2,figsize=(14,9),sharex=True)
    for j,ax in enumerate(axes.flat):
        ax.plot(recorded['t_test_s'],np.rad2deg(snapshots_ref[:,j]),label='Reference',lw=1)
        ax.plot(recorded['t_test_s'],np.rad2deg(snapshots_q[:,j]),label='Measured',lw=.8)
        ax.set_ylabel(f'J{j+1} (deg)');ax.grid(alpha=.2)
    axes[0,0].legend();axes[-1,0].set_xlabel('Test time (s)');axes[-1,1].set_xlabel('Test time (s)')
    fig.suptitle(f'HARDWARE | amplitude {cfg["trajectory_scale"]}, speed {cfg.get("trajectory_speed_scale",1.)} | see operator annotations for base conditions')
    fig.tight_layout();fig.savefig(output/'tracking.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(3,2,figsize=(14,9),sharex=True)
    for j,ax in enumerate(axes.flat):
        ax.plot(ts,target[:,j],label='Current-derived torque (uncalibrated)',lw=1,color='k')
        for name,pred in predictions.items():ax.plot(ts,pred[:,j],label=name,lw=.7,alpha=.8)
        ax.axvline(duration/2,color='gray',ls='--',alpha=.5)
        ax.set_ylabel(f'J{j+1} candidate Nm');ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=6);axes[-1,0].set_xlabel('Test time (s)');axes[-1,1].set_xlabel('Test time (s)')
    fig.suptitle('Exploratory original LS | even cycles train, odd cycles validate | NOT calibrated torque')
    fig.tight_layout();fig.savefig(output/'torque_fit.png',dpi=160);plt.close(fig)
    print('Saved',output,flush=True)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path)
    p.add_argument('--out',type=Path,required=True)
    p.add_argument('--sg-window',type=int,default=21)
    p.add_argument('--fit-hz',type=float,default=20.)
    p.add_argument('--torque-coefficients',type=float,nargs=6,default=None,
                   help='Explicit sensitivity study only; not a calibration')
    a=p.parse_args()
    if not 0<a.fit_hz<=100:raise SystemExit('fit-hz must be in (0,100]')
    run_analysis(a.run,a.out,a.sg_window,a.fit_hz,a.torque_coefficients)


if __name__=='__main__':main()
