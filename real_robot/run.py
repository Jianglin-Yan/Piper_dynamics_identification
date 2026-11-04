"""python -m real_robot.run --sim / --hardware / --read-only / --calibrate-pad"""
import argparse, datetime, json, signal, sys, time
from pathlib import Path
import numpy as np
from .controller import Controller
from .gamepad import Gamepad, calibrate
from .monitoring import validate_speed_limits
from .test_data import TestRecorder, export_test_data
from .ui import Printer

BASE=Path(__file__).parent

def serializable(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)

def validate_config(c):
    validate_speed_limits(c)
    c.setdefault('trajectory','pilot')
    if c['trajectory'] not in ('pilot','multisine','static','reciprocal','payload','friction','gravity','mit'):raise ValueError('Unknown trajectory')
    c.setdefault('trajectory_scale',1.)
    c.setdefault('trajectory_speed_scale',1.)
    if not np.isfinite(c['trajectory_speed_scale']) or not 0<c['trajectory_speed_scale']<=1:raise ValueError('trajectory_speed_scale must be in (0, 1]')
    if not np.isfinite(c['trajectory_scale']) or not 0<c['trajectory_scale']<=1:raise ValueError('trajectory_scale must be in (0, 1]')
    c.setdefault('startup_limit_tolerance_deg',10.);c.setdefault('tracking_warning_deg',10.);c.setdefault('measurement_angle_tolerance_deg',6.)
    if not np.isfinite(c['startup_limit_tolerance_deg']) or not 0<c['startup_limit_tolerance_deg']<=15:raise ValueError('Invalid startup allowance')
    if not np.isfinite(c['measurement_angle_tolerance_deg']) or not 0<=c['measurement_angle_tolerance_deg']<=15:raise ValueError('Invalid measurement angle tolerance')
    if not np.isfinite(c['tracking_warning_deg']) or not 0<c['tracking_warning_deg']<c['tracking_error_deg']:raise ValueError('Tracking warning must be below hard threshold')
    c.setdefault('settle_position_window_deg',.20);c.setdefault('arming_velocity_limit_rad_s',.15)
    c.setdefault('park_release_tolerance_deg',3.);c.setdefault('disable_timeout_s',2.);c.setdefault('disable_confirm_s',.15)
    for key in ('settle_position_window_deg','arming_velocity_limit_rad_s','park_release_tolerance_deg','disable_timeout_s','disable_confirm_s'):
        if not np.isfinite(c[key]) or c[key]<=0:raise ValueError('Invalid '+key)
    if not c['settle_error_deg']<=c['park_release_tolerance_deg']<=3.:raise ValueError('Park release tolerance must be between settle tolerance and 3 degrees')
    if c['disable_confirm_s']>=c['disable_timeout_s']:raise ValueError('Disable confirmation must fit the timeout')
    for key in ('lower_deg','upper_deg','simulation_initial_deg','max_current_feedback_A'):
        if len(c[key])!=6 or not np.all(np.isfinite(c[key])):raise ValueError('Invalid '+key)
    if np.any(np.array(c['max_current_feedback_A'])<=0):raise ValueError('Current thresholds must be positive')
    if np.any(np.array(c['lower_deg'])+2*c['margin_deg']>=np.array(c['upper_deg'])):raise ValueError('Empty joint safety interval')
    if c['start_deg'] is not None and (len(c['start_deg'])!=6 or not np.all(np.isfinite(c['start_deg']))):raise ValueError('Invalid start_deg')
    for key in ('amplitude_deg','frequency_hz','duration_s','ramp_s','margin_deg','max_velocity_rad_s','max_acceleration_rad_s2','tracking_error_deg','settle_error_deg','settle_velocity_rad_s','settle_timeout_s','feedback_timeout_s','status_timeout_s','loop_watchdog_s','max_temperature_C','command_hz'):
        if not np.isfinite(c[key]) or c[key]<=0:raise ValueError('Invalid '+key)
    if type(c['joint']) is not int or c['joint'] not in range(1,7):raise ValueError('joint must be 1..6')
    if c['model']!='piper':raise ValueError('Current real-robot profile is confirmed standard PiPER; legacy X is offline reproduction only')
    coeff=np.asarray(c.get('torque_candidate_coefficients_Nm_per_A',[]),float)
    if coeff.shape!=(6,) or not np.all(np.isfinite(coeff)) or np.any(coeff==0):raise ValueError('Six finite nonzero candidate torque coefficients required')
    if c['command_hz']!=100:raise ValueError('First pilot profile uses 100 Hz commands')
    if c['max_velocity_rad_s']>.2 or c['max_acceleration_rad_s2']>.3 or c['amplitude_deg']>5 or c['frequency_hz']>.3:
        raise ValueError('Outside small single-joint pilot profile')
    if not 1<=c['move_speed_percent']<=20:raise ValueError('Pilot move speed percentage must be 1..20')
    if c['trajectory']=='multisine':
        if c.get('current_pose_only'):raise ValueError('Six-joint multisine cannot use --from-current')
        for key in ('test_max_velocity_rad_s','test_max_acceleration_rad_s2','braking_max_acceleration_rad_s2','test_tracking_warning_deg','test_tracking_hard_deg','multisine_base_frequency_hz','multisine_sample_dt_s'):
            if not np.isfinite(c[key]) or c[key]<=0:raise ValueError('Invalid '+key)
        if c['test_tracking_warning_deg']>=c['test_tracking_hard_deg']:raise ValueError('Tracking warning must be below hard threshold')
        if type(c['multisine_harmonics']) is not int or not 1<=c['multisine_harmonics']<=5:raise ValueError('Invalid harmonic count')
        if type(c['multisine_seed']) is not int or c['multisine_seed']<0:raise ValueError('Invalid multisine seed')
        if not 0<c['multisine_amplitude_fraction']<=.6:raise ValueError('Invalid multisine amplitude fraction')
        for key in ('multisine_generation_lower_deg','multisine_generation_upper_deg'):
            if len(c[key])!=6 or not np.all(np.isfinite(c[key])):raise ValueError('Invalid '+key)
        if np.any(np.array(c['multisine_generation_lower_deg'])>=np.array(c['multisine_generation_upper_deg'])):raise ValueError('Invalid multisine generation interval')
        if c['multisine_sample_dt_s']!=.002:raise ValueError('Original generator uses 0.002 s interpolation knots')
        if c.get('feedback_stop_policy')!='warn':raise ValueError('Multisine profile requires warn-only reference policy')
    if c['trajectory'] in ('static','reciprocal','payload'):
        if not c.get('standing_start') or c.get('current_pose_only'):raise ValueError('Validation trials require --standing')
        payload=c['trajectory']=='payload'
        if c.get('passive_wrist') or bool(c.get('release_gripper'))!=payload or bool(c.get('hold_start_wrist'))!=payload:raise ValueError('Validation uses six enabled joints; only payload holds startup J6 and releases gripper')
        speeds=c['reciprocal_speeds_rad_s']
        if not speeds or any(not np.isfinite(v) or not .01<=v<=.12 for v in speeds):raise ValueError('Reciprocal speeds must be within 0.01..0.12 rad/s')
        if c['trajectory_scale']!=1 or c['trajectory_speed_scale']!=1:raise ValueError('Validation protocols use fixed slow paths; do not apply multisine scales')
        for key in ('validation_move_velocity_rad_s','validation_move_acceleration_rad_s2','validation_settle_s','validation_capture_s','validation_settle_timeout_s'):
            if not np.isfinite(c[key]) or c[key]<=0:raise ValueError('Invalid '+key)
        if c['validation_move_velocity_rad_s']>.12 or c['validation_move_acceleration_rad_s2']>.12:raise ValueError('Validation paths exceed slow profile')
        if not 1<=c['validation_settle_s']<=30 or not 2<=c['validation_capture_s']<=60:raise ValueError('Invalid diagnostic dwell')
    if c['trajectory']=='friction':
        if c.get('friction_start','current')=='current':
            if c.get('standing_start') or c.get('current_pose_only'):raise ValueError('Current-pose friction collection must not use --standing/--from-current')
        elif not c.get('standing_start') or c.get('current_pose_only'):raise ValueError('Standing friction collection requires --standing')
        if c.get('passive_wrist') or c.get('hold_start_wrist') or c.get('release_gripper'):raise ValueError('Friction collection uses six enabled joints and no gripper')
        if c['trajectory_scale']!=1 or c['trajectory_speed_scale']!=1:raise ValueError('Friction collection uses fixed paths')
        joints=[int(j) for j in c['friction_joints']]
        if sorted(set(joints))!=joints or any(not 1<=j<=6 for j in joints):raise ValueError('Invalid friction joint list')
        strokes=np.asarray(c['friction_stroke_deg'],float)
        if strokes.shape!=(6,) or not np.isfinite(strokes).all() or np.any(strokes<2) or np.any(strokes>40):raise ValueError('Invalid friction stroke')
        speeds=np.asarray(c['friction_speeds_rad_s'],float)
        if speeds.size<2 or not np.isfinite(speeds).all() or np.any(speeds<.002) or np.any(speeds>.25) or np.any(np.diff(speeds)<=0):raise ValueError('Invalid friction speeds')
        if not .5<=c['friction_dwell_s']<=10 or not 1<=c['friction_repeats']<=5:raise ValueError('Invalid friction dwell/repeats')
        if not np.isfinite(c['friction_move_acceleration_rad_s2']) or not 0<c['friction_move_acceleration_rad_s2']<=.5:raise ValueError('Invalid friction acceleration')
    if c['trajectory']=='mit':
        if not c.get('standing_start') or c.get('current_pose_only') or c['trajectory_scale']!=1 or c['trajectory_speed_scale']!=1:raise ValueError('MIT uses its fixed standing path without multisine scaling')
        if not np.isfinite(c['mit_ff_scale']) or not 0<=c['mit_ff_scale']<=1 or c['mit_ff_ramp_s']!=3.:raise ValueError('MIT FF fraction/ramp invalid')
        for key in ('mit_kp','mit_kd','mit_prepare_kp','mit_prepare_kd','mit_return_kp','mit_return_kd'):
            values=np.asarray(c[key],float)
            if values.shape!=(6,) or not np.isfinite(values).all() or np.any(values<=0):raise ValueError('Invalid '+key)
        if np.any(np.array(c['mit_return_kp'])<c['mit_kp']) or np.any(np.array(c['mit_return_kd'])<c['mit_kd']):raise ValueError('MIT return gains must not be lower than test gains')
    if c['duration_s']>120 or c['duration_s']<2*c['ramp_s']:raise ValueError('Invalid trial duration')

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    mode=ap.add_mutually_exclusive_group(required=True)
    for flag in ('sim','hardware','read-only','calibrate-pad','monitor-pad','recover'):mode.add_argument('--'+flag,action='store_true')
    ap.add_argument('--supported',action='store_true',help='explicit recovery only: confirm the folded arm is physically supported for power loss')
    ap.add_argument('--config',type=Path,default=BASE/'config.json');ap.add_argument('--pad',type=Path,default=BASE/'g8.json')
    ap.add_argument('--standing',action='store_true',help='LB+X performs boundary entry then moves to a fixed standing test pose')
    ap.add_argument('--from-current',action='store_true',help='J6-only trial anchored at actual startup pose; limited parked J2/J3 exceptions')
    ap.add_argument('--trajectory',choices=['pilot','multisine','static','reciprocal','payload','friction','gravity','mit'],help='pilot/multisine diagnostics or unified MIT PD/FF trial')
    ap.add_argument('--ff-scale',type=float,help='MIT-only trial: gravity feedforward fraction 0..1, default0')
    ap.add_argument('--mit-gain-scale',type=float,help='MIT-only trial: protocol PD gain fraction 0.5..1, default1')
    ap.add_argument('--gravity-model',choices=['nominal','identified'],help='gravity/MIT: official nominal or corrected balanced-v2 identification')
    ap.add_argument('--gravity-fit',choices=['raw_LS','sg21_LS','sg41_LS','sg21_NNLS','sg21_Stribeck_NLS','sg21_Stribeck_GA'],help='gravity only: frozen balanced-v2 fit; default sg21_LS')
    ap.add_argument('--gravity-seconds',type=float,help='gravity only: test duration 10..120s, default30')
    ap.add_argument('--gravity-stiffness',type=float,help='gravity only: final weak position gain fraction 0..1; 0 gives gravity+damping')
    ap.add_argument('--reciprocal-speeds',type=float,nargs='+',help='reciprocal only: paired speed caps in rad/s; default 0.04 0.12')
    ap.add_argument('--friction-joints',type=int,nargs='+',help='friction only: joint order, default 1 6 4 5 2 3')
    ap.add_argument('--friction-speeds',type=float,nargs='+',help='friction only: speed levels in rad/s')
    ap.add_argument('--friction-repeats',type=int,help='friction only: repeats per speed')
    ap.add_argument('--friction-start',choices=['current','standing'],default='standing',help='friction only: start and return at the measured pose (default) or at the standing pose')
    ap.add_argument('--capture-seconds',type=float,help='validation only: stationary recording duration, default 6')
    ap.add_argument('--settle-seconds',type=float,help='validation only: extra stable time before recording, default 2')
    ap.add_argument('--payload-point-m',type=float,nargs=3,help='optional hanging point xyz in link6 coordinates, metres; do not guess')
    ap.add_argument('--seed',type=int,help='reproducible multisine trajectory seed; original experiment used 0')
    ap.add_argument('--candidate',help='frozen optimized multisine candidate, e.g. smooth-v1')
    ap.add_argument('--candidate-bank',type=Path,default=BASE.parent.parent/'identification/trajectories/candidates-v1.json')
    ap.add_argument('--speed-scale',type=float,help='test playback-speed fraction (0,1]; 0.5 doubles test/ramp duration, preserving motion range')
    ap.add_argument('--scale',type=float,help='test displacement fraction (0,1]; preparation, return, time and frequency unchanged')
    ap.add_argument('--preflight-only',action='store_true',help='validate the hardware session without reading button actions or enabling motors')
    ap.add_argument('--headless',action='store_true',help='simulation only: skip the preview window for automated checks')
    ap.add_argument('--viewer',action='store_true');ap.add_argument('--auto',action='store_true',help='自动 PREPARE→START，跑完自行回位失能；不需要手柄')
    ap.add_argument('--raw-can',action='store_true',help='merge: also log every CAN frame to raw_can.jsonl (large)')
    ap.add_argument('--csv',action='store_true',help='merge: also export test_signals.csv and test_acceleration.csv');ap.add_argument('--scripted',choices=['normal','abort','speed-stop'],help='simulation only; speed-stop injects speed feedback to preview a controlled stop')
    ap.add_argument('--output',type=Path,default=BASE.parent/'runs');ap.add_argument('--max-seconds',type=float,default=120)
    a=ap.parse_args()
    if a.headless:
        if not a.sim:ap.error('--headless is simulation-only')
        a.viewer=False
    if a.scripted and not a.sim:ap.error('--scripted is simulation-only')
    if not np.isfinite(a.max_seconds) or a.max_seconds<=0:ap.error('--max-seconds must be positive and finite')
    if a.viewer and not a.sim:ap.error('--viewer is simulation-only')
    if a.standing and a.from_current:ap.error('--standing and --from-current cannot be combined')
    if a.standing and not (a.hardware or a.sim):ap.error('--standing requires --hardware or --sim')
    if a.from_current and not (a.hardware or a.sim):ap.error('--from-current requires --hardware or --sim')
    if a.preflight_only and not a.hardware:ap.error('--preflight-only requires --hardware')
    if a.recover and not a.supported:ap.error('恢复会立即失电：先支撑好蜷缩机械臂，再使用 recover --supported 明确确认；不会自动使能。')
    if a.supported and not a.recover:ap.error('--supported is only for --recover')
    if a.calibrate_pad:calibrate(a.pad);return
    if a.monitor_pad:
        pad=Gamepad(a.pad);print('只读手柄验证：LB+X / LB+Y / LB+RB；不连接 CAN。',flush=True);end=time.monotonic()+a.max_seconds
        try:
            while time.monotonic()<end:
                action=pad.poll(time.monotonic())
                if action:print(action,flush=True)
                time.sleep(.005)
        except KeyboardInterrupt:
            print('手柄监视已结束。',flush=True)
        finally:pad.close()
        return
    cfg=json.loads(a.config.read_text())
    selected=a.trajectory or cfg.get('trajectory','pilot')
    if selected=='multisine':
        if not a.standing or a.from_current:ap.error('Six-joint multisine uses --standing; cannot use --from-current')
        if cfg.get('trajectory')!='multisine':cfg.update(json.loads((BASE/'multisine_profile.json').read_text()))
    cfg['trajectory']=selected;cfg['current_pose_only']=a.from_current;cfg['standing_start']=a.standing
    if selected=='mit':
        if not (a.sim or a.hardware):ap.error('MIT trial is sim/hardware only')
        if a.gravity_seconds is not None or a.gravity_stiffness is not None:ap.error('MIT fixed-path trial uses --ff-scale and --mit-gain-scale, not gravity hold options')
        from .mit_trial import configure
        configure(cfg,0. if a.ff_scale is None else a.ff_scale,1. if a.mit_gain_scale is None else a.mit_gain_scale,a.gravity_model or 'nominal',a.gravity_fit or 'sg21_LS')
    elif a.ff_scale is not None or a.mit_gain_scale is not None:ap.error('--ff-scale/--mit-gain-scale require --trajectory mit')
    if selected=='gravity':
        if a.hardware:ap.error('Legacy MOVE J-to-MIT hardware trial retired. Use bash real_robot/test_mit.sh hardware --ff-scale 0 or 0.25')
        from .gravity import configure
        configure(cfg,a.gravity_model or 'identified',a.gravity_fit or 'sg21_LS',30. if a.gravity_seconds is None else a.gravity_seconds,1. if a.gravity_stiffness is None else a.gravity_stiffness)
    elif selected!='mit' and any(x is not None for x in (a.gravity_model,a.gravity_fit,a.gravity_seconds,a.gravity_stiffness)):ap.error('Gravity options require --trajectory gravity/mit')
    if selected=='friction':
        from .friction import configure as friction_configure
        friction_configure(cfg)
        if a.friction_joints is not None:cfg['friction_joints']=a.friction_joints
        if a.friction_speeds is not None:cfg['friction_speeds_rad_s']=a.friction_speeds
        if a.friction_repeats is not None:cfg['friction_repeats']=a.friction_repeats
        cfg['friction_start']=a.friction_start
    elif any(x is not None for x in (a.friction_joints,a.friction_speeds,a.friction_repeats)):ap.error('--friction-* options require --trajectory friction')
    if selected in ('static','reciprocal','payload'):
        from .validation_trials import configure
        configure(cfg)
        if a.reciprocal_speeds is not None:
            if selected!='reciprocal':ap.error('--reciprocal-speeds requires the reciprocal test')
            cfg['reciprocal_speeds_rad_s']=a.reciprocal_speeds
        if a.capture_seconds is not None:cfg['validation_capture_s']=a.capture_seconds
        if a.settle_seconds is not None:cfg['validation_settle_s']=a.settle_seconds
        if a.payload_point_m is not None:
            if not np.all(np.isfinite(a.payload_point_m)):ap.error('Invalid hanging point')
            cfg['payload_point_link6_m']=a.payload_point_m
    elif any(x is not None for x in (a.capture_seconds,a.settle_seconds,a.payload_point_m,a.reciprocal_speeds)):ap.error('Diagnostic options require static, reciprocal or payload')
    if a.scale is not None:cfg['trajectory_scale']=a.scale
    if a.speed_scale is not None:cfg['trajectory_speed_scale']=a.speed_scale
    if a.seed is not None:
        if selected!='multisine':ap.error('--seed requires --trajectory multisine')
        cfg['multisine_seed']=a.seed
    if a.standing:
        from .standing import STANDING_DEG
        cfg['start_deg']=cfg.get('standing_start_deg',STANDING_DEG)
    if a.raw_can:cfg['raw_can_enabled']=True
    if a.csv:cfg['export_csv']=True
    cfg.setdefault('raw_can_enabled',True);cfg.setdefault('export_csv',True)
    if selected=='friction' and cfg.get('friction_start')=='current':
        cfg['standing_start']=False;cfg['start_deg']=None
    if a.candidate:
        if selected!='multisine' or a.seed is not None:
            ap.error('--candidate requires --trajectory multisine; cannot combine with --seed')
        from .designed_trajectory import load_candidate
        candidate=load_candidate(cfg,a.candidate,a.candidate_bank)
        print('CANDIDATE：'+a.candidate+f'；冻结幅度/速度={cfg["trajectory_scale"]:g}/{cfg["trajectory_speed_scale"]:g}；'+candidate['description_zh'],flush=True)
    validate_config(cfg)
    if a.from_current:
        from .current_pose import current_pose_bounds
        # Validate mode restrictions before opening CAN; actual bounds use a fresh snapshot later.
        if cfg['joint']!=6 or cfg['start_deg'] is not None or cfg['frequency_hz']>.1:
            raise ValueError('--from-current requires joint=6, start_deg=null, frequency<=0.1 Hz')
    path=a.output/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f');path.mkdir(parents=True)
    (path/'config.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2))
    b=None;pad=None;viewer=None;ctrl=None;lease=None;recorder=None;result={'mode':'sim' if a.sim else 'recovery' if a.recover else 'read-only' if a.read_only else 'hardware','completed':False}
    events=(path/'events.jsonl').open('w',buffering=1);samples=(path/'samples.jsonl').open('w',buffering=65536)
    log_failure=None
    def emit(msg):
        nonlocal log_failure
        try:print(msg,flush=True)
        except OSError:pass
        try:events.write(json.dumps({'t_monotonic_s':time.monotonic(),'message':msg},ensure_ascii=False)+'\n')
        except OSError as error:log_failure=str(error)
    ui=Printer(emit)
    if selected=='mit':
        ui.header('MIT 固定轨迹 · 重力前馈验证',['控制方式：MIT（PD＋重力前馈）',f'前馈比例：{100*cfg["mit_ff_scale"]:.0f}%','任务：五姿态固定轨迹，核对辨识模型的重力补偿'])
    elif selected=='gravity':
        ui.header('MIT 重力补偿试验',['控制方式：MIT（PD＋重力前馈）',f'前馈比例：{100*cfg["mit_ff_scale"]:.0f}%','任务：重力补偿保持'])
    elif selected=='multisine':
        ui.header('MOVE J 六轴多正弦',['控制方式：MOVE J（固件位置控制）',f'任务：{100*float(cfg.get("trajectory_scale",1)):.0f}% 幅度六轴多正弦'])
    else:
        names={'static':'静态验证','reciprocal':'正反对照','payload':'负载测试','pilot':'J6 试采'}
        ui.header(f'MOVE J {names.get(cfg["trajectory"],cfg["trajectory"])}',['控制方式：MOVE J（固件位置控制）',f'任务：{names.get(cfg["trajectory"],cfg["trajectory"])}'])
    def record(row):
        nonlocal log_failure
        if ctrl is not None and getattr(ctrl,'mit_telemetry',None) is not None:row['mit']=ctrl.mit_telemetry
        try:samples.write(json.dumps(row,default=serializable)+'\n')
        except OSError as error:log_failure=str(error)
    from .operator_exit import OperatorExit
    operator_exit=OperatorExit(emit)
    old_handlers={s:signal.signal(s,operator_exit.signal) for s in (signal.SIGINT,signal.SIGTERM)}
    sim_note='实机'
    try:
        if a.sim:
            if cfg.get('passive_wrist',False):
                from .validation_simulation import ValidationSimulation as Simulation
            elif selected=='mit':
                from .mit_trial import MitSimulation as Simulation
            elif selected=='gravity':
                from .gravity import GravitySimulation as Simulation
            else:
                from .simulation import Simulation
            b=Simulation(cfg);sim_note='仿真预览（未连接机械臂）'
        else:
            if selected=='mit':
                from .mit_trial import MitHardware as Hardware
            elif selected=='gravity':
                from .gravity import GravityHardware as Hardware
            else:
                from .hardware import Hardware
            if a.hardware or a.recover:
                import fcntl, re
                if not re.fullmatch(r'[A-Za-z0-9_-]+',cfg['channel']):raise ValueError('Invalid CAN channel name')
                lease=open('/tmp/piper-pilot-'+cfg['channel']+'.lock','w')
                fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
            if a.hardware:
                from .session_history import previous_controller_session, recovery_notice
                history=previous_controller_session([BASE.parent/'runs',a.output],cfg['channel'],exclude=path)
                result['previous_controller_session']=history
                if history:
                    emit('STARTUP_HISTORY：'+history['reason']+' 记录：'+history['run'])
                pending=path.parent/'.recovery_pending'
                notice=None if a.preflight_only else ('上次控制会话以故障/电子急停结束，未记录到成功恢复。' if pending.exists() else recovery_notice(history))
                if notice:
                    ui.warn(notice)
                    if sys.stdin.isatty():
                        ui.event('请人工确认机械臂已支撑、无电子急停。按 Enter 继续，Ctrl+C 退出。')
                        try:input()
                        except EOFError:raise RuntimeError('未确认恢复条件，退出')
                        result['operator_confirmed_after_recovery_warning']=True
                    else:ui.warn('非交互运行：不等待确认，请自行核实')
            b=Hardware(cfg,path);info=b.discover();(path/'hardware.json').write_text(json.dumps(info,ensure_ascii=False,indent=2));emit('只读查询完成：'+next((x for x in info['firmware'].split('\x00') if x.startswith('S-V')),'未知固件')+'；详细参数已保存 hardware.json')
            s=b.snapshot();b.check(s,False)
            if result.get('operator_confirmed_after_recovery_warning') and history and history.get('requires_recovery'):
                b.reset_once();time.sleep(1.);s=b.snapshot()
                if np.any(s['enabled']) or s['status'].get('arm_status')!=0 or s['status'].get('error_code')!=0:
                    raise RuntimeError('恢复未确认成功：六轴仍使能或故障码未清零；不自动重试，请断电重启后再运行')
                (path.parent/'.recovery_pending').unlink(missing_ok=True)
                emit('RECOVERED：已按确认发送一次重置，六轴失能、故障码为零；继续本次任务。')
            if a.recover:
                if 'S-V1.7-3250618' not in info['firmware']:raise ValueError('Recovery not reviewed for this firmware')
                if np.max(np.abs(s['qd']))>.1:raise ValueError('Robot moving: recovery refused')
                result['before']=s
                emit('RECOVER：已明确确认机械臂蜷缩有支撑；仅发送一次恢复/重置，不使能、不发送目标。')
                b.reset_supported();time.sleep(1.);after=b.snapshot();result['after']=after
                if np.any(after['enabled']) or after['status'].get('arm_status')!=0 or after['status'].get('error_code')!=0 or any(after['faults']):
                    raise RuntimeError('恢复未确认成功；不自动重试。')
                (path.parent/'.recovery_pending').unlink(missing_ok=True)
                emit('RECOVERED：六轴已失能、故障码为零。恢复命令结束；运动需要另行运行并由手柄触发。')
                result.update(completed=True,state='RECOVERED');return
            if a.read_only:
                samples.write(json.dumps(s,default=serializable)+'\n');result.update(completed=True,state='READ_ONLY');return
            b.preflight(info)
            (path/'effective_config.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2))
        if not a.scripted and not a.preflight_only and not a.auto:pad=Gamepad(a.pad)
        if a.viewer:
            if not a.sim:raise ValueError('viewer is for simulation only')
            import mujoco.viewer
            viewer=mujoco.viewer.launch_passive(b.model,b.data)
            viewer.cam.distance=1.3;viewer.cam.lookat[:]=[0,0,.3];viewer.cam.azimuth=135;viewer.cam.elevation=-25
        s=b.snapshot();now=0. if a.sim else time.monotonic();ctrl_type=Controller
        if selected=='mit':
            from .mit_trial import MitController
            ctrl_type=MitController
        elif selected=='gravity':
            from .gravity import GravityController
            ctrl_type=GravityController
        ctrl=ctrl_type(b,cfg,s,now,ui.event);ctrl.ui=ui
        if selected in ('gravity','mit'):(path/'reference_plan.json').write_text(json.dumps(ctrl.reference_plan,ensure_ascii=False,indent=2))
        if ctrl.reference_plan and ctrl.multisine:
            from .multisine import save_reference
            save_reference(path,ctrl.test,ctrl.reference_plan)
            limits=cfg.get('reported_motor_max_acceleration_rad_s2')
            if limits and np.any(np.array(ctrl.reference_plan['max_acceleration_per_joint_rad_s2'])>limits):
                emit('FIRMWARE_PLANNER_WARNING：参考加速度高于固件返回的规划配置；仅提示，固件可能平滑/限速。不会修改固件配置。')
        if ctrl.trial:ctrl.trial.save(path)
        result['home_rad']=ctrl.home.tolist()
        result['return_home_rad']=ctrl.return_home.tolist()
        result['current_pose_only']=a.from_current
        result['standing_start']=a.standing
        (path/'session_policy.json').write_text(json.dumps({'current_pose_only':a.from_current,'home_deg':np.rad2deg(ctrl.home).tolist(),'return_home_deg':np.rad2deg(ctrl.return_home).tolist(),'return_home_delta_deg':np.rad2deg(ctrl.return_home-ctrl.home).tolist(),'measurement_bounds_reference':'firmware/config intersection expanded outward; preparation/return also accept startup corridor','measurement_angle_tolerance_deg':cfg['measurement_angle_tolerance_deg'],'test_measurement_lower_deg':(np.asarray(cfg['lower_deg'])-cfg['measurement_angle_tolerance_deg']).tolist(),'test_measurement_upper_deg':(np.asarray(cfg['upper_deg'])+cfg['measurement_angle_tolerance_deg']).tolist(),'measurement_lower_deg':np.rad2deg(ctrl.measurement_bounds()[0]).tolist(),'measurement_upper_deg':np.rad2deg(ctrl.measurement_bounds()[1]).tolist(),'command_lower_deg':np.rad2deg(ctrl.lower).tolist(),'command_upper_deg':np.rad2deg(ctrl.upper).tolist(),'held_command_joints':[1,2,3,4,5] if a.from_current else [],'moving_joint':cfg['joint'] if cfg['trajectory']=='pilot' else None,'moving_joints':[1,2,3,4,5,6] if cfg['trajectory'] in ('multisine','mit','gravity') else [cfg['joint']],'shutdown':'park_then_confirm_six_axis_disabled','park_release_tolerance_deg':cfg['park_release_tolerance_deg'],'disable_confirm_s':cfg['disable_confirm_s']},ensure_ascii=False,indent=2))
        if ctrl.trial:
            policy=json.loads((path/'session_policy.json').read_text())
            policy.update(controlled_joints=ctrl.trial.active_joints,passive_joints=ctrl.trial.passive_joints,moving_joints=[1,2,3,4,5] if cfg.get('hold_start_wrist') else ctrl.trial.active_joints,gripper_control=ctrl.trial.plan['gripper_policy'],fixed_at_startup_joints=[6] if cfg.get('hold_start_wrist') else [],return_pose_policy='J1–J5 park; J6 holds startup angle' if cfg.get('hold_start_wrist') else 'all six joints park')
            (path/'session_policy.json').write_text(json.dumps(policy,ensure_ascii=False,indent=2))
        if a.preflight_only:
            samples.write(json.dumps({'feedback':s,'home_rad':ctrl.home},default=serializable)+'\n')
            result.update(completed=True,state='PREFLIGHT_ONLY')
            return
        recorder=TestRecorder(path,cfg,'sim' if a.sim else 'hardware')
        dt=1/cfg['command_hz'];deadline=time.monotonic();start_wall=deadline;step=0;last_wall=deadline;scripted_started=False;scripted_aborted=False;timeout_handled=False;input_failed=False;viewer_closed=False;last_rx_dropped=0;last_tick_duration=0.;last_tick_state=ctrl.state;last_status=-1e9;last_step=-1;last_friction_key=None
        while not ctrl.finished:
            now=step*dt if a.sim else time.monotonic()
            if viewer and not viewer.is_running() and not viewer_closed:
                viewer_closed=True;ctrl.request_stop(b.snapshot(),now,'Viewer closed')
            action=None
            if pad and not input_failed:
                try:action=pad.poll(time.monotonic())
                except OSError as e:
                    input_failed=True;action='STOP';emit('INPUT_STOP：'+str(e))
            if a.auto:
                if step==10:action='PREPARE'
                elif ctrl.state=='READY' and not scripted_started:action='START';scripted_started=True
            if a.scripted:
                if step==10:action='PREPARE'
                elif ctrl.state=='READY' and not scripted_started:action='START';scripted_started=True
                elif a.scripted=='normal' and ctrl.trial and ctrl.state=='TESTING':action=ctrl.trial.scripted_action()
                elif a.scripted=='abort' and ctrl.state=='TESTING' and now-ctrl.path_t>=5.2 and not scripted_aborted:
                    action='ABORT';scripted_aborted=True
            if ctrl.shutdown_phase and action in ('PREPARE','START'):action=None
            if action:ui.event(('自动：' if a.auto else '手柄：')+action)
            if a.sim:b.advance(dt)
            s=b.snapshot()
            dropped=s.get('can_rx_dropped_total',0)
            if dropped!=last_rx_dropped:
                emit(f'CAN_RX_WARNING：接收累计丢帧{dropped}；记录数据缺口，接收继续。最新反馈超时仍会停止。')
                last_rx_dropped=dropped
            now=s['time'] if a.sim else time.monotonic()
            if a.scripted=='speed-stop' and ctrl.state=='TESTING' and 5.2<=now-ctrl.path_t<5.5:
                # Simulation-only sensor injection; never changes motor position or real CAN data.
                s['qd']=s['qd'].copy();s['qd'][cfg['joint']-1]=cfg['feedback_speed_stop_rad_s']+.05
            operator_exit.update(ctrl,s,now)
            if time.monotonic()-start_wall>a.max_seconds and not timeout_handled:
                timeout_handled=True;ctrl.request_stop(s,now,'Session timeout')
            loop_gap=time.monotonic()-last_wall
            if not a.sim and ctrl.active and loop_gap>cfg['loop_watchdog_s']:
                ctrl.fault(f'Control loop late: gap={loop_gap*1000:.1f} ms, threshold={cfg["loop_watchdog_s"]*1000:.1f} ms, previous_state={last_tick_state}, previous_tick={last_tick_duration*1000:.1f} ms')
            last_wall=time.monotonic();tick_started=last_wall;last_tick_state=ctrl.state
            ctrl.tick(now,s,action);last_tick_duration=time.monotonic()-tick_started
            if ctrl.state=='TESTING' and ctrl.trial and getattr(ctrl.trial,'kind',None)=='friction':
                lab=ctrl.trial.reference.labels[max(ctrl.trial.last_index,0)]
                if lab['phase']=='forward':
                    key=(lab['joint'],lab['speed'],lab['repeat'])
                    if key!=last_friction_key:
                        last_friction_key=key
                        ui.status(f"摩擦采集 J{lab['joint']} {lab['speed']:g} rad/s 第{lab['repeat']}次 正向")
            elif ctrl.state=='TESTING' and getattr(ctrl.test,'ends',None) is not None:
                idx=min(int(np.searchsorted(ctrl.test.ends,max(0.,now-ctrl.path_t),side='right')),len(ctrl.test.parts)-1)
                if idx!=last_step:
                    last_step=idx;ui.status(f'移动到期望位置 {idx+1}/{len(ctrl.test.parts)}')
            else:last_step=-1
            if ctrl.active and not ctrl.shutdown_phase and ctrl.state in ('FF_RAMP_IN','FF_RAMP_OUT') and now-last_status>=1.:
                last_status=now
                scale=float(cfg.get('mit_ff_scale',0.))
                if scale>0:
                    pct=100*ctrl.mit_ff_ratio/scale
                    if 0.5<pct<99.5:
                        peak=float(np.max(np.abs(ctrl.mit_telemetry['ff_requested_Nm']))) if ctrl.mit_telemetry else 0.
                        ui.status(f'力矩渐变 {scale:.2f}  进度 {pct:.0f}%  当前最大 |FF| {peak:.2f} Nm')
            try:recorder.update(ctrl,now,s,now if a.sim else time.monotonic(),action)
            except OSError as e:log_failure=str(e);ctrl.request_stop(s,now,'Test data logging unavailable: '+str(e))
            tracking_target=ctrl.last_target if not ctrl.shutdown_phase else np.full(6,np.nan)
            record({'t_control':now,'control_tick_wall_s':last_tick_duration,'control_loop_gap_s':loop_gap,'state':ctrl.state,'shutdown_phase':ctrl.shutdown_phase,'action':action,'q_target':tracking_target,'feedback':s})
            if log_failure:ctrl.request_stop(s,now,'Logging unavailable: '+log_failure)
            if viewer and viewer.is_running():
                j=cfg['joint']-1
                if ctrl.trial:
                    tag=ctrl.trial.metadata(s,now);mass='-' if tag['mass_kg'] is None else f"{tag['mass_kg']:.3f} kg"
                    viewer.set_texts((None,None,f"SIM ONLY | {cfg['trajectory']} | {ctrl.state} | {tag['phase']}\nPose {tag['pose_id']} | load {mass} | {('J6 passive' if cfg['passive_wrist'] else '6-axis control')}\nLB+X prepare | LB+Y start/next pose | LB+RB abort",f"J2: {np.rad2deg(s['q'][1]):.2f} deg\nJ3: {np.rad2deg(s['q'][2]):.2f} deg"))
                else:
                    viewer.set_texts((None,None,'SIM ONLY | '+cfg['trajectory']+' | scale '+str(cfg['trajectory_scale'])+' | speed '+str(cfg['trajectory_speed_scale'])+' | '+ctrl.state+'\nLB+X prepare | LB+Y start | LB+RB abort',f'J{j+1}: {np.rad2deg(s["q"][j]):.2f} deg\nTarget: {np.rad2deg(ctrl.last_target[j]):.2f} deg'))
                viewer.sync()
            step+=1
            # Automatic GUI previews must retain real playback speed. Headless
            # scripted runs stay accelerated for controller regression checks.
            if not a.scripted or a.viewer:
                deadline+=dt;remaining=deadline-time.monotonic()
                if remaining>0:time.sleep(remaining)
                else:deadline=time.monotonic()
        result.update(completed=ctrl.state=='DONE',state=ctrl.state,aborted=ctrl.aborted,last_q=b.snapshot()['q'].tolist(),max_requested_velocity=cfg['max_velocity_rad_s'],stop_reason=ctrl.stop_reason)
    except Exception as e:
        result['error']=str(e);emit('STOPPED：'+str(e))
        if hasattr(e,'reference_report'):(path/'reference_plan.json').write_text(json.dumps(e.reference_report,ensure_ascii=False,indent=2)+'\n')
        if ctrl:ctrl.fault(e)
        if b and b.active:b.quick_stop(str(e))
        result['state']='FAULT' if ctrl else 'PREFLIGHT_REJECTED'
    finally:
        if recorder:
            try:recorder.close(b.snapshot()['time'],'loop_ended')
            except OSError as e:log_failure=str(e)
        if ctrl and ctrl.fault_reason:result['fault_reason']=ctrl.fault_reason
        if log_failure:result['log_error']=log_failure
        if b and ctrl and not ctrl.finished and not a.preflight_only:
            # Exceptions must not bypass the same no-powered-exit contract.
            if not ctrl.fault_reason:ctrl.fault('Unexpected loop termination')
            emit('SHUTDOWN_MONITOR：等待安全停放及六轴失能确认后，才关闭通信。')
            while not ctrl.finished:
                if a.sim:b.advance(.01)
                observation=b.snapshot();shutdown_now=observation['time'] if a.sim else time.monotonic()
                operator_exit.update(ctrl,observation,shutdown_now)
                ctrl.tick(shutdown_now,observation)
                record({'t_control':shutdown_now,'state':ctrl.state,'shutdown_phase':ctrl.shutdown_phase,'feedback':observation})
                time.sleep(.01)
        if ctrl and not a.preflight_only:
            result.update(state=ctrl.state,completed=ctrl.state=='DONE' and ctrl.disarm_confirmed and not log_failure,
                          motors_disabled_confirmed=ctrl.disarm_confirmed,shutdown_phase=ctrl.shutdown_phase,
                          operator_supported_disarm=ctrl.operator_supported_disarm,operator_power_off_confirmed=ctrl.operator_power_off_confirmed,
                          release_q_rad=None if ctrl.release_q is None else ctrl.release_q.tolist(),
                          final_enabled=b.snapshot()['enabled'].tolist(),last_q=b.snapshot()['q'].tolist())
            if ctrl.fault_reason:result['fault_reason']=ctrl.fault_reason
            if log_failure:result['log_error']=log_failure
        if viewer and ctrl and ctrl.finished:
            emit('仿真已结束，窗口保持显示。关闭窗口即可退出。')
            view_deadline=time.monotonic()+60
            while viewer.is_running() and not operator_exit.requested and time.monotonic()<view_deadline:
                viewer.sync();time.sleep(.02)
        if viewer:viewer.close()
        if pad:pad.close()
        if b:
            if not a.sim and ctrl and ctrl.operator_power_off_confirmed:b.close(allow_unconfirmed_power_off=True)
            else:b.close()
            if not a.sim:
                result.update(can_rx_dropped_total=b.rx_dropped,can_raw_log_dropped_total=b.raw_log_dropped,can_receive_buffer_bytes=b.receive_buffer_bytes)
                if b.error and 'logging' in b.error:log_failure=b.error;result.update(completed=False,log_error=log_failure)
        if recorder:
            try:
                result['test_data']=export_test_data(path)
                if ctrl and ctrl.trial and ctrl.trial.kind=='friction':
                    plan=ctrl.trial.plan
                    result['friction_protocol']={'kind':'friction','joints':plan['joint_order'],'speeds_rad_s':plan['speeds_rad_s'],'repeats':plan['repeats'],'segments':plan['segments'],'planned_duration_s':plan['duration_s'],'automatic_sequence_completed':ctrl.trial.completed,'torque_calibrated':False}
                    emit('FRICTION_SAVED：'+str(plan['segments'])+'段，计划'+f"{plan['duration_s']:.0f}"+'秒；标签（关节／速度档／方向／重复）已写入 test_signals.npz，详见 reference_plan.json。')
                elif ctrl and ctrl.trial:
                    from .validation_trials import summarize
                    report=summarize(path)
                    result['validation_protocol']={'kind':ctrl.trial.kind,'automatic_sequence_completed':ctrl.trial.phase=='complete','complete_captures':sum(x['complete'] for x in report['captures']),'partial_captures':sum(not x['complete'] for x in report['captures']),'passive_joints':ctrl.trial.passive_joints,'torque_calibrated':False}
                    emit('VALIDATION_SAVED：'+str(sum(x['complete'] for x in report['captures']))+'段完整静态采集（另含'+str(sum(not x['complete'] for x in report['captures']))+'段中断记录）；标签／姿态／温度和夹爪信息已保存，详见validation_summary.json。')
                csv_note='test_signals.csv / ' if (path/'test_signals.csv').exists() else ''
                emit('DATA_SAVED：'+str(result['test_data']['sample_count'])+' 条测试快照；'+csv_note+'test_signals.npz，详见 test_quality.json。')
            except Exception as e:
                result['data_export_error']=str(e);result['completed']=False;emit('DATA_EXPORT_FAILED：'+str(e))
        if lease:lease.close()
        for s,h in old_handlers.items():signal.signal(s,h)
        if result.get('state')=='FAULT':(path.parent/'.recovery_pending').write_text(str(path)+'\n')
        samples.close();events.close();(path/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,default=serializable));print('Run files:',path,flush=True)
    if result.get('state') in ('FAULT','PREFLIGHT_REJECTED') or result.get('log_error') or result.get('data_export_error'):raise SystemExit(2)
if __name__=='__main__':main()
