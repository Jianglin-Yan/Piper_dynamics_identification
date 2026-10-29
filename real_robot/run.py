"""python -m real_robot.run --sim / --hardware / --read-only / --calibrate-pad"""
import argparse, datetime, json, signal, time
from pathlib import Path
import numpy as np
from .controller import Controller
from .gamepad import Gamepad, calibrate
from .monitoring import validate_speed_limits
from .test_data import TestRecorder, export_test_data

BASE=Path(__file__).parent

def serializable(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)

def validate_config(c):
    validate_speed_limits(c)
    c.setdefault('trajectory_scale',1.)
    if not np.isfinite(c['trajectory_scale']) or not 0<c['trajectory_scale']<=1:raise ValueError('trajectory_scale must be in (0, 1]')
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
    if c['model']!='piper_x':raise ValueError('Only nominal piper_x preview bundled; confirm actual model first')
    if c['command_hz']!=100:raise ValueError('First pilot profile uses 100 Hz commands')
    if c['max_velocity_rad_s']>.2 or c['max_acceleration_rad_s2']>.3 or c['amplitude_deg']>5 or c['frequency_hz']>.3:
        raise ValueError('Outside small single-joint pilot profile')
    if not 1<=c['move_speed_percent']<=20:raise ValueError('Pilot move speed percentage must be 1..20')
    if c['duration_s']>120 or c['duration_s']<2*c['ramp_s']:raise ValueError('Invalid trial duration')

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    mode=ap.add_mutually_exclusive_group(required=True)
    for flag in ('sim','hardware','read-only','calibrate-pad','monitor-pad','recover'):mode.add_argument('--'+flag,action='store_true')
    ap.add_argument('--supported',action='store_true',help='explicit recovery only: confirm the folded arm is physically supported for power loss')
    ap.add_argument('--config',type=Path,default=BASE/'config.json');ap.add_argument('--pad',type=Path,default=BASE/'g8.json')
    ap.add_argument('--standing',action='store_true',help='LB+X performs boundary entry then moves to a fixed standing test pose')
    ap.add_argument('--from-current',action='store_true',help='J6-only trial anchored at actual startup pose; limited parked J2/J3 exceptions')
    ap.add_argument('--scale',type=float,help='test displacement fraction (0,1]; preparation, return, time and frequency unchanged')
    ap.add_argument('--preflight-only',action='store_true',help='validate the hardware session without reading button actions or enabling motors')
    ap.add_argument('--viewer',action='store_true');ap.add_argument('--scripted',choices=['normal','abort','speed-stop'],help='simulation only; speed-stop injects speed feedback to preview a controlled stop')
    ap.add_argument('--output',type=Path,default=BASE.parent/'runs');ap.add_argument('--max-seconds',type=float,default=120)
    a=ap.parse_args()
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
    cfg=json.loads(a.config.read_text());cfg['current_pose_only']=a.from_current;cfg['standing_start']=a.standing
    if a.scale is not None:cfg['trajectory_scale']=a.scale
    if a.standing:
        from .standing import STANDING_DEG
        cfg['start_deg']=cfg.get('standing_start_deg',STANDING_DEG)
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
    def record(row):
        nonlocal log_failure
        try:samples.write(json.dumps(row,default=serializable)+'\n')
        except OSError as error:log_failure=str(error)
    stop_requested=False
    def stop(signum,frame):
        nonlocal stop_requested
        # Operator stop follows the same braking/park/disable sequence as LB+RB.
        stop_requested=True
    old_handlers={s:signal.signal(s,stop) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        if a.sim:
            from .simulation import Simulation
            b=Simulation(cfg);emit('SIMULATION：名义 X 模型，简化几何；未连接机械臂。仿真增益不是实机建议值。')
        else:
            from .hardware import Hardware
            if a.hardware or a.recover:
                import fcntl, re
                if not re.fullmatch(r'[A-Za-z0-9_-]+',cfg['channel']):raise ValueError('Invalid CAN channel name')
                lease=open('/tmp/piper-pilot-'+cfg['channel']+'.lock','w')
                fcntl.flock(lease,fcntl.LOCK_EX|fcntl.LOCK_NB)
            if a.hardware:
                from .session_history import previous_controller_session, require_recovered
                history=previous_controller_session([BASE.parent/'runs',a.output],cfg['channel'],exclude=path)
                result['previous_controller_session']=history
                if history:
                    emit('STARTUP_HISTORY：'+history['reason']+' 记录：'+history['run'])
                if not a.preflight_only:require_recovered(history)
            b=Hardware(cfg,path);info=b.discover();(path/'hardware.json').write_text(json.dumps(info,ensure_ascii=False,indent=2));emit('只读查询完成：'+next((x for x in info['firmware'].split('\x00') if x.startswith('S-V')),'未知固件')+'；详细参数已保存 hardware.json')
            s=b.snapshot();b.check(s,False)
            if a.recover:
                if 'S-V1.7-3250618' not in info['firmware']:raise ValueError('Recovery not reviewed for this firmware')
                if np.max(np.abs(s['qd']))>.1:raise ValueError('Robot moving: recovery refused')
                result['before']=s
                emit('RECOVER：已明确确认机械臂蜷缩有支撑；仅发送一次恢复/重置，不使能、不发送目标。')
                b.reset_supported();time.sleep(1.);after=b.snapshot();result['after']=after
                if np.any(after['enabled']) or after['status'].get('arm_status')!=0 or after['status'].get('error_code')!=0 or any(after['faults']):
                    raise RuntimeError('恢复未确认成功；不自动重试。')
                emit('RECOVERED：六轴已失能、故障码为零。恢复命令结束；运动需要另行运行并由手柄触发。')
                result.update(completed=True,state='RECOVERED');return
            if a.read_only:
                samples.write(json.dumps(s,default=serializable)+'\n');result.update(completed=True,state='READ_ONLY');return
            b.preflight(info)
            emit(f'反馈监测：速度 {cfg["feedback_speed_warning_rad_s"]:.2f} rad/s 告警，{cfg["feedback_speed_stop_rad_s"]:.2f} rad/s 持续 {cfg["feedback_speed_stop_duration_s"]:.2f} s 制动回位并失能；严重故障仍走电子急停并监视收尾。')
            if a.from_current:
                emit('FROM_CURRENT：按操作者指定使用当前姿态，只进行 J6 有界试采；不修改限位/零位/保护等级。')
            if a.from_current or a.standing:
                emit('当前碰撞检测等级：'+str(info['collision']['collision_levels']))
            (path/'effective_config.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2))
        if not a.scripted and not a.preflight_only:pad=Gamepad(a.pad)
        if a.viewer:
            if not a.sim:raise ValueError('viewer is for simulation only')
            import mujoco.viewer
            viewer=mujoco.viewer.launch_passive(b.model,b.data)
            viewer.cam.distance=1.3;viewer.cam.lookat[:]=[0,0,.3];viewer.cam.azimuth=135;viewer.cam.elevation=-25
        s=b.snapshot();now=0. if a.sim else time.monotonic();ctrl=Controller(b,cfg,s,now,emit)
        result['home_rad']=ctrl.home.tolist()
        result['return_home_rad']=ctrl.return_home.tolist()
        result['current_pose_only']=a.from_current
        result['standing_start']=a.standing
        (path/'session_policy.json').write_text(json.dumps({'current_pose_only':a.from_current,'home_deg':np.rad2deg(ctrl.home).tolist(),'return_home_deg':np.rad2deg(ctrl.return_home).tolist(),'return_home_delta_deg':np.rad2deg(ctrl.return_home-ctrl.home).tolist(),'measurement_lower_deg':np.rad2deg(ctrl.lower).tolist(),'measurement_upper_deg':np.rad2deg(ctrl.upper).tolist(),'held_command_joints':[1,2,3,4,5] if a.from_current else [],'moving_joint':cfg['joint'],'shutdown':'park_then_confirm_six_axis_disabled','park_release_tolerance_deg':cfg['park_release_tolerance_deg'],'disable_confirm_s':cfg['disable_confirm_s']},ensure_ascii=False,indent=2))
        emit('启动角度（度）：'+str(np.round(np.rad2deg(ctrl.home),3).tolist()))
        if a.preflight_only:
            samples.write(json.dumps({'feedback':s,'home_rad':ctrl.home},default=serializable)+'\n')
            result.update(completed=True,state='PREFLIGHT_ONLY')
            return
        recorder=TestRecorder(path,cfg,'sim' if a.sim else 'hardware')
        emit('DATA：只将 TESTING 写入 test_samples.jsonl；原始测试帧和导出数据在收尾后生成。')
        dt=1/cfg['command_hz'];deadline=time.monotonic();start_wall=deadline;step=0;last_wall=deadline;scripted_started=False;scripted_aborted=False;timeout_handled=False;input_failed=False;viewer_closed=False
        while not ctrl.finished:
            now=step*dt if a.sim else time.monotonic()
            if viewer and not viewer.is_running() and not viewer_closed:
                viewer_closed=True;ctrl.request_stop(b.snapshot(),now,'Viewer closed')
            action=None
            if pad and not input_failed:
                try:action=pad.poll(time.monotonic())
                except OSError as e:
                    input_failed=True;action='STOP';emit('INPUT_STOP：'+str(e))
            if a.scripted:
                if step==10:action='PREPARE'
                elif ctrl.state=='READY' and not scripted_started:action='START';scripted_started=True
                elif a.scripted=='abort' and ctrl.state=='TESTING' and now-ctrl.path_t>=5.2 and not scripted_aborted:
                    action='ABORT';scripted_aborted=True
            if action:emit('手柄指令：'+action)
            if a.sim:b.advance(dt)
            s=b.snapshot()
            now=s['time'] if a.sim else time.monotonic()
            if a.scripted=='speed-stop' and ctrl.state=='TESTING' and 5.2<=now-ctrl.path_t<5.5:
                # Simulation-only sensor injection; never changes motor position or real CAN data.
                s['qd']=s['qd'].copy();s['qd'][cfg['joint']-1]=cfg['feedback_speed_stop_rad_s']+.05
            if stop_requested:
                stop_requested=False;ctrl.request_stop(s,now,'SIGINT/SIGTERM')
            if time.monotonic()-start_wall>a.max_seconds and not timeout_handled:
                timeout_handled=True;ctrl.request_stop(s,now,'Session timeout')
            if not a.sim and ctrl.active and time.monotonic()-last_wall>cfg['loop_watchdog_s']:ctrl.fault('Control loop late')
            last_wall=time.monotonic();ctrl.tick(now,s,action)
            try:recorder.update(ctrl,now,s,now if a.sim else time.monotonic(),action)
            except OSError as e:log_failure=str(e);ctrl.request_stop(s,now,'Test data logging unavailable: '+str(e))
            tracking_target=ctrl.last_target if not ctrl.shutdown_phase else np.full(6,np.nan)
            record({'t_control':now,'state':ctrl.state,'shutdown_phase':ctrl.shutdown_phase,'action':action,'q_target':tracking_target,'feedback':s})
            if log_failure:ctrl.request_stop(s,now,'Logging unavailable: '+log_failure)
            if viewer and viewer.is_running():
                j=cfg['joint']-1
                viewer.set_texts((None,None,'SIM ONLY | '+ctrl.state+'\nLB+X prepare | LB+Y start | LB+RB abort',f'J{j+1}: {np.rad2deg(s["q"][j]):.2f} deg\nTarget: {np.rad2deg(ctrl.last_target[j]):.2f} deg'))
                viewer.sync()
            step+=1
            if not a.scripted:
                deadline+=dt;remaining=deadline-time.monotonic()
                if remaining>0:time.sleep(remaining)
                else:deadline=time.monotonic()
        result.update(completed=ctrl.state=='DONE',state=ctrl.state,aborted=ctrl.aborted,last_q=b.snapshot()['q'].tolist(),max_requested_velocity=cfg['max_velocity_rad_s'],stop_reason=ctrl.stop_reason)
    except Exception as e:
        result['error']=str(e);emit('STOPPED：'+str(e))
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
                ctrl.tick(shutdown_now,observation)
                record({'t_control':shutdown_now,'state':ctrl.state,'shutdown_phase':ctrl.shutdown_phase,'feedback':observation})
                time.sleep(.01)
        if ctrl and not a.preflight_only:
            result.update(state=ctrl.state,completed=ctrl.state=='DONE' and ctrl.disarm_confirmed and not log_failure,
                          motors_disabled_confirmed=ctrl.disarm_confirmed,shutdown_phase=ctrl.shutdown_phase,
                          release_q_rad=None if ctrl.release_q is None else ctrl.release_q.tolist(),
                          final_enabled=b.snapshot()['enabled'].tolist(),last_q=b.snapshot()['q'].tolist())
            if ctrl.fault_reason:result['fault_reason']=ctrl.fault_reason
            if log_failure:result['log_error']=log_failure
        if viewer and ctrl and ctrl.finished:
            emit('仿真已结束，窗口保持显示。关闭窗口即可退出。')
            view_deadline=time.monotonic()+60
            while viewer.is_running() and not stop_requested and time.monotonic()<view_deadline:
                viewer.sync();time.sleep(.02)
        if viewer:viewer.close()
        if pad:pad.close()
        if b:b.close()
        if recorder:
            try:
                result['test_data']=export_test_data(path)
                emit('DATA_SAVED：'+str(result['test_data']['sample_count'])+' 条测试快照；test_signals.csv / test_signals.npz，详见 test_quality.json。')
            except Exception as e:
                result['data_export_error']=str(e);result['completed']=False;emit('DATA_EXPORT_FAILED：'+str(e))
        if lease:lease.close()
        for s,h in old_handlers.items():signal.signal(s,h)
        samples.close();events.close();(path/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,default=serializable));print('Run files:',path,flush=True)
    if result.get('state') in ('FAULT','PREFLIGHT_REJECTED') or result.get('log_error') or result.get('data_export_error'):raise SystemExit(2)
if __name__=='__main__':main()
