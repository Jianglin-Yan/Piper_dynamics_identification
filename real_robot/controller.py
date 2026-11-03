"""Backend-independent state machine. ABORT = controlled braking + return, not safety-rated E-stop."""
import numpy as np
from .trajectory import Excitation, ScaledTrajectory, Segment, transit, brake, check_path
from .current_pose import current_pose_bounds, ARM_POSE_TOLERANCE_DEG, HOLD_TOLERANCE_DEG
from .standing import standing_plan
from .monitoring import SpeedMonitor, CurrentMonitor
from .shutdown import Shutdown


class Controller(Shutdown):
    def __init__(self, backend, cfg, snapshot, now, emit=print):
        self.b=backend;self.c=cfg;self.emit=emit;self.home=snapshot['q'].copy();self.now=now
        self.controlled_axes=np.array([True]*5+[not cfg.get('passive_wrist',False)])
        self.passive_q=self.home[5]
        self.trial=None
        self.current_pose_only=cfg.get('current_pose_only',False)
        self.standing_start=cfg.get('standing_start',False);self.boundary_goal=None;self.entry_stable_since=None
        self.return_home=self.home.copy()
        self.start=self.home.copy() if cfg['start_deg'] is None else np.deg2rad(cfg['start_deg'])
        self.lower=np.deg2rad(cfg['lower_deg'])+np.deg2rad(cfg['margin_deg'])
        self.upper=np.deg2rad(cfg['upper_deg'])-np.deg2rad(cfg['margin_deg'])
        if self.standing_start:
            if cfg.get('passive_wrist',False) or cfg.get('hold_start_wrist',False):
                cfg['standing_start_deg']=list(cfg.get('standing_start_deg',[0.,45.,-60.,0.,0.,0.]))
                cfg['standing_start_deg'][5]=float(np.rad2deg(self.home[5]))
            self.start,self.boundary_goal,self.lower,self.upper=standing_plan(cfg,self.home)
            self.return_home=self.boundary_goal.copy()
        if self.current_pose_only:
            self.lower,self.upper=current_pose_bounds(cfg,self.home)
            self.emit('当前姿态模式：J1–J5 保持启动目标，仅 J6 运动')
        if cfg.get('passive_wrist',False):
            self.lower[5]=min(np.deg2rad(cfg['lower_deg'][5]),self.home[5]-np.deg2rad(2.))
            self.upper[5]=max(np.deg2rad(cfg['upper_deg'][5]),self.home[5]+np.deg2rad(2.))
            self.emit('J6／夹爪释放，可手调')
        if cfg.get('hold_start_wrist'):
            self.lower[5]=np.deg2rad(cfg['lower_deg'][5]);self.upper[5]=np.deg2rad(cfg['upper_deg'][5])
            if not self.lower[5]<=self.home[5]<=self.upper[5]:raise ValueError('Payload startup J6 must lie within reported joint limits to hold its angle')
            self.emit('J6 保持启动角度；夹爪释放')
        outside=(self.home<self.lower)|(self.home>self.upper)
        if np.any(outside):
            raw_lower=np.deg2rad(cfg['lower_deg']);raw_upper=np.deg2rad(cfg['upper_deg'])
            details=[]
            for j in np.flatnonzero(outside):
                kind='反馈超出配置范围' if self.home[j]<raw_lower[j] or self.home[j]>raw_upper[j] else '仅未满足程序余量'
                details.append(f'J{j+1}={np.rad2deg(self.home[j]):.3f}°, 配置 [{cfg["lower_deg"][j]}, {cfg["upper_deg"][j]}]°, 程序内部区间 [{np.rad2deg(self.lower[j]):.1f}, {np.rad2deg(self.upper[j]):.1f}]°（{kind}）')
            raise ValueError('启动检查器拒绝；这不等于固件已报机械超限：'+'；'.join(details)+'。保持原始读数，不自动钳位或改零位。')
        if self.standing_start:
            raw_lower=np.deg2rad(cfg['lower_deg']);raw_upper=np.deg2rad(cfg['upper_deg'])
            excursion=np.rad2deg(np.maximum(0.,np.maximum(raw_lower-self.home,self.home-raw_upper)))
        self.vmax=cfg['max_velocity_rad_s'];self.amax=cfg['max_acceleration_rad_s2']
        self.multisine=cfg.get('trajectory','pilot')=='multisine'
        self.test_vmax=cfg.get('test_max_velocity_rad_s',self.vmax);self.test_amax=cfg.get('test_max_acceleration_rad_s2',self.amax)
        self.braking_amax=cfg.get('braking_max_acceleration_rad_s2',self.amax)
        self.braking_from_test=False;self.last_tracking_warning=-np.inf;self.last_envelope_warning=-np.inf
        self.reference_plan=None
        if self.multisine:
            from .multisine import RepositoryMultisine,reference_report,TrajectoryRejected
            self.nominal_test=RepositoryMultisine(self.start,cfg)
            self.test=ScaledTrajectory(self.nominal_test,self.start,cfg['trajectory_scale'],cfg.get('trajectory_speed_scale',1.))
            low=np.deg2rad(cfg['lower_deg'])+np.deg2rad(cfg['margin_deg'])
            high=np.deg2rad(cfg['upper_deg'])-np.deg2rad(cfg['margin_deg'])
            self.reference_plan=reference_report(self.test,self.nominal_test,low,high,self.test_vmax,self.test_amax)
            self.emit(f'MULTISINE：六关节多正弦，幅度scale={self.test.scale:g}，速度scale={self.test.speed_scale:g}，时长{self.test.duration:g}秒，'+
                      f'峰值速度 {max(self.reference_plan["max_velocity_per_joint_rad_s"]):.3f} rad/s，'+
                      f'峰值加速度 {max(self.reference_plan["max_acceleration_per_joint_rad_s2"]):.3f} rad/s²。')
            if self.reference_plan['hard_violations']:raise TrajectoryRejected(self.reference_plan)
            if self.reference_plan['warnings']:self.emit('REFERENCE_WARNING：'+str(self.reference_plan['warnings'])+'；规划参考告警，不作急停。')
        elif cfg.get('trajectory') in ('gravity','mit'):
            self.test=Segment(self.start,self.start,cfg['duration_s']);self.nominal_test=self.test
        elif cfg.get('trajectory') in ('static','reciprocal','payload'):
            from .validation_trials import ValidationTrial
            self.trial=ValidationTrial(self.start,cfg,self.lower,self.upper)
            self.test=self.trial.reference;self.nominal_test=self.test;self.reference_plan=self.trial.plan
            self.emit('VALIDATION：'+cfg['trajectory']+'；'+('J1–J5受控，J6被动。' if cfg.get('passive_wrist') else '六轴受控。')+('正反速度档 '+str(cfg['reciprocal_speeds_rad_s'])+' rad/s。' if cfg['trajectory']=='reciprocal' else '移动上限 '+str(cfg['validation_move_velocity_rad_s'])+' rad/s。'))
            if cfg['trajectory']=='payload':self.emit('PAYLOAD：自行挂载／更换并等待；全程持续记录，不自动标重量。卸载后LB+Y直接切下一姿态，LB+RB结束。')
        else:
            self.nominal_test=Excitation(self.start,cfg['joint']-1,np.deg2rad(cfg['amplitude_deg']),cfg['frequency_hz'],cfg['duration_s'],cfg['ramp_s'])
            self.test=ScaledTrajectory(self.nominal_test,self.start,cfg.get('trajectory_scale',1.),cfg.get('trajectory_speed_scale',1.))
            self.check(self.test)
            self.emit(f'TEST_SCALE：{self.test.scale:.0%} 幅度；J{cfg["joint"]} ±{cfg["amplitude_deg"]*self.test.scale:.3f}°，速度比例 {self.test.speed_scale:g}，时长 {self.test.duration:g} 秒。')
        outbound_start=self.boundary_goal if self.standing_start else self.home
        self.outbound=transit(outbound_start,self.start,self.vmax,self.amax)
        self.inbound=transit(self.start,self.return_home,self.vmax,self.amax)
        for p in (self.outbound,self.inbound):self.check(p)
        self.state='WAITING';self.active=False;self.armed=False;self.aborted=False;self.last_target=self.home.copy()
        self.path=None;self.path_t=now;self.next_state=None;self.stable_since=None;self.settle_started=now
        self.speed_monitor=SpeedMonitor(cfg);self.current_monitor=CurrentMonitor(cfg);self.stop_reason=None;self.hold_goal=None
        self.can_stable_since=None
        self.pose_history=[];self.pose_history_state=None
        self.fault_reason=None
        self.init_shutdown()
        self.emit('WAITING：LB+X 准备，LB+Y 开始，LB+RB 中止' if not (self.trial and self.trial.kind=='payload') else 'WAITING：LB+X 准备并记录首姿态，LB+Y 切姿态，LB+RB 结束')

    def return_target(self):
        q=self.return_home.copy()
        if not self.controlled_axes[5]:q[5]=self.passive_q
        return q
    def motors_ready(self,s):
        enabled=np.asarray(s['enabled'],bool)
        return bool(np.all(enabled[self.controlled_axes]) and not np.any(enabled[~self.controlled_axes]))
    def begin_testing(self,now):
        self.set_path('TESTING',self.test,now)
        if self.trial:self.trial.begin(self,now)
    def check(self,path,vmax=None,amax=None):
        check_path(path,self.lower,self.upper,self.vmax if vmax is None else vmax,self.amax if amax is None else amax)
    def measurement_bounds(self):
        # Feedback tolerance extends the reported/configured physical bounds,
        # not the inward planning margin. Command bounds remain independent.
        allowance=np.deg2rad(self.c.get('measurement_angle_tolerance_deg',6.))
        lower=np.deg2rad(self.c['lower_deg'])-allowance
        upper=np.deg2rad(self.c['upper_deg'])+allowance
        testing=self.state=='TESTING' or (self.state=='BRAKING' and self.braking_from_test)
        if not testing:
            # Keep the separately reviewed startup corridor usable for entry/return.
            lower=np.minimum(lower,self.lower);upper=np.maximum(upper,self.upper)
        if self.current_pose_only:
            lower[:5]=self.home[:5]-np.deg2rad(HOLD_TOLERANCE_DEG)
            upper[:5]=self.home[:5]+np.deg2rad(HOLD_TOLERANCE_DEG)
        return lower,upper
    def tracking_hard_limit(self):
        return self.c.get('test_tracking_hard_deg',self.c['tracking_error_deg']) if self.multisine and (self.state=='TESTING' or (self.state=='BRAKING' and self.braking_from_test)) else self.c['tracking_error_deg']
    def event(self,text):self.emit(text)
    def set_path(self,state,path,now,note=''):
        self.path=path;self.path_t=now;self.state=state;self.stable_since=None;self.event(state+(f'  {note}' if note else ''))
    def reference_velocity(self,now):
        if self.path is not None and self.state in ('PREPARING','TESTING','BRAKING','RETURNING'):
            v=self.path.at(max(0.,now-self.path_t-1/self.c['command_hz']))[1].copy();v[~self.controlled_axes]=0.
            return v
        return np.zeros(6)
    def position_stable(self,s,now,target,tolerance_deg):
        # Judge rest from a position window, not one quantized motor-speed sample.
        if self.pose_history_state!=self.state:
            self.pose_history=[];self.pose_history_state=self.state
        self.pose_history.append((now,s['q'].copy()))
        while len(self.pose_history)>1 and self.pose_history[1][0]<=now-.5:self.pose_history.pop(0)
        if now-self.pose_history[0][0]<.5-1e-8:return False
        span=np.ptp(np.array([q for _,q in self.pose_history]),axis=0)
        return (np.max(np.abs(np.rad2deg(s['q']-target))[self.controlled_axes])<=tolerance_deg and
                np.max(np.rad2deg(span)[self.controlled_axes])<=self.c.get('settle_position_window_deg',.20))
    def fault(self,reason,part=None,measured=None):
        if self.fault_reason:return
        self.fault_reason=str(reason)
        self.state='FAULT';self.shutdown_phase='FAULT_WAITING_PARK';self.settle_started=self.now
        reason_text=str(reason)
        if part is None:part=next((f'J{n}' for n in range(1,7) if f'J{n}' in reason_text),'未细分')
        if measured is None:measured='见原因'
        action='已发送停止，未自动回位；保持监视' if self.active else '未使能，无需动作'
        ui=getattr(self,'ui',None)
        if ui is not None:ui.fault(reason_text,part=part,measured=measured,action=action,next_step='确认机械臂有支撑后按 Ctrl+C 失能；核对原因后再运行')
        else:self.event('FAULT：'+reason_text)
        if self.active:
            try:self.b.quick_stop(reason_text)
            except Exception as e:self.event('FAULT_STOP_FAILED：'+str(e))
        self.active=False
    def request_stop(self,s,now,reason):
        """Controlled stop: brake the accepted reference, return to park, then disable."""
        self.now=now
        if self.finished or self.shutdown_phase:return
        if self.state in ('BRAKING','RETURNING','SETTLING_HOME'):return
        self.stop_reason=str(reason)
        if not self.active:
            self.park_for_shutdown(now,reason);return
        try:
            self.b.check(s,True)
            if not self.motors_ready(s):raise ValueError('Unexpected motor enable state (including passive J6)')
            if s.get('status',{}).get('control_mode',1)!=1:raise ValueError('CAN position control not established')
            if np.max(np.abs(np.rad2deg(s['q']-self.last_target))[self.controlled_axes])>self.tracking_hard_limit():
                raise ValueError('tracking too far from accepted reference')
            lo,hi=self.measurement_bounds()
            if np.any(s['q']<lo) or np.any(s['q']>hi):raise ValueError('measured pose outside session hard bounds')
            v=self.reference_velocity(now)
            if self.current_pose_only:v[:5]=0.
            self.braking_from_test=self.state=='TESTING'
            stop_vmax=max(self.test_vmax,float(np.max(np.abs(v)))) if self.braking_from_test else self.vmax
            stop_amax=self.braking_amax if self.braking_from_test else self.amax
            p=brake(self.last_target.copy(),v,stop_vmax,stop_amax);self.check(p,stop_vmax,stop_amax)
            if self.state=='BOUNDARY_ENTRY':self.b.finish_native_move()
            self.set_path('BRAKING',p,now)
            self.event('停止：'+self.stop_reason+'（制动→回位→失能）')
        except Exception as e:
            self.fault('Controlled stop unavailable: '+str(e)+'; original reason: '+self.stop_reason)
    def activate(self,s,now,next_state):
        pose_tolerance=ARM_POSE_TOLERANCE_DEG if self.current_pose_only else self.c['settle_error_deg']
        if np.max(np.abs(s['q']-self.home)[self.controlled_axes])>np.deg2rad(pose_tolerance):
            raise ValueError('Robot moved after startup; restart to capture a new home pose')
        if np.max(np.abs(s['qd'])[self.controlled_axes])>self.c.get('arming_velocity_limit_rad_s',.15):raise ValueError('Robot is moving before arming')
        self.active=True;self.b.arm(self.home if self.current_pose_only else s['q']);self.state='ARMING';self.path_t=now;self.next_state=next_state
        self.event('ARMING：保持当前目标，等待J1–J5使能、J6失能反馈。' if self.c.get('passive_wrist') else 'ARMING：保持当前目标，等待六轴使能反馈。')
    def begin_ready_or_test(self,next_state,now):
        if next_state=='PREPARE' and self.standing_start and np.max(np.abs(self.boundary_goal-self.home))>1e-7:
            self.b.begin_native_move(self.boundary_goal)
            self.last_target=self.boundary_goal.copy();self.path_t=now;self.state='BOUNDARY_ENTRY';self.entry_stable_since=None
            self.event('BOUNDARY_ENTRY：固件 MOVE J 以 2% 速度进入合法边界；先确认实际运动，不开始试采。')
        elif next_state=='PREPARE':self.set_path('PREPARING',self.outbound,now,'进入准备位置')
        else:self.begin_testing(now)
    def finish_settle(self,now,s):
        target=self.start if self.state=='SETTLING_READY' else self.return_home
        self.command(target)
        stable=self.position_stable(s,now,target,self.c['settle_error_deg'])
        if stable:
            if self.state=='SETTLING_READY':
                if self.trial and self.trial.kind=='payload':
                    self.event('READY  到达准备位置（自动记录）')
                    self.begin_testing(now)
                else:self.state='READY';self.event('READY  到达准备位置')
            else:self.begin_disarm(s,now)
        elif now-self.settle_started>self.c['settle_timeout_s']:
            if self.state=='SETTLING_HOME' and self.position_stable(s,now,target,self.c.get('park_release_tolerance_deg',3.)):
                self.event('PARK_NEAR：精确到位超时，但已在允许的启动位置附近静止；记录实际差值后失能。')
                self.begin_disarm(s,now)
            else:raise ValueError('Timed out settling at target')
    def command(self,q):
        q=np.asarray(q).copy()
        if self.c.get('hold_start_wrist') and abs(q[5]-self.home[5])>1e-9:raise ValueError('Payload cannot change startup J6 target')
        if not self.controlled_axes[5]:q[5]=self.passive_q
        if self.current_pose_only and np.max(np.abs(np.asarray(q)[:5]-self.home[:5]))>1e-9:
            raise ValueError('Current-pose pilot cannot change a held-axis command')
        if np.any(q<self.lower-1e-9) or np.any(q>self.upper+1e-9):raise ValueError('command outside bounds')
        self.b.command(q);self.last_target=np.asarray(q).copy()
    def tick(self,now,s,action=None):
        self.now=now
        if not self.controlled_axes[5]:self.passive_q=float(s['q'][5])
        if self.finished:return
        if self.shutdown_phase:
            if action=='ABORT':self.event('FAULT_EXIT：本次已在收尾／故障停止；先支撑机械臂，再按 Ctrl+C 按提示失能退出。')
            self.shutdown_tick(now,s);return
        try:
            self.b.check(s,self.active)
            if self.active:
                if self.state!='ARMING' and not self.motors_ready(s):raise ValueError('Joint disabled during active session')
                errors=np.rad2deg(s['q']-self.last_target);errors[~self.controlled_axes]=0.
                if np.max(np.abs(errors))>self.tracking_hard_limit():
                    j=int(np.argmax(np.abs(errors)))
                    raise ValueError(f'tracking error exceeded: J{j+1}, target={np.rad2deg(self.last_target[j]):.3f}°, actual={np.rad2deg(s["q"][j]):.3f}°, error={errors[j]:+.3f}°')
                lo,hi=self.measurement_bounds()
                if np.any(s['q']<lo) or np.any(s['q']>hi):raise ValueError('measured angle outside session hard bounds')
                if not self.multisine and np.max(np.abs(errors))>self.c.get('tracking_warning_deg',10.) and now-self.last_tracking_warning>=1.:
                    self.last_tracking_warning=now;self.event(f'TRACKING_WARNING：跟踪误差 {np.max(np.abs(errors)):.2f}°，记录告警；独立硬线 {self.tracking_hard_limit():g}°。')
                if (np.any(s['q']<self.lower) or np.any(s['q']>self.upper)) and now-self.last_envelope_warning>=1.:
                    self.last_envelope_warning=now;self.event('ANGLE_WARNING：实测角度超出规划余量；记录告警，尚未达到独立反馈角度硬线。')
                if self.multisine and self.state=='TESTING':
                    if np.max(np.abs(errors))>self.c['test_tracking_warning_deg'] and now-self.last_tracking_warning>=1.:
                        self.last_tracking_warning=now;self.event(f'TRACKING_WARNING：跟踪误差 {np.max(np.abs(errors)):.2f}°，记录告警；独立硬线 {self.tracking_hard_limit():g}°。')
                    low=np.deg2rad(self.reference_plan['q_min_deg']);high=np.deg2rad(self.reference_plan['q_max_deg'])
                    speed_envelope=np.array(self.reference_plan['max_velocity_per_joint_rad_s'])*1.25+.15
                    exceeds=np.any(np.abs(s['qd'])>speed_envelope) or np.any(s['q']<low-np.deg2rad(2)) or np.any(s['q']>high+np.deg2rad(2))
                    if exceeds and now-self.last_envelope_warning>=1.:
                        self.last_envelope_warning=now;self.event('REFERENCE_ENVELOPE_WARNING：现在的运动比仿真参考更夸张；记录告警，不因参考超出而停止。')
                speed=self.speed_monitor.update(now,np.where(self.controlled_axes,s['qd'],0.))
                if speed['level']=='emergency':
                    raise ValueError(f'emergency overspeed: J{speed["joint"]}, feedback={speed["speed"]:+.3f} rad/s, threshold={self.speed_monitor.emergency:.3f} rad/s for {self.speed_monitor.emergency_duration:.2f} s')
                if speed['level']=='stop' and self.c.get('feedback_stop_policy')=='warn':
                    if now-self.last_envelope_warning>=1.:
                        self.last_envelope_warning=now;self.event(f'警告：持续速度 {speed["speed"]:+.3f} rad/s')
                elif speed['level']=='stop':
                    self.request_stop(s,now,f'sustained overspeed: J{speed["joint"]}, feedback={speed["speed"]:+.3f} rad/s, threshold={self.speed_monitor.stop:.3f} rad/s for {self.speed_monitor.duration:.2f} s')
                    if self.shutdown_phase:return
                elif speed['level']=='warning':
                    self.event(f'警告：J{speed["joint"]} 速度 {speed["speed"]:+.3f} rad/s')
                if 'current_A' in s:
                    current=self.current_monitor.update(now,np.where(self.controlled_axes,s['current_A'],0.))
                    if current['level']=='emergency':
                        self.request_stop(s,now,f'over-current: J{current["joint"]}, {current["current"]:+.3f} A for {self.current_monitor.emergency_duration:.2f} s')
                        if self.shutdown_phase:return
                    if current['level']=='stop' and self.c.get('feedback_stop_policy')=='warn':
                        if now-self.last_envelope_warning>=1.:
                            self.last_envelope_warning=now;self.event(f'警告：持续电流 {current["current"]:+.3f} A')
                    elif current['level']=='stop':
                        self.request_stop(s,now,f'sustained feedback current: J{current["joint"]}, feedback={current["current"]:+.3f} A')
                        if self.shutdown_phase:return
                    elif current['level']=='warning':
                        self.event(f'警告：J{current["joint"]} 电流 {current["current"]:+.3f} A')
            if action=='STOP':
                self.request_stop(s,now,'operator/input stop')
                if self.state in ('DONE','FAULT'):return
            if action=='START' and self.state not in ('WAITING','READY') and not (self.trial and self.state=='TESTING'):
                self.event('尚未到位：LB+Y 已忽略，等待 READY 后重新按下。')
            if action=='ABORT':
                self.aborted=True
                self.request_stop(s,now,'LB+RB')
                if self.shutdown_phase:return
            if self.state=='WAITING':
                if action=='PREPARE':
                    if np.max(np.abs(s['q']-self.home)[self.controlled_axes])>np.deg2rad(self.c['settle_error_deg']):raise ValueError('Startup pose changed; restart')
                    if not self.trial and np.max(np.abs(self.start-self.home)[self.controlled_axes])<1e-7:
                        self.state='READY';self.event('READY  到达准备位置（启动姿态）')
                    else:self.activate(s,now,'PREPARE')
                elif action=='START':self.event('尚未准备：先 LB+X；没有发送运动指令。')
            elif self.state=='READY':
                if self.active:self.command(self.start)
                if action=='START':
                    if np.max(np.abs(s['q']-self.start)[self.controlled_axes])>np.deg2rad(self.c.get('mit_prepare_tolerance_deg',self.c['settle_error_deg'])):raise ValueError('Not at prepared start')
                    if not self.active:self.activate(s,now,'TEST')
                    else:self.begin_testing(now)
                elif action=='PREPARE':self.event('已在 READY')
            elif self.state=='ARMING':
                self.command(self.home)
                mode_ready=s.get('status',{}).get('control_mode',1)==1
                fresh_enable=('low_times' not in s or np.min(s['low_times'])>=self.path_t)
                if self.motors_ready(s) and mode_ready and fresh_enable:
                    if self.can_stable_since is None:self.can_stable_since=now
                else:self.can_stable_since=None
                ready=(self.can_stable_since is not None and
                       ('control_mode' not in s.get('status',{}) or now-self.can_stable_since>=.10))
                if ready:
                    self.armed=True;self.begin_ready_or_test(self.next_state,now)
                elif now-self.path_t>2.:raise ValueError('Enable timeout')
            elif self.state=='BOUNDARY_ENTRY':
                self.command(self.boundary_goal)
                at_goal=self.position_stable(s,now,self.boundary_goal,.2)
                if at_goal:
                    self.b.finish_native_move();self.set_path('PREPARING',self.outbound,now,'进入准备位置')
                else:self.entry_stable_since=None
                if self.state=='BOUNDARY_ENTRY' and now-self.path_t>8.:
                    raise ValueError('Boundary entry did not reach target in 8 s; controlled-axis errors_deg='+str(np.rad2deg(s['q']-self.boundary_goal)[self.controlled_axes].round(3).tolist())+'; passive_J6='+str(bool(self.c.get('passive_wrist')))+'; position control not proven, no standing move or trial')
            elif self.state=='TESTING' and self.trial:
                self.trial.tick(self,now,s,action)
            elif self.state in ('PREPARING','TESTING','BRAKING','RETURNING'):
                if self.armed and not self.motors_ready(s):raise ValueError('Joint disabled during motion')
                elapsed=now-self.path_t;q,_,_=self.path.at(elapsed);self.command(q)
                if elapsed>=self.path.duration:
                    if self.state=='PREPARING':self.state='SETTLING_READY';self.settle_started=now;self.stable_since=None
                    elif self.state in ('TESTING','BRAKING'):
                        if self.state=='TESTING':p=self.inbound
                        else:p=transit(q,self.return_target(),self.vmax,self.amax);self.check(p)
                        self.set_path('RETURNING',p,now,'开始回位')
                    else:self.state='SETTLING_HOME';self.settle_started=now;self.stable_since=None
            elif self.state in ('SETTLING_READY','SETTLING_HOME'):self.finish_settle(now,s)
        except Exception as e:self.fault(e)
