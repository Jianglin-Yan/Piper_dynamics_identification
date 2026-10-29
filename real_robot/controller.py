"""Backend-independent state machine. ABORT = controlled braking + return, not safety-rated E-stop."""
import numpy as np
from .trajectory import Excitation, transit, brake, check_path
from .current_pose import current_pose_bounds, ARM_POSE_TOLERANCE_DEG
from .standing import standing_plan
from .monitoring import SpeedMonitor, CurrentMonitor


class Controller:
    def __init__(self, backend, cfg, snapshot, now, emit=print):
        self.b=backend;self.c=cfg;self.emit=emit;self.home=snapshot['q'].copy();self.now=now
        self.current_pose_only=cfg.get('current_pose_only',False)
        self.standing_start=cfg.get('standing_start',False);self.boundary_goal=None;self.entry_stable_since=None
        self.return_home=self.home.copy()
        self.start=self.home.copy() if cfg['start_deg'] is None else np.deg2rad(cfg['start_deg'])
        self.lower=np.deg2rad(cfg['lower_deg'])+np.deg2rad(cfg['margin_deg'])
        self.upper=np.deg2rad(cfg['upper_deg'])-np.deg2rad(cfg['margin_deg'])
        if self.standing_start:
            self.start,self.boundary_goal,self.lower,self.upper=standing_plan(cfg,self.home)
            self.return_home=self.boundary_goal.copy()
            self.emit('STANDING：LB+X 将先进入合法边界小过渡点，再平滑移到 '+str(np.rad2deg(self.start).round(3).tolist())+'°；到位后才允许 LB+Y。')
            difference=np.rad2deg(self.return_home-self.home)
            if np.max(np.abs(difference))>1e-8:
                self.emit('RETURN_HOME：原始启动值保留；实际回位到最近合法点 '+str(np.rad2deg(self.return_home).round(3).tolist())+'°；相对原始启动值差 '+str(difference.round(3).tolist())+'°。')
        if self.current_pose_only:
            self.lower,self.upper=current_pose_bounds(cfg,self.home)
            self.emit('CURRENT_POSE：已锚定启动姿态，仅 J6 运动；J1–J5 保持原目标，允许测量漂移不超过 0.5°。')
        outside=(self.home<self.lower)|(self.home>self.upper)
        if np.any(outside):
            raw_lower=np.deg2rad(cfg['lower_deg']);raw_upper=np.deg2rad(cfg['upper_deg'])
            details=[]
            for j in np.flatnonzero(outside):
                kind='反馈超出配置范围' if self.home[j]<raw_lower[j] or self.home[j]>raw_upper[j] else '仅未满足程序余量'
                details.append(f'J{j+1}={np.rad2deg(self.home[j]):.3f}°, 配置 [{cfg["lower_deg"][j]}, {cfg["upper_deg"][j]}]°, 程序内部区间 [{np.rad2deg(self.lower[j]):.1f}, {np.rad2deg(self.upper[j]):.1f}]°（{kind}）')
            raise ValueError('启动检查器拒绝；这不等于固件已报机械超限：'+'；'.join(details)+'。保持原始读数，不自动钳位或改零位。')
        self.vmax=cfg['max_velocity_rad_s'];self.amax=cfg['max_acceleration_rad_s2']
        self.test=Excitation(self.start,cfg['joint']-1,np.deg2rad(cfg['amplitude_deg']),cfg['frequency_hz'],cfg['duration_s'],cfg['ramp_s'])
        outbound_start=self.boundary_goal if self.standing_start else self.home
        self.outbound=transit(outbound_start,self.start,self.vmax,self.amax)
        self.inbound=transit(self.start,self.return_home,self.vmax,self.amax)
        for p in (self.test,self.outbound,self.inbound):self.check(p)
        self.state='WAITING';self.active=False;self.armed=False;self.aborted=False;self.last_target=self.home.copy()
        self.path=None;self.path_t=now;self.next_state=None;self.stable_since=None;self.settle_started=now
        self.speed_monitor=SpeedMonitor(cfg);self.current_monitor=CurrentMonitor(cfg);self.stop_reason=None;self.hold_goal=None
        self.can_stable_since=None
        self.pose_history=[];self.pose_history_state=None
        self.fault_reason=None
        self.emit('WAITING：启动位置已保存。LB+X 准备；LB+Y 仅在 READY 后开始；LB+RB 中止。')

    def check(self,path):check_path(path,self.lower,self.upper,self.vmax,self.amax)
    def event(self,text):self.emit(text)
    def set_path(self,state,path,now):
        self.path=path;self.path_t=now;self.state=state;self.stable_since=None;self.event(state)
    def reference_velocity(self,now):
        if self.path is not None and self.state in ('PREPARING','TESTING','BRAKING','RETURNING','BRAKING_HOLD'):
            return self.path.at(max(0.,now-self.path_t-1/self.c['command_hz']))[1]
        return np.zeros(6)
    def position_stable(self,s,now,target,tolerance_deg):
        # Judge rest from a position window, not one quantized motor-speed sample.
        if self.pose_history_state!=self.state:
            self.pose_history=[];self.pose_history_state=self.state
        self.pose_history.append((now,s['q'].copy()))
        while len(self.pose_history)>1 and self.pose_history[1][0]<=now-.5:self.pose_history.pop(0)
        if now-self.pose_history[0][0]<.5-1e-8:return False
        span=np.ptp(np.array([q for _,q in self.pose_history]),axis=0)
        return (np.max(np.abs(np.rad2deg(s['q']-target)))<=tolerance_deg and
                np.max(np.rad2deg(span))<=self.c.get('settle_position_window_deg',.20))
    def fault(self,reason):
        if self.state=='FAULT':return
        self.fault_reason=str(reason)
        self.state='FAULT';self.event('FAULT：'+str(reason)+'；电子急停可能使抬起的机械臂阻尼回落，禁止自动回位/恢复。')
        if self.active:self.b.quick_stop(str(reason))
        self.active=False
    def request_stop(self,s,now,reason):
        """Brake the accepted reference and retain position control when feedback is healthy.

        Use reference velocity, not a noisy motor-speed sample, so the stop cannot
        introduce an acceleration toward the reported speed transient.
        """
        if self.state in ('DONE','FAULT','HELD'):return
        if self.state in ('BRAKING_HOLD','SETTLING_HOLD'):return
        self.stop_reason=str(reason)
        if not self.active:
            self.state='DONE';self.event('DONE：等待阶段已停止，没有发送运动指令。');return
        try:
            self.b.check(s,True)
            if not np.all(s['enabled']):raise ValueError('incomplete motor enable')
            if s.get('status',{}).get('control_mode',1)!=1:raise ValueError('CAN position control not established')
            if np.max(np.abs(np.rad2deg(s['q']-self.last_target)))>self.c['tracking_error_deg']:
                raise ValueError('tracking too far from accepted reference')
            if np.any(s['q']<self.lower) or np.any(s['q']>self.upper):raise ValueError('measured pose outside session bounds')
            v=self.reference_velocity(now)
            if self.current_pose_only:v[:5]=0.
            p=brake(self.last_target.copy(),v,self.vmax,self.amax);self.check(p)
            if self.state=='BOUNDARY_ENTRY':self.b.finish_native_move()
            self.set_path('BRAKING_HOLD',p,now)
            self.event('CONTROLLED_STOP：'+self.stop_reason+'；取消轨迹，制动后保持位置，不回位、不发送电子急停。')
        except Exception as e:
            self.fault('Controlled stop unavailable: '+str(e)+'; original reason: '+self.stop_reason)
    def activate(self,s,now,next_state):
        pose_tolerance=ARM_POSE_TOLERANCE_DEG if self.current_pose_only else self.c['settle_error_deg']
        if np.max(np.abs(s['q']-self.home))>np.deg2rad(pose_tolerance):
            raise ValueError('Robot moved after startup; restart to capture a new home pose')
        if np.max(np.abs(s['qd']))>self.c.get('arming_velocity_limit_rad_s',.15):raise ValueError('Robot is moving before arming')
        self.active=True;self.b.arm(self.home if self.current_pose_only else s['q']);self.state='ARMING';self.path_t=now;self.next_state=next_state
        self.event('ARMING：保持当前目标，等待六轴使能反馈。')
    def begin_ready_or_test(self,next_state,now):
        if next_state=='PREPARE' and self.standing_start and np.max(np.abs(self.boundary_goal-self.home))>1e-7:
            self.b.begin_native_move(self.boundary_goal)
            self.last_target=self.boundary_goal.copy();self.path_t=now;self.state='BOUNDARY_ENTRY';self.entry_stable_since=None
            self.event('BOUNDARY_ENTRY：固件 MOVE J 以 2% 速度进入合法边界；先确认实际运动，不开始试采。')
        elif next_state=='PREPARE':self.set_path('PREPARING',self.outbound,now)
        else:self.set_path('TESTING',self.test,now)
    def finish_settle(self,now,s):
        target=self.start if self.state=='SETTLING_READY' else self.return_home
        self.command(target)
        stable=self.position_stable(s,now,target,self.c['settle_error_deg'])
        if stable:
            if self.state=='SETTLING_READY':self.state='READY';self.event('READY：已到测试起点。松开按键后 LB+Y 开始。')
            else:
                self.state='DONE';self.active=False;self.b.finish()
                label='最近合法回位点' if np.max(np.abs(self.return_home-self.home))>1e-8 else '程序启动位置'
                self.event('DONE：已回到'+label+'，试采结束；保持使能，不自动卸力。再次运行无需因正常结束而重置。')
        else:self.stable_since=None
        if not stable and now-self.settle_started>self.c['settle_timeout_s']:raise ValueError('Timed out settling at target')
    def command(self,q):
        if self.current_pose_only and np.max(np.abs(np.asarray(q)[:5]-self.home[:5]))>1e-9:
            raise ValueError('Current-pose pilot cannot change a held-axis command')
        if np.any(q<self.lower-1e-9) or np.any(q>self.upper+1e-9):raise ValueError('command outside bounds')
        self.b.command(q);self.last_target=np.asarray(q).copy()
    def tick(self,now,s,action=None):
        if self.state in ('DONE','FAULT','HELD'):return
        try:
            self.b.check(s,self.active)
            if self.active:
                if self.state!='ARMING' and not np.all(s['enabled']):raise ValueError('Joint disabled during active session')
                errors=np.rad2deg(s['q']-self.last_target)
                if np.max(np.abs(errors))>self.c['tracking_error_deg']:
                    j=int(np.argmax(np.abs(errors)))
                    raise ValueError(f'tracking error exceeded: J{j+1}, target={np.rad2deg(self.last_target[j]):.3f}°, actual={np.rad2deg(s["q"][j]):.3f}°, error={errors[j]:+.3f}°')
                if np.any(s['q']<self.lower) or np.any(s['q']>self.upper):raise ValueError('measured angle outside safety interior')
                speed=self.speed_monitor.update(now,s['qd'])
                if speed['level']=='emergency':
                    raise ValueError(f'emergency overspeed: J{speed["joint"]}, feedback={speed["speed"]:+.3f} rad/s, threshold={self.speed_monitor.emergency:.3f} rad/s')
                if speed['level']=='stop':
                    self.request_stop(s,now,f'sustained overspeed: J{speed["joint"]}, feedback={speed["speed"]:+.3f} rad/s, threshold={self.speed_monitor.stop:.3f} rad/s for {self.speed_monitor.duration:.2f} s')
                    if self.state=='FAULT':return
                elif speed['level']=='warning':
                    self.event(f'SPEED_WARNING：J{speed["joint"]}={speed["speed"]:+.3f} rad/s；记录瞬时反馈，不卸力。')
                if 'current_A' in s:
                    current=self.current_monitor.update(now,s['current_A'])
                    if current['level']=='emergency':raise ValueError(f'emergency current: J{current["joint"]}, feedback={current["current"]:+.3f} A')
                    if current['level']=='stop':
                        self.request_stop(s,now,f'sustained feedback current: J{current["joint"]}, feedback={current["current"]:+.3f} A')
                        if self.state=='FAULT':return
                    elif current['level']=='warning':
                        self.event(f'CURRENT_WARNING：J{current["joint"]}={current["current"]:+.3f} A；记录短时越阈值，不卸力。')
            if action=='STOP':
                self.request_stop(s,now,'operator/input stop')
                if self.state in ('DONE','FAULT'):return
            if action=='START' and self.state not in ('WAITING','READY'):
                self.event('尚未到位：LB+Y 已忽略，等待 READY 后重新按下。')
            if action=='ABORT':
                if self.state=='BOUNDARY_ENTRY':self.b.finish_native_move()
                self.aborted=True
                if not self.active:
                    self.state='DONE';self.event('DONE：等待阶段已中止，没有发送运动指令。');return
                if not np.all(s['enabled']):raise ValueError('Abort during incomplete enable: quick stop, no return')
                brake_q=s['q'].copy();brake_v=self.reference_velocity(now)
                if self.current_pose_only:
                    brake_q[:5]=self.home[:5];brake_v[:5]=0.
                p=brake(brake_q,brake_v,self.vmax,self.amax);self.check(p)
                self.set_path('BRAKING',p,now)
                self.event('LB+RB：立即取消激励，受限平滑制动后回到已显示的回位点。')
            if self.state=='WAITING':
                if action=='PREPARE':
                    if np.max(np.abs(s['q']-self.home))>np.deg2rad(self.c['settle_error_deg']):raise ValueError('Startup pose changed; restart')
                    if np.max(np.abs(self.start-self.home))<1e-7:
                        self.state='READY';self.event('READY：使用启动姿态，无需移动。松开按键后 LB+Y 开始。')
                    else:self.activate(s,now,'PREPARE')
                elif action=='START':self.event('尚未准备：先 LB+X；没有发送运动指令。')
            elif self.state=='READY':
                if self.active:self.command(self.start)
                if action=='START':
                    if np.max(np.abs(s['q']-self.start))>np.deg2rad(self.c['settle_error_deg']):raise ValueError('Not at prepared start')
                    if not self.active:self.activate(s,now,'TEST')
                    else:self.set_path('TESTING',self.test,now)
                elif action=='PREPARE':self.event('READY：已经准备，无需重复移动。')
            elif self.state=='ARMING':
                self.command(self.home)
                mode_ready=s.get('status',{}).get('control_mode',1)==1
                fresh_enable=('low_times' not in s or np.min(s['low_times'])>=self.path_t)
                if np.all(s['enabled']) and mode_ready and fresh_enable:
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
                    self.b.finish_native_move();self.set_path('PREPARING',self.outbound,now)
                else:self.entry_stable_since=None
                if self.state=='BOUNDARY_ENTRY' and now-self.path_t>8.:
                    raise ValueError('Boundary entry did not reach target in 8 s; position control not proven, no standing move or trial')
            elif self.state=='BRAKING_HOLD':
                elapsed=now-self.path_t;q,_,_=self.path.at(elapsed);self.command(q)
                if elapsed>=self.path.duration:
                    self.hold_goal=q.copy();self.state='SETTLING_HOLD';self.settle_started=now;self.stable_since=None
            elif self.state=='SETTLING_HOLD':
                self.command(self.hold_goal)
                stable=self.position_stable(s,now,self.hold_goal,self.c['settle_error_deg'])
                if stable:
                    self.state='HELD';self.active=False;self.b.finish()
                    self.event('HELD：制动并到位，已发送位置保持目标；保持使能退出，不自动卸力或回位。')
                else:self.stable_since=None
                if self.state!='HELD' and now-self.settle_started>self.c['settle_timeout_s']:
                    raise ValueError('Could not confirm controlled hold')
            elif self.state in ('PREPARING','TESTING','BRAKING','RETURNING'):
                if self.armed and not np.all(s['enabled']):raise ValueError('Joint disabled during motion')
                elapsed=now-self.path_t;q,_,_=self.path.at(elapsed);self.command(q)
                if elapsed>=self.path.duration:
                    if self.state=='PREPARING':self.state='SETTLING_READY';self.settle_started=now;self.stable_since=None
                    elif self.state in ('TESTING','BRAKING'):
                        if self.state=='TESTING':p=self.inbound
                        else:p=transit(q,self.return_home,self.vmax,self.amax);self.check(p)
                        self.set_path('RETURNING',p,now)
                    else:self.state='SETTLING_HOME';self.settle_started=now;self.stable_since=None
            elif self.state in ('SETTLING_READY','SETTLING_HOME'):self.finish_settle(now,s)
        except Exception as e:self.fault(e)
