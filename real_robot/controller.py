"""Backend-independent state machine. ABORT = controlled braking + return, not safety-rated E-stop."""
import numpy as np
from .trajectory import Excitation, transit, brake, check_path
from .current_pose import current_pose_bounds, ARM_POSE_TOLERANCE_DEG
from .standing import standing_plan


class Controller:
    def __init__(self, backend, cfg, snapshot, now, emit=print):
        self.b=backend;self.c=cfg;self.emit=emit;self.home=snapshot['q'].copy();self.now=now
        self.current_pose_only=cfg.get('current_pose_only',False)
        self.standing_start=cfg.get('standing_start',False);self.boundary_goal=None;self.entry_stable_since=None
        self.start=self.home.copy() if cfg['start_deg'] is None else np.deg2rad(cfg['start_deg'])
        self.lower=np.deg2rad(cfg['lower_deg'])+np.deg2rad(cfg['margin_deg'])
        self.upper=np.deg2rad(cfg['upper_deg'])-np.deg2rad(cfg['margin_deg'])
        if self.standing_start:
            self.start,self.boundary_goal,self.lower,self.upper=standing_plan(cfg,self.home)
            self.emit('STANDING：LB+X 将先进入合法边界小过渡点，再平滑移到 '+str(np.rad2deg(self.start).round(3).tolist())+'°；到位后才允许 LB+Y。')
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
        self.inbound=transit(self.start,self.home,self.vmax,self.amax)
        for p in (self.test,self.outbound,self.inbound):self.check(p)
        self.state='WAITING';self.active=False;self.armed=False;self.aborted=False;self.last_target=self.home.copy()
        self.path=None;self.path_t=now;self.next_state=None;self.stable_since=None;self.settle_started=now
        self.emit('WAITING：启动位置已保存。LB+X 准备；LB+Y 仅在 READY 后开始；LB+RB 中止。')

    def check(self,path):check_path(path,self.lower,self.upper,self.vmax,self.amax)
    def event(self,text):self.emit(text)
    def set_path(self,state,path,now):
        self.path=path;self.path_t=now;self.state=state;self.stable_since=None;self.event(state)
    def fault(self,reason):
        if self.state=='FAULT':return
        self.state='FAULT';self.event('FAULT：'+str(reason)+'；禁止自动回位/恢复。')
        if self.active:self.b.quick_stop(str(reason))
        self.active=False
    def activate(self,s,now,next_state):
        pose_tolerance=ARM_POSE_TOLERANCE_DEG if self.current_pose_only else self.c['settle_error_deg']
        if np.max(np.abs(s['q']-self.home))>np.deg2rad(pose_tolerance):
            raise ValueError('Robot moved after startup; restart to capture a new home pose')
        if np.max(np.abs(s['qd']))>self.c['settle_velocity_rad_s']:raise ValueError('Robot is moving before arming')
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
        target=self.start if self.state=='SETTLING_READY' else self.home
        self.command(target)
        stable=(np.max(np.abs(s['q']-target))<=np.deg2rad(self.c['settle_error_deg']) and np.max(np.abs(s['qd']))<=self.c['settle_velocity_rad_s'])
        if stable:
            if self.stable_since is None:self.stable_since=now
            if now-self.stable_since>=.5:
                if self.state=='SETTLING_READY':self.state='READY';self.event('READY：已到测试起点。松开按键后 LB+Y 开始。')
                else:
                    self.state='DONE';self.active=False;self.b.finish()
                    self.event('DONE：已回到程序启动位置，试采结束；保持使能，不自动卸力。')
        else:self.stable_since=None
        if now-self.settle_started>self.c['settle_timeout_s']:raise ValueError('Timed out settling at target')
    def command(self,q):
        if self.current_pose_only and np.max(np.abs(np.asarray(q)[:5]-self.home[:5]))>1e-9:
            raise ValueError('Current-pose pilot cannot change a held-axis command')
        if np.any(q<self.lower-1e-9) or np.any(q>self.upper+1e-9):raise ValueError('command outside bounds')
        self.b.command(q);self.last_target=np.asarray(q).copy()
    def tick(self,now,s,action=None):
        if self.state in ('DONE','FAULT'):return
        try:
            self.b.check(s,self.active)
            if self.active:
                if self.state!='ARMING' and not np.all(s['enabled']):raise ValueError('Joint disabled during active session')
                errors=np.rad2deg(s['q']-self.last_target)
                if np.max(np.abs(errors))>self.c['tracking_error_deg']:
                    j=int(np.argmax(np.abs(errors)))
                    raise ValueError(f'tracking error exceeded: J{j+1}, target={np.rad2deg(self.last_target[j]):.3f}°, actual={np.rad2deg(s["q"][j]):.3f}°, error={errors[j]:+.3f}°')
                if np.any(s['q']<self.lower) or np.any(s['q']>self.upper):raise ValueError('measured angle outside safety interior')
                if np.max(np.abs(s['qd']))>self.vmax*1.5:raise ValueError('measured overspeed')
            if action=='START' and self.state not in ('WAITING','READY'):
                self.event('尚未到位：LB+Y 已忽略，等待 READY 后重新按下。')
            if action=='ABORT':
                if self.state=='BOUNDARY_ENTRY':self.b.finish_native_move()
                self.aborted=True
                if not self.active:
                    self.state='DONE';self.event('DONE：等待阶段已中止，没有发送运动指令。');return
                if not np.all(s['enabled']):raise ValueError('Abort during incomplete enable: quick stop, no return')
                brake_q=s['q'].copy();brake_v=s['qd'].copy()
                if self.current_pose_only:
                    brake_q[:5]=self.home[:5];brake_v[:5]=0.
                p=brake(brake_q,brake_v,self.vmax,self.amax);self.check(p)
                self.set_path('BRAKING',p,now)
                self.event('LB+RB：立即取消激励，受限平滑制动后回到启动位置。')
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
                if np.all(s['enabled']):
                    self.armed=True;self.begin_ready_or_test(self.next_state,now)
                elif now-self.path_t>2.:raise ValueError('Enable timeout')
            elif self.state=='BOUNDARY_ENTRY':
                self.command(self.boundary_goal)
                at_goal=np.max(np.abs(s['q']-self.boundary_goal))<=np.deg2rad(.2) and np.max(np.abs(s['qd']))<=self.c['settle_velocity_rad_s']
                if at_goal:
                    if self.entry_stable_since is None:self.entry_stable_since=now
                    if now-self.entry_stable_since>=.5:
                        self.b.finish_native_move();self.set_path('PREPARING',self.outbound,now)
                else:self.entry_stable_since=None
                if self.state=='BOUNDARY_ENTRY' and now-self.path_t>8.:
                    raise ValueError('Boundary entry did not reach target in 8 s; position control not proven, no standing move or trial')
            elif self.state in ('PREPARING','TESTING','BRAKING','RETURNING'):
                if self.armed and not np.all(s['enabled']):raise ValueError('Joint disabled during motion')
                elapsed=now-self.path_t;q,_,_=self.path.at(elapsed);self.command(q)
                if elapsed>=self.path.duration:
                    if self.state=='PREPARING':self.state='SETTLING_READY';self.settle_started=now;self.stable_since=None
                    elif self.state in ('TESTING','BRAKING'):
                        if self.state=='TESTING':p=self.inbound
                        else:p=transit(q,self.home,self.vmax,self.amax);self.check(p)
                        self.set_path('RETURNING',p,now)
                    else:self.state='SETTLING_HOME';self.settle_started=now;self.stable_since=None
            elif self.state in ('SETTLING_READY','SETTLING_HOME'):self.finish_settle(now,s)
        except Exception as e:self.fault(e)
