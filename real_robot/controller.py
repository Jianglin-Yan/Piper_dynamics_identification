"""Backend-independent state machine. ABORT = controlled braking + return, not safety-rated E-stop."""
import numpy as np
from .trajectory import Excitation, transit, brake, check_path


class Controller:
    def __init__(self, backend, cfg, snapshot, now, emit=print):
        self.b=backend;self.c=cfg;self.emit=emit;self.home=snapshot['q'].copy();self.now=now
        self.start=self.home.copy() if cfg['start_deg'] is None else np.deg2rad(cfg['start_deg'])
        self.lower=np.deg2rad(cfg['lower_deg'])+np.deg2rad(cfg['margin_deg'])
        self.upper=np.deg2rad(cfg['upper_deg'])-np.deg2rad(cfg['margin_deg'])
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
        self.outbound=transit(self.home,self.start,self.vmax,self.amax)
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
        if np.max(np.abs(s['q']-self.home))>np.deg2rad(self.c['settle_error_deg']):
            raise ValueError('Robot moved after startup; restart to capture a new home pose')
        if np.max(np.abs(s['qd']))>self.c['settle_velocity_rad_s']:raise ValueError('Robot is moving before arming')
        self.active=True;self.b.arm(s['q']);self.state='ARMING';self.path_t=now;self.next_state=next_state
        self.event('ARMING：保持当前目标，等待六轴使能反馈。')
    def begin_ready_or_test(self,next_state,now):
        if next_state=='PREPARE':self.set_path('PREPARING',self.outbound,now)
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
        if np.any(q<self.lower-1e-9) or np.any(q>self.upper+1e-9):raise ValueError('command outside bounds')
        self.b.command(q);self.last_target=np.asarray(q).copy()
    def tick(self,now,s,action=None):
        if self.state in ('DONE','FAULT'):return
        try:
            self.b.check(s,self.active)
            if self.active:
                if self.state!='ARMING' and not np.all(s['enabled']):raise ValueError('Joint disabled during active session')
                if np.max(np.abs(s['q']-self.last_target))>np.deg2rad(self.c['tracking_error_deg']):raise ValueError('tracking error exceeded')
                if np.any(s['q']<self.lower) or np.any(s['q']>self.upper):raise ValueError('measured angle outside safety interior')
                if np.max(np.abs(s['qd']))>self.vmax*1.5:raise ValueError('measured overspeed')
            if action=='ABORT':
                self.aborted=True
                if not self.active:
                    self.state='DONE';self.event('DONE：等待阶段已中止，没有发送运动指令。');return
                if not np.all(s['enabled']):raise ValueError('Abort during incomplete enable: quick stop, no return')
                p=brake(s['q'],s['qd'],self.vmax,self.amax);self.check(p)
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
            elif self.state in ('PREPARING','TESTING','BRAKING','RETURNING'):
                if self.armed and not np.all(s['enabled']):raise ValueError('Joint disabled during motion')
                elapsed=now-self.path_t;q,_,_=self.path.at(elapsed);self.command(q)
                if elapsed>=self.path.duration:
                    if self.state=='PREPARING':self.state='SETTLING_READY';self.settle_started=now;self.stable_since=None
                    elif self.state in ('TESTING','BRAKING'):
                        p=transit(q,self.home,self.vmax,self.amax);self.check(p);self.set_path('RETURNING',p,now)
                    else:self.state='SETTLING_HOME';self.settle_started=now;self.stable_since=None
            elif self.state in ('SETTLING_READY','SETTLING_HOME'):self.finish_settle(now,s)
        except Exception as e:self.fault(e)
