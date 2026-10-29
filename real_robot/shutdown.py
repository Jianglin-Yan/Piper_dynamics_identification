"""Park, remove motor torque, and confirm fresh six-axis disable feedback before exit."""
import numpy as np


class Shutdown:
    def init_shutdown(self):
        self.shutdown_phase=None;self.shutdown_complete=False;self.disarm_confirmed=False
        self.disarm_started=None;self.disable_sent=None;self.disable_last_attempt=None
        self.disable_stable_since=None;self.shutdown_warning=None;self.release_q=None

    @property
    def finished(self):
        return self.shutdown_complete and self.state in ('DONE','FAULT')

    def fresh_disabled(self,s,now,after=-np.inf):
        stamps=np.asarray(s.get('low_times',[]),dtype=float)
        enabled=np.asarray(s.get('enabled',[]))
        return (stamps.shape==(6,) and enabled.shape==(6,) and np.all(np.isfinite(stamps)) and
                np.min(stamps)>=after and np.max(stamps)<=now+1e-6 and
                now-np.min(stamps)<=self.c['status_timeout_s'] and not np.any(enabled))

    def fresh_pose(self,s,now):
        q=np.asarray(s.get('q',[]));stamps=np.asarray(s.get('q_times',[]))
        return (q.shape==(6,) and np.all(np.isfinite(q)) and stamps.shape==(6,) and
                np.all(np.isfinite(stamps)) and np.max(stamps)<=now+1e-6 and
                now-np.min(stamps)<=self.c['feedback_timeout_s'])

    def park_for_shutdown(self,now,reason):
        # WAITING may inherit powered motors from a different program. Never assume off.
        self.stop_reason=str(reason);self.shutdown_phase='VERIFY_PARK'
        self.state='PARK_VERIFY';self.settle_started=now
        self.event('PARK_VERIFY：未启动运动；确认启动位置静止后失能，不带电退出。')

    def begin_disarm(self,s,now):
        self.release_q=np.asarray(s['q']).copy();self.shutdown_phase='DISABLING'
        if not self.fault_reason:self.state='DISABLING'
        self.disarm_started=now;self.disable_last_attempt=None;self.disable_sent=None
        self.disable_stable_since=None
        delta=np.rad2deg(self.release_q-self.home).round(3).tolist()
        self.event('DISABLING：回位结束，实际位置相对原始启动值差 '+str(delta)+
                   '°；失能即撤去驱动力矩，等待六轴新反馈确认。')
        self.poll_disarm(s,now)

    def poll_disarm(self,s,now):
        # Keep receiving while disabling. No position, re-enable, reset, or electronic-stop command here.
        acknowledged=(self.disable_sent is not None and self.fresh_disabled(s,now,self.disable_sent))
        if acknowledged:
            if self.disable_stable_since is None:self.disable_stable_since=now
            if now-self.disable_stable_since>=self.c.get('disable_confirm_s',.15)-1e-8:
                self.disarm_confirmed=True;self.shutdown_complete=True;self.shutdown_phase='DISABLED'
                self.b.confirm_disabled();self.active=False
                self.state='FAULT' if self.fault_reason else 'DONE'
                self.event(self.state+'：六轴已确认失能，驱动力矩已撤去，关闭采集并退出。')
                return
        else:self.disable_stable_since=None
        if self.disable_last_attempt is None or (not acknowledged and now-self.disable_last_attempt>=.1):
            self.disable_last_attempt=now
            try:
                self.b.disable()
                if self.disable_sent is None:self.disable_sent=now
                self.active=False
            except Exception as e:
                if self.shutdown_warning!=str(e):
                    self.shutdown_warning=str(e);self.event('DISABLE_SEND_FAILED：'+str(e)+'；保持监视，不退出。')
        if now-self.disarm_started>self.c.get('disable_timeout_s',2.) and self.shutdown_phase!='DISABLE_UNCONFIRMED':
            self.shutdown_phase='DISABLE_UNCONFIRMED'
            self.event('DISABLE_UNCONFIRMED：尚未收到六轴完整失能反馈；继续失能请求及记录，不退出。请现场确认通信/电源并支撑机械臂。')

    def shutdown_tick(self,now,s):
        if self.shutdown_complete:return
        if self.shutdown_phase in ('DISABLING','DISABLE_UNCONFIRMED'):
            self.poll_disarm(s,now);return
        if self.shutdown_phase not in ('VERIFY_PARK','FAULT_WAITING_PARK'):return
        # Already off: do not command a return or re-enable just to tidy the pose.
        if self.fresh_disabled(s,now):
            self.begin_disarm(s,now);return
        fresh=self.fresh_pose(s,now)
        tolerance=self.c.get('park_release_tolerance_deg',3.)
        # In unarmed WAITING the raw folded start can be just outside firmware limits.
        target=self.home if self.shutdown_phase=='VERIFY_PARK' else self.return_home
        if fresh and self.position_stable(s,now,target,tolerance):
            self.begin_disarm(s,now);return
        if now-self.settle_started>self.c['settle_timeout_s'] and self.shutdown_warning!='park':
            self.shutdown_warning='park'
            self.event('PARK_UNCONFIRMED：无法确认已回到启动位置附近；禁止自动卸力和退出。请现场支撑并移回启动位置附近，程序只监视，不恢复或使能。')
