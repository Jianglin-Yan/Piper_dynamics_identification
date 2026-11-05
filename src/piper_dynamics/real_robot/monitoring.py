"""Feedback speed monitoring; reference limits are not instantaneous motor limits."""
import numpy as np

SPEED_DEFAULTS={
    'feedback_speed_warning_rad_s': .30,
    'feedback_speed_stop_rad_s': .60,
    'feedback_speed_stop_duration_s': .15,
    'feedback_speed_emergency_rad_s': 1.20,
    'feedback_speed_emergency_duration_s': .10,
}
CURRENT_DEFAULTS={'feedback_current_stop_duration_s':.20,'feedback_current_emergency_multiplier':2.0,'feedback_current_emergency_duration_s':.05}


def validate_speed_limits(cfg):
    for name,value in SPEED_DEFAULTS.items():
        cfg.setdefault(name,value)
        if not np.isfinite(cfg[name]) or cfg[name]<=0:
            raise ValueError('Invalid '+name)
    if not cfg['feedback_speed_warning_rad_s']<cfg['feedback_speed_stop_rad_s']<cfg['feedback_speed_emergency_rad_s']:
        raise ValueError('Feedback speed thresholds must be warning < stop < emergency')
    if cfg['feedback_speed_stop_rad_s']<=cfg['max_velocity_rad_s']:
        raise ValueError('Feedback stop threshold must exceed planned velocity')
    for name,value in CURRENT_DEFAULTS.items():
        cfg.setdefault(name,value)
        if not np.isfinite(cfg[name]) or cfg[name]<=0:raise ValueError('Invalid '+name)
    if cfg['feedback_current_emergency_multiplier']<=1:raise ValueError('Current emergency multiplier must exceed 1')


class SpeedMonitor:
    def __init__(self,cfg):
        cfg=dict(cfg);validate_speed_limits(cfg)
        self.warning=cfg['feedback_speed_warning_rad_s']
        self.stop=cfg['feedback_speed_stop_rad_s']
        self.duration=cfg['feedback_speed_stop_duration_s']
        self.emergency=cfg['feedback_speed_emergency_rad_s']
        self.emergency_duration=cfg['feedback_speed_emergency_duration_s']
        self.since=[None]*6;self.since_emergency=[None]*6;self.last_warning=-np.inf

    def update(self,now,qd):
        qd=np.asarray(qd,float)
        if qd.shape!=(6,) or not np.all(np.isfinite(qd)):
            raise ValueError('Missing/nonfinite speed feedback')
        j=int(np.argmax(np.abs(qd)));peak=abs(qd[j]);level=None
        for axis,value in enumerate(np.abs(qd)):
            if value>self.stop:
                if self.since[axis] is None:self.since[axis]=now
            else:self.since[axis]=None
            # A single-sample spike is not a runaway; require the extreme level to persist.
            if value>self.emergency:
                if self.since_emergency[axis] is None:self.since_emergency[axis]=now
            else:self.since_emergency[axis]=None
        sustained=[axis for axis,since in enumerate(self.since)
                   if since is not None and now-since>=self.duration-1e-9]
        emergency=[axis for axis,since in enumerate(self.since_emergency)
                   if since is not None and now-since>=self.emergency_duration-1e-9]
        if emergency:
            j=max(emergency,key=lambda axis:abs(qd[axis]));level='emergency'
        elif sustained:
            j=max(sustained,key=lambda axis:abs(qd[axis]));level='stop'
        elif peak>self.warning and now-self.last_warning>=1.:
            self.last_warning=now;level='warning'
        return {'level':level,'joint':j+1,'speed':float(qd[j])}


class CurrentMonitor:
    def __init__(self,cfg):
        self.limits=np.asarray(cfg['max_current_feedback_A'],float)
        self.duration=cfg.get('feedback_current_stop_duration_s',.20)
        self.multiplier=cfg.get('feedback_current_emergency_multiplier',2.)
        self.emergency_duration=cfg.get('feedback_current_emergency_duration_s',.05)
        self.warning_multiplier=cfg.get('feedback_current_warning_multiplier',.6)
        self.since=[None]*6;self.since_emergency=[None]*6;self.last_warning=-np.inf

    def update(self,now,current):
        current=np.asarray(current,float)
        if current.shape!=(6,) or not np.all(np.isfinite(current)):raise ValueError('Missing/nonfinite current feedback')
        ratio=np.abs(current)/self.limits;j=int(np.argmax(ratio));level=None
        for axis,value in enumerate(ratio):
            if value>1:
                if self.since[axis] is None:self.since[axis]=now
            else:self.since[axis]=None
            if value>self.multiplier:
                if self.since_emergency[axis] is None:self.since_emergency[axis]=now
            else:self.since_emergency[axis]=None
        sustained=[axis for axis,since in enumerate(self.since) if since is not None and now-since>=self.duration-1e-9]
        emergency=[axis for axis,since in enumerate(self.since_emergency) if since is not None and now-since>=self.emergency_duration-1e-9]
        if emergency:j=max(emergency,key=lambda axis:ratio[axis]);level='emergency'
        elif sustained:j=max(sustained,key=lambda axis:ratio[axis]);level='stop'
        elif np.max(ratio)>self.warning_multiplier and now-self.last_warning>=1:
            self.last_warning=now;level='warning'
        return {'level':level,'joint':j+1,'current':float(current[j])}
