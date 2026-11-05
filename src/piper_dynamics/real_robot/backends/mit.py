"""MIT-only hardware and simulation transports."""
import time
import numpy as np
from piper_dynamics.core.torque import NM_PER_A, WIRE_TO_JOINT
from piper_dynamics.real_robot.backends.hardware import Hardware
from piper_dynamics.real_robot.backends.simulation import Simulation
from piper_dynamics.real_robot.protocol.mit import encode_mit, decode_command
class MitHardware(Hardware):
    def preflight(self,info):
        super().preflight(info)
        if np.any(self.snapshot()['enabled']):
            raise ValueError('MIT-only startup requires six disabled axes. Park/disable the previous session first; no automatic disable or mode handoff.')
    def _pd_locked(self,q,v,kp,kd,ff):
        frames=[encode_mit(j+1,float(q[j]),float(ff[j]),float(v[j]),float(kp[j]),float(kd[j]))[0] for j in range(6)]
        self._send_locked(0x151,[1,4,0,0xAD,0,0,0,0])
        for j,frame in enumerate(frames):self._send_locked(0x15a+j,frame)
    def arm(self,q):
        with self.tx_lock:
            if self.stopped or self.motion_closed:raise RuntimeError('Stop/disable latched')
            self.heartbeat=time.monotonic();self.armed_at=self.heartbeat;self.active=True;self.motion_started=True
            self._pd_locked(q,np.zeros(6),np.array(self.c['mit_prepare_kp']),np.array(self.c['mit_prepare_kd']),np.zeros(6))
            for j in range(1,7):self._send_locked(0x471,[j,2,0,0,0,0,0,0])
    def command_pd(self,q,v,kp,kd,ff):
        values=[np.asarray(x,float) for x in (q,v,kp,kd,ff)]
        if any(x.shape!=(6,) or not np.isfinite(x).all() for x in values):raise ValueError('Invalid six-axis MIT command')
        # Validate every field before writing the mode frame or any partial target.
        for j in range(6):encode_mit(j+1,float(q[j]),float(ff[j]),float(v[j]),float(kp[j]),float(kd[j]))
        with self.tx_lock:
            if self.stopped or self.motion_closed:raise RuntimeError('MIT command after stop/disable')
            self._pd_locked(*values)
    def command(self,q):self.command_pd(q,np.zeros(6),np.array(self.c['mit_prepare_kp']),np.array(self.c['mit_prepare_kd']),np.zeros(6))
    def begin_native_move(self,q):raise RuntimeError('MOVE J entry forbidden in MIT-only trial')
    def check(self,s,active):
        super().check(s,active)
        if active and time.monotonic()-self.armed_at>1. and s['status'].get('motion_mode')!=4:
            raise RuntimeError('MIT mode not confirmed; no fallback to MOVE J')

class MitSimulation(Simulation):
    supports_stiffness_release=False
    def __init__(self,c):
        super().__init__(c);self.model.body_gravcomp[:]=0.;self.model.actuator_gainprm[:]=0.;self.model.actuator_biasprm[:]=0.
        self.ref=self.data.qpos.copy();self.vel=np.zeros(6);self.kp=np.zeros(6);self.kd=np.zeros(6);self.ff=np.zeros(6);self.output=np.zeros(6)
    def command_pd(self,q,v,kp,kd,ff):
        if self.stopped or self.motion_closed:raise RuntimeError('MIT command after stop/disable')
        self.ref,self.vel,self.kp,self.kd,self.ff=decode_command(np.asarray(q),np.asarray(v),np.asarray(kp),np.asarray(kd),np.asarray(ff))
    def command(self,q):self.command_pd(q,np.zeros(6),np.array(self.c['mit_prepare_kp']),np.array(self.c['mit_prepare_kd']),np.zeros(6))
    def arm(self,q):
        if self.motion_closed:raise RuntimeError('Arm after disable')
        self.enabled=True;self.active=True;self.command(q)
    def advance(self,dt):
        for _ in range(round(dt/self.model.opt.timestep)):
            if self.enabled or self.released:
                self.output=((self.kp*(self.ref-self.data.qpos)+self.kd*(self.vel-self.data.qvel)+self.ff)*WIRE_TO_JOINT if self.enabled else np.zeros(6))
                self.data.qfrc_applied[:]=self.output;self.mj.mj_step(self.model,self.data)
            self.time+=self.model.opt.timestep
    def snapshot(self):
        s=super().snapshot();s.update(tau_candidate_Nm=self.output.copy(),current_A=self.output/NM_PER_A,torque_source='simulation PD using protocol gains and assumed b*c output scaling; no hidden gravity compensation')
        s['status'].update(control_mode=1,motion_mode=4);return s
    def begin_native_move(self,q):raise RuntimeError('MOVE J forbidden in MIT simulation')
    def disable(self):self.data.qfrc_applied[:]=0.;self.output[:]=0.;super().disable()
    def quick_stop(self,reason):self.data.qfrc_applied[:]=0.;self.output[:]=0.;super().quick_stop(reason)
