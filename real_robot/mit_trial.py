"""Unified MIT PD / proportional-gravity trial. No MOVE J targets at any stage.

Protocol gains follow the official single-joint demo convention. Their joint-side
scale in simulation is an explicit b*c assumption, not a measured gain guarantee.
"""
import time
import numpy as np
from .controller import Controller
from .hardware import Hardware
from .simulation import Simulation
from .gravity import GravityModel,NM_PER_A,WIRE_TO_JOINT,encode_mit,uint
from .trajectory import transit,smooth
from .validation_trials import Waypoints


# Official reference PD gains, identical on all six joints and on every stage
# (prepare / test / return). The 15/1.2 step was rejected after hardware noise and
# joint kick events; at 10/0.8 the measured static error is about 1 deg on J3 and
# 4 deg on J5, so the arrival window is relaxed instead of raising the gains.
MIT_KP=[10.]*6
MIT_KD=[.8]*6


def configure(c,ff_scale=0.,gain_scale=1.,model='nominal',fit='sg21_LS'):
    if not np.isfinite(ff_scale) or not 0<=ff_scale<=1:raise ValueError('ff-scale must be in [0,1]')
    if not np.isfinite(gain_scale) or not .5<=gain_scale<=1:raise ValueError('mit-gain-scale must be in [0.5,1]')
    if not c.get('standing_start') or c.get('current_pose_only'):raise ValueError('MIT trial requires --standing')
    if model not in ('nominal','identified'):raise ValueError('Unknown gravity model')
    c.update(mit_ff_scale=float(ff_scale),mit_gain_scale=float(gain_scale),gravity_model=model,gravity_fit=fit,
        # Every PD-controlled stage uses the same vector; no stage may fall back to a lower gain.
        mit_kp=[float(x*gain_scale) for x in MIT_KP],mit_kd=[float(x*gain_scale) for x in MIT_KD],
        mit_prepare_kp=[float(x*gain_scale) for x in MIT_KP],mit_prepare_kd=[float(x*gain_scale) for x in MIT_KD],
        mit_return_kp=[float(x*gain_scale) for x in MIT_KP],mit_return_kd=[float(x*gain_scale) for x in MIT_KD],
        mit_ff_ramp_s=3.,mit_joint_torque_estimate_caps_Nm=[16.,16.,14.,3.,3.,2.],
        # A one-cycle velocity spike must not abort the run: the estimate only faults
        # after a sustained exceedance, and its D term uses a clamped velocity.
        mit_budget_exceed_duration_s=.10,mit_estimate_velocity_limit_rad_s=.6,
        # Prepare/return keep the slow 0.08 rad/s; only the TESTING path uses 0.15 so
        # the arm visibly swings while gravity still dominates the torque.
        max_velocity_rad_s=.08,max_acceleration_rad_s2=.08,
        test_max_velocity_rad_s=.15,test_max_acceleration_rad_s2=.15,
        # Test path: start -> +shoulder/elbow -> add wrist -> shoulder/elbow back ->
        # return. Four segments of roughly 15-30 deg keep every joint moving while the
        # whole TESTING phase stays near 19 s.
        mit_test_offsets_deg=[[10.,15.,15.,0.,0.,0.],
                              [10.,15.,15.,12.,15.,20.],
                              [-10.,-15.,-15.,12.,15.,20.]],
        mit_test_minimum_segment_s=3.5,
        settle_error_deg=2.8,mit_prepare_tolerance_deg=8.0,
        # Pure PD leaves ~4.2-4.3 deg static sag at the park pose, so the MOVE J 3 deg
        # release window would always time out. This MIT-only window is used instead.
        mit_park_tolerance_deg=8.,park_release_tolerance_deg=3.,settle_timeout_s=20.,
        feedback_stop_policy='stop',passive_wrist=False,hold_start_wrist=False,release_gripper=False,
        torque_candidate_coefficients_Nm_per_A=NM_PER_A.tolist(),
        torque_candidate_profile='official standard PiPER k*b*c; not individually calibrated')


def decode_command(q,v,kp,kd,ff):
    """Decoded protocol floats, including unavoidable old 8-bit zero-FF residue."""
    def quant(x,lo,hi,bits):return uint(float(x),lo,hi,bits)*(hi-lo)/((1<<bits)-1)+lo
    return tuple(np.array([quant(x,lo,hi,bits) for x in values]) for values,lo,hi,bits in
        [(q,-12.5,12.5,16),(v,-45.,45.,12),(kp,0.,500.,12),(kd,-5.,5.,12),(ff/WIRE_TO_JOINT,-8.,8.,8)])


def bumpless_reference(q,qd,old_q,old_v,old_kp,old_kd,old_ff_wire,new_kp,new_kd=None):
    # In protocol units, new v_ref=measured velocity removes the new D term.
    # New P term carries the prior total request while the FF field becomes 0.
    old_sum=old_kp*(old_q-q)+old_kd*(old_v-qd)+old_ff_wire
    velocity=qd.copy() if new_kd is None else np.zeros(6)
    correction=0. if new_kd is None else new_kd*qd
    return q+(old_sum+correction)/new_kp,velocity


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


# The stage machine lives in Controller; `mit` is a mode of it. The old name
# stays as an alias so imports and tests keep working.
MitController = Controller
