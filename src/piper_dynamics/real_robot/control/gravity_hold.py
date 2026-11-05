"""Existing gravity-hold configuration and control sequence."""
import time
import numpy as np
from piper_dynamics.core.torque import K,B,C,NM_PER_A,WIRE_TO_JOINT
from piper_dynamics.core.gravity_model import GravityModel,sha
from piper_dynamics.real_robot.protocol.mit import uint,encode_mit,decode_command
from piper_dynamics.real_robot.controller import Controller
from piper_dynamics.paths import LEGACY_ROBOT_ROOT as BASE
from piper_dynamics.real_robot.backends.gravity import GravityHardware,GravitySimulation
from piper_dynamics.real_robot.trials.trajectory import Segment,transit
def configure(c,model='identified',fit='sg21_LS',duration=30.,stiffness=1.):
    c.update(gravity_model=model,gravity_fit=fit,duration_s=float(duration),ramp_s=3.,
        gravity_stiffness=float(stiffness),gravity_entry_ramp_s=3.,gravity_gain_ramp_s=5.,
        gravity_hold_kp=[40.,50.,40.,5.,3.,1.],gravity_soft_kp=[3.,3.,2.,.3,.3,.2],
        gravity_kd=[3.,4.,3.,.2,.12,.08],gravity_torque_caps_Nm=[12.,12.,10.,2.,2.,1.],
        gravity_torque_slew_Nm_s=[15.,15.,15.,3.,3.,2.],gravity_excursion_deg=20.,
        gravity_speed_return_rad_s=.5,feedback_stop_policy='stop',
        torque_candidate_coefficients_Nm_per_A=NM_PER_A.tolist(),
        torque_candidate_profile='official standard PiPER k*b*c; not individually calibrated',
        passive_wrist=False,hold_start_wrist=False,release_gripper=False)
    if model not in ('nominal','identified') or not 10<=duration<=120 or not 0<=stiffness<=1:
        raise ValueError('gravity: model nominal/identified, duration 10..120 s, stiffness 0..1 required')
    if not c.get('standing_start') or c.get('current_pose_only'):raise ValueError('gravity requires --standing')

class GravityController(Controller):
    def __init__(self,b,c,s,now,emit=print):
        self.gravity=GravityModel(c);self.gravity_streaming=False;self.gravity_sample=s;self.gravity_time=now;self.gravity_telemetry=None
        super().__init__(b,c,s,now,emit)
        self.test=Segment(self.start,self.start,c['duration_s']);self.nominal_test=self.test
        self.reference_plan={'trajectory_type':'gravity','mode':'MIT after LB+Y; preparation MOVE J','duration_s':c['duration_s'],'source':self.gravity.provenance,'wire_coefficients':{'k':K.tolist(),'b':B.tolist(),'c':C.tolist()},'software_PD':True,'wire_kp':0,'wire_kd':0,'torque_caps_Nm':c['gravity_torque_caps_Nm'],'final_soft_kp':(np.array(c['gravity_soft_kp'])*c['gravity_stiffness']).tolist()}
        self.event('GRAVITY：LB+X站立准备；LB+Y进入MIT重力补偿，逐步降低刚度；LB+RB回位失能。官方系数，不做额外挂重标定。')
    def begin_testing(self,now):
        self.gravity_streaming=True;self.gravity_started=now;self.gravity_last_time=now
        current=np.asarray(self.gravity_sample.get('current_A',np.zeros(6)))
        self.gravity_initial_torque=np.clip(current*NM_PER_A,-np.array(self.c['gravity_torque_caps_Nm']),self.c['gravity_torque_caps_Nm'])
        if isinstance(self.b,GravitySimulation):self.gravity_initial_torque=self.gravity(self.gravity_sample['q'])
        self.gravity_last_torque=self.gravity_initial_torque.copy();self.gravity_recover_started=None;self.gravity_phase=None
        super().begin_testing(now)
        self.command(self.start)
    def command(self,q):
        if not self.gravity_streaming:return super().command(q)
        q=np.asarray(q);s=self.gravity_sample;now=self.gravity_time
        if np.any(q<self.lower-1e-9) or np.any(q>self.upper+1e-9):raise ValueError('MIT position reference outside command bounds')
        elapsed=max(0.,now-self.gravity_started);hold=np.array(self.c['gravity_hold_kp']);soft=np.array(self.c['gravity_soft_kp'])*self.c['gravity_stiffness'];kd=np.array(self.c['gravity_kd'])
        entry=self.c['gravity_entry_ramp_s'];gain=self.c['gravity_gain_ramp_s'];mix=float(np.clip(elapsed/entry,0,1));mix=mix*mix*(3-2*mix)
        if self.state=='TESTING':
            f=float(np.clip((elapsed-entry)/gain,0,1));f=f*f*(3-2*f);kp=(1-f)*hold+f*soft
            phase='takeover' if elapsed<entry else 'lower_stiffness' if f<1 else 'gravity_soft_hold'
        else:
            f=float(np.clip((now-self.gravity_recover_started)/2.,0,1));kp=(1-f)*self.gravity_recover_kp+f*hold;phase='return_MIT'
        g=self.gravity(s['q']);support=(1-mix)*self.gravity_initial_torque+mix*g
        pd=kp*(q-s['q'])-kd*s['qd'];requested=support+pd;caps=np.array(self.c['gravity_torque_caps_Nm']);capped=np.clip(requested,-caps,caps)
        dt=max(0.,min(now-self.gravity_last_time,.02));slew=np.array(self.c['gravity_torque_slew_Nm_s'])*dt
        sent=np.clip(capped,self.gravity_last_torque-slew,self.gravity_last_torque+slew)
        self.b.command_mit(s['q'],sent);self.last_target=q.copy();self.gravity_last_torque=sent.copy();self.gravity_last_time=now
        self.gravity_telemetry={'phase':phase,'g_Nm':g,'support_Nm':support,'pd_Nm':pd,'requested_Nm':requested,'sent_Nm':sent,'wire_t_ref':sent/WIRE_TO_JOINT,'quantized_Nm':np.array([encode_mit(j+1,float(s['q'][j]),float(sent[j]))[2] for j in range(6)]),'kp_Nm_rad':kp,'kd_Nm_s_rad':kd,'limited':bool(np.any(abs(requested-sent)>1e-8))}
        if phase!=self.gravity_phase:self.gravity_phase=phase;self.event('GRAVITY_PHASE：'+phase+'；Kp='+str(kp.round(3).tolist()))
    def reference_velocity(self,now):
        if hasattr(self,'gravity_stop_velocity'):return self.gravity_stop_velocity.copy()
        if self.gravity_streaming and self.state=='TESTING':return self.gravity_sample['qd'].copy()
        return super().reference_velocity(now)
    def request_stop(self,s,now,reason):
        if self.gravity_streaming and self.state=='TESTING' and not self.shutdown_phase:
            self.now=now
            try:
                self.b.check(s,True)
                if not self.motors_ready(s):raise ValueError('Cannot hand back a disabled joint')
                if np.any(s['q']<np.deg2rad(self.c['lower_deg'])) or np.any(s['q']>np.deg2rad(self.c['upper_deg'])):
                    raise ValueError('Measured pose outside firmware command bounds; cannot seed position return')
                self.last_target=s['q'].copy();self.gravity_stop_velocity=s['qd'].copy()
                self.b.exit_mit(self.last_target);self.gravity_streaming=False
                self.event('GRAVITY_RETURN：以当前实测角度接回MOVE J，再制动回位失能；回收不依赖待验证重力模型。')
            except Exception as e:
                self.fault('Gravity return handoff unavailable: '+str(e));return
        try:return super().request_stop(s,now,reason)
        finally:
            if hasattr(self,'gravity_stop_velocity'):del self.gravity_stop_velocity
    def tick(self,now,s,action=None):
        self.gravity_sample=s;self.gravity_time=now
        if self.gravity_streaming and self.state=='TESTING' and not self.shutdown_phase:
            excursion=np.max(abs(np.rad2deg(s['q']-self.start)))
            if now-self.gravity_started>=self.c['duration_s']:self.request_stop(s,now,'Gravity test completed')
            elif excursion>self.c['gravity_excursion_deg'] or np.max(abs(s['qd']))>self.c['gravity_speed_return_rad_s']:
                self.request_stop(s,now,'Gravity excursion/speed return threshold')
        super().tick(now,s,action)
