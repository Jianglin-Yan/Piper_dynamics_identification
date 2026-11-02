"""Official-reference MIT gravity test; CAN is opened only by the hardware CLI.

Torque is computed in joint-output Nm; firmware DEFAULT wire torque is Nm/(b*c).
No extra mechanical gear ratio is applied. Preparations retain the proven MOVE J
path; normal recovery seeds MOVE J with the measured pose, then returns/disables.
"""
import hashlib,json,time
from pathlib import Path
import numpy as np
from .controller import Controller
from .hardware import Hardware
from .simulation import Simulation
from .trajectory import Segment,transit

BASE=Path(__file__).parent
K=np.array([1.18125]*3+[.9576]*3)
B=np.array([4.]*3+[1.]*3)
C=np.array([1.]*3+[.813252]*3)
NM_PER_A=K*B*C
WIRE_TO_JOINT=B*C

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

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

def uint(x,lo,hi,bits):
    if not np.isfinite(x) or not lo<=x<=hi:raise ValueError('MIT value outside reviewed encoding range')
    return int((x-lo)*((1<<bits)-1)/(hi-lo))

def encode_mit(joint,q,torque,vel_ref=0.,kp=0.,kd=0.):
    """Match pinned official DEFAULT driver/parser; gains are protocol values."""
    if type(joint) is not int or not 1<=joint<=6:raise ValueError('Invalid MIT joint')
    raw=float(torque)/WIRE_TO_JOINT[joint-1]
    p=uint(float(q),-12.5,12.5,16);v=uint(float(vel_ref),-45.,45.,12)
    kp=uint(float(kp),0.,500.,12);kd=uint(float(kd),-5.,5.,12);t=uint(raw,-8.,8.,8)
    frame=bytearray([p>>8,p&255,v>>4,((v&15)<<4)|(kp>>8),kp&255,kd>>4,((kd&15)<<4)|(t>>4)])
    frame.append(((t&15)<<4)|(int(np.bitwise_xor.reduce(frame))&15))
    actual=(t*16./255.-8.)*WIRE_TO_JOINT[joint-1]
    return bytes(frame),raw,float(actual)

class GravityModel:
    def __init__(self,c):
        import mujoco
        from .identify_offline import inertial_model,original_modules
        self.mj=mujoco;self.model=inertial_model();self.data=mujoco.MjData(self.model)
        rg,_,_=original_modules();self.theta=rg.ground_truth_theta(self.model).reshape(-1)
        assets=[BASE/'assets/piper.urdf',BASE/'assets/nominal_piper.json']
        self.provenance={'model':'piper','gravity_source':c['gravity_model'],'assets':{str(p):sha(p) for p in assets},'friction_included':False,'fit_offset_included':False,'torque_reference':'official k*b*c; no extra ratio','torque_calibrated':False}
        if c['gravity_model']=='identified':
            root=BASE.parent.parent/'identification';active=json.loads((root/'active_revision.json').read_text())
            if active.get('model')!='piper':raise ValueError('Current identified model is not ordinary PiPER')
            folder=root/active['revision_directory'];summary=json.loads((folder/'summary.json').read_text())
            for p in assets:
                if summary['model_fingerprint'][str(p.relative_to(BASE.parent))]!=sha(p):raise ValueError('Identified/model asset hash mismatch')
            if not np.allclose(summary['candidate_torque_coefficients'],NM_PER_A,rtol=0,atol=1e-12):raise ValueError('Identified torque profile differs from official MIT test profile')
            source=folder/'training/main_balanced_v2.npz'
            with np.load(source) as z:self.theta=z[c['gravity_fit']][:60].copy()
            self.provenance.update(revision=active['revision_id'],training='balanced_v2',method=c['gravity_fit'],parameter_file=str(source),sha256=sha(source))
        if self.theta.shape!=(60,) or not np.isfinite(self.theta).all():raise ValueError('Invalid gravity parameters')
        self.bodies=[mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_BODY,'link'+str(i)) for i in range(1,7)]
        self.jp=np.zeros((3,6));self.jr=np.zeros((3,6))
    def __call__(self,q):
        # Linear gravity wrench permits non-unique base-parameter representatives;
        # it does not create an invalid negative-mass simulation model.
        if np.shape(q)!=(6,) or not np.isfinite(q).all():raise ValueError('Invalid gravity state')
        d=self.data;m=self.model;d.qpos[:]=q;d.qvel[:]=0.;self.mj.mj_kinematics(m,d);self.mj.mj_comPos(m,d)
        g=m.opt.gravity;result=np.zeros(6)
        for i,bid in enumerate(self.bodies):
            self.mj.mj_jacBody(m,d,self.jp,self.jr,bid);th=self.theta[10*i:10*i+10]
            h_world=d.xmat[bid].reshape(3,3)@th[1:4]
            result-=self.jp.T@(th[0]*g)+self.jr.T@np.cross(h_world,g)
        return result

class GravityHardware(Hardware):
    def command(self,q):
        if getattr(self,'mit_active',False):self.position_started=time.monotonic()
        self.mit_active=False
        super().command(q)
    def exit_mit(self,q):self.command(q)
    def command_mit(self,q,torque):
        frames=[encode_mit(j+1,float(q[j]),float(torque[j]))[0] for j in range(6)]
        with self.tx_lock:
            if self.stopped or self.motion_closed:raise RuntimeError('MIT command after stop/disable')
            if not getattr(self,'mit_active',False):self.mit_started=time.monotonic()
            self.mit_active=True
            self._send_locked(0x151,[1,4,0,0xAD,0,0,0,0])
            for j,frame in enumerate(frames):self._send_locked(0x15a+j,frame)
    def check(self,s,active):
        super().check(s,active)
        if active and getattr(self,'mit_active',False) and time.monotonic()-self.mit_started>1.:
            if s['status'].get('motion_mode')!=4:raise RuntimeError('MIT mode not confirmed by firmware feedback')
        if active and not getattr(self,'mit_active',False) and hasattr(self,'position_started') and time.monotonic()-self.position_started>1.:
            if s['status'].get('motion_mode')!=1:raise RuntimeError('MOVE J return mode not confirmed by firmware feedback')

class GravitySimulation(Simulation):
    def __init__(self,c):
        super().__init__(c);self.mit_active=False;self.mit_output=np.zeros(6)
        self.position_gain=self.model.actuator_gainprm.copy();self.position_bias=self.model.actuator_biasprm.copy();self.position_gravcomp=self.model.body_gravcomp.copy()
    def exit_mit(self,q):
        self.data.qfrc_applied[:]=0.;self.mit_output[:]=0.;self.mit_active=False
        self.model.actuator_gainprm[:]=self.position_gain;self.model.actuator_biasprm[:]=self.position_bias;self.model.body_gravcomp[:]=self.position_gravcomp
        super().command(q)
    def command_mit(self,q,torque):
        if self.stopped or self.motion_closed:raise RuntimeError('MIT command after stop/disable')
        self.mit_active=True;self.model.body_gravcomp[:]=0.;self.model.actuator_gainprm[:]=0.;self.model.actuator_biasprm[:]=0.
        self.mit_output=np.array([encode_mit(j+1,float(q[j]),float(torque[j]))[2] for j in range(6)])
        self.data.qfrc_applied[:]=self.mit_output
    def snapshot(self):
        s=super().snapshot();s['status'].update(control_mode=1,motion_mode=4 if self.mit_active else 1)
        if self.mit_active:
            s.update(tau_candidate_Nm=self.mit_output.copy(),current_A=self.mit_output/NM_PER_A,torque_source='simulation, explicit gravity and 8-bit MIT torque quantization; not hardware evidence')
        return s
    def disable(self):self.data.qfrc_applied[:]=0.;self.mit_output[:]=0.;super().disable()
    def quick_stop(self,reason):self.data.qfrc_applied[:]=0.;self.mit_output[:]=0.;super().quick_stop(reason)

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
