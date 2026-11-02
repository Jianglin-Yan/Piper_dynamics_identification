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


# Protocol PD gains per joint, shared by prepare/test/return. J2/J3/J5 carry half
# again over the 10/0.8 baseline: at baseline the folded-to-standing prepare sags
# 6.3-6.5 deg on those axes, 15/1.2 brings the simulated static error to about
# 4.2-4.3 deg. 20/1.6 and the rejected 40/60 pair both produced audible motor
# whine on hardware, so this step is deliberately half of the previous increase.
MIT_KP=[10.,15.,15.,10.,15.,10.]
MIT_KD=[.8,1.2,1.2,.8,1.2,.8]


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
        max_velocity_rad_s=.08,max_acceleration_rad_s2=.08,settle_error_deg=2.8,mit_prepare_tolerance_deg=5.0,
        # Pure PD leaves ~4.2-4.3 deg static sag at the park pose, so the MOVE J 3 deg
        # release window would always time out. This MIT-only window is used instead.
        mit_park_tolerance_deg=5.,park_release_tolerance_deg=3.,settle_timeout_s=20.,
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


class MitController(Controller):
    def __init__(self,b,c,s,now,emit=print):
        self.mit_sample=s;self.mit_time=now;self.mit_kp=np.array(c['mit_prepare_kp']);self.mit_kd=np.array(c['mit_prepare_kd']);self.mit_ff_ratio=0.;self.mit_telemetry=None;self.returning_mit=False;self.ff_print_t=-np.inf
        self.gravity=GravityModel(c) if c['mit_ff_scale']>0 else None
        super().__init__(b,c,s,now,emit)
        self.outbound=transit(self.home,self.start,self.vmax,self.amax);self.check(self.outbound)
        delta=np.deg2rad([6.,4.,-4.,4.,3.,6.]);poses=np.array([self.start,self.start+delta,self.start-delta,self.start])
        self.test=Waypoints(poses,self.vmax,self.amax);self.nominal_test=self.test;self.check(self.test)
        self.reference_plan={'trajectory_type':'mit','all_stages':'MIT only, including prepare/return','poses_deg':np.rad2deg(poses).tolist(),'duration_s':self.test.duration,'ff_scale':c['mit_ff_scale'],'ff_ramp_s':c['mit_ff_ramp_s'],'prepare_kp_protocol':c['mit_prepare_kp'],'prepare_kd_protocol':c['mit_prepare_kd'],'kp_protocol':c['mit_kp'],'kd_protocol':c['mit_kd'],'return_kp_protocol':c['mit_return_kp'],'return_kd_protocol':c['mit_return_kd'],'gravity_model':None if self.gravity is None else self.gravity.provenance,'simulation_gain_assumption':'PD protocol torque scaled to joint by b*c; not hardware gain calibration','zero_ff_note':'requested FF exactly 0; old 8-bit encoding has finite zero residue'}
        self.event('MIT_ONLY：准备／固定轨迹／回收全程MIT；前馈比例 '+str(c['mit_ff_scale'])+'；无MOVE J回退。协议Kp='+str(c['mit_kp'])+'，Kd='+str(c['mit_kd'])+'。')
        self.event('PD_SETTLE：纯PD允许重力静差，到位容差5°，实际误差持续记录；不把静差判成编码失败。')
    def motors_ready(self,s):return super().motors_ready(s) and s.get('status',{}).get('motion_mode',4)==4
    def activate(self,s,now,next_state):
        super().activate(s,now,next_state)
        if self.active and self.state=='ARMING':self.command(self.home)
    def begin_ready_or_test(self,next_state,now):
        if next_state=='PREPARE':self.set_path('PREPARING',self.outbound,now)
        else:self.begin_testing(now)
    def reference_velocity(self,now):
        if hasattr(self,'stop_velocity'):return self.stop_velocity.copy()
        return super().reference_velocity(now)
    def command(self,q):
        q=np.asarray(q,float);s=self.mit_sample;v=self.reference_velocity(self.mit_time)
        if q.shape!=(6,) or np.any(q<self.lower-1e-9) or np.any(q>self.upper+1e-9):raise ValueError('MIT reference outside command bounds')
        g=np.zeros(6) if self.gravity is None or self.mit_ff_ratio==0 else self.gravity(s['q'])
        ff=self.mit_ff_ratio*g
        pd=(self.mit_kp*(q-s['q'])+self.mit_kd*(v-s['qd']))*WIRE_TO_JOINT
        est=pd+ff
        if np.any(abs(est)>np.array(self.c['mit_joint_torque_estimate_caps_Nm'])):
            raise ValueError('MIT combined PD+FF estimate exceeds command budget')
        self.b.command_pd(q,v,self.mit_kp,self.mit_kd,ff);self.last_target=q.copy()
        self.mit_telemetry={'phase':self.state,'ff_ratio':self.mit_ff_ratio,'q_ref':q.copy(),'v_ref':v.copy(),'kp_protocol':self.mit_kp.copy(),'kd_protocol':self.mit_kd.copy(),'g_Nm':g,'ff_requested_Nm':ff,'wire_t_ref':ff/WIRE_TO_JOINT,'quantized_ff_Nm':np.array([encode_mit(j+1,float(q[j]),float(ff[j]),float(v[j]),float(self.mit_kp[j]),float(self.mit_kd[j]))[2] for j in range(6)]),'pd_estimate_Nm':pd,'total_estimate_Nm':est}
    def ff_progress(self,now,tag,target_pct):
        # The ramp stages hold the arm still, so the operator needs a running
        # explanation of why nothing is moving and when the next step starts.
        if now-self.ff_print_t<1.-1e-9:return
        self.ff_print_t=now
        if float(self.c['mit_ff_scale'])<=0:
            self.event(tag+'：前馈比例为0（纯PD），无重力前馈；机械臂保持静止。');return
        ff=np.zeros(6) if self.mit_telemetry is None else np.asarray(self.mit_telemetry['ff_requested_Nm'])
        self.event(f'{tag}：前馈比例 {100*self.mit_ff_ratio:.0f}%（目标 {target_pct:.0f}%），请求 '+str(np.round(ff,2).tolist())+' Nm；机械臂保持静止，PD继续维持姿态。')
    def finish_settle(self,now,s):
        if self.state=='SETTLING_HOME':
            # MIT return holds the park pose with pure PD, which has a repeatable static
            # sag; accept the MIT-only window rather than the 3 deg MOVE J release value.
            if self.position_stable(s,now,self.return_home,self.c['mit_park_tolerance_deg']):self.begin_disarm(s,now)
            elif now-self.settle_started>self.c['settle_timeout_s']:raise ValueError('Timed out settling at target')
            return
        if self.state!='SETTLING_READY':return super().finish_settle(now,s)
        self.command(self.start)
        if self.position_stable(s,now,self.start,self.c['mit_prepare_tolerance_deg']):
            self.state='FF_RAMP_IN';self.ff_phase_t=now;self.ff_print_t=now
            scale=float(self.c['mit_ff_scale'])
            if scale<=0:self.event('FF_RAMP_IN_START：前馈比例为0（纯PD），无重力前馈；机械臂保持静止3秒后进入READY。')
            else:self.event(f'FF_RAMP_IN_START：已到标准姿态；机械臂保持静止，重力前馈将在{self.c["mit_ff_ramp_s"]:g}秒内由0%升到{100*scale:.0f}%，按实测关节角计算。异常请按LB+RB。')
        elif now-self.settle_started>self.c['settle_timeout_s']:raise ValueError('MIT preparation failed to settle; no gain auto-escalation')
    def request_stop(self,s,now,reason):
        self.now=now
        if self.finished or self.shutdown_phase or self.state in ('BRAKING','RETURNING','SETTLING_HOME'):return
        if not self.active:return super().request_stop(s,now,reason)
        if reason=='MIT trial completed':
            self.stop_reason=reason;self.state='FF_RAMP_OUT';self.ff_phase_t=now;self.ff_exit_ratio=self.mit_ff_ratio;self.exit_kp=self.mit_kp.copy();self.exit_kd=self.mit_kd.copy();self.ff_print_t=now
            if float(self.c['mit_ff_scale'])<=0:self.event('FF_RAMP_OUT_START：前馈比例为0（纯PD），无前馈可撤；机械臂保持静止3秒后开始MIT回位。')
            else:self.event(f'FF_RAMP_OUT_START：轨迹结束，机械臂保持静止；重力前馈将在{self.c["mit_ff_ramp_s"]:g}秒内由{100*self.mit_ff_ratio:.0f}%降到0%，PD增益不变。')
            return
        if self.returning_mit:return
        if self.state=='ARMING' and not self.motors_ready(s):
            self.mit_ff_ratio=0.;self.begin_disarm(s,now);return
        try:
            self.b.check(s,True)
            if not self.motors_ready(s):raise ValueError('MIT not active; no forced return')
            old=self.mit_telemetry
            kp=np.array(self.c['mit_return_kp']);kd=np.array(self.c['mit_return_kd'])
            if old is None:q=s['q'].copy();v=np.zeros(6)
            else:q,v=bumpless_reference(s['q'],s['qd'],old['q_ref'],old['v_ref'],old['kp_protocol'],old['kd_protocol'],old['wire_t_ref'],kp,kd)
            bounded=np.clip(q,self.lower,self.upper)
            if np.max(abs(q-bounded))>1e-8:self.event('RETURN_REFERENCE_WARNING：无扰参考触及规划边界，已限制；切换不保证完全连续。')
            self.mit_ff_ratio=0.;self.mit_kp=kp;self.mit_kd=kd;self.stop_velocity=v
            self.command(bounded);self.returning_mit=True
            self.event('MIT_ABORT_RETURN：前馈请求立即清零；统一PD增益接管，以无扰参考制动，MIT回位失能。')
            super().request_stop(s,now,reason)
        except Exception as e:self.fault('MIT controlled return unavailable: '+str(e))
        finally:
            if hasattr(self,'stop_velocity'):del self.stop_velocity
    def tick(self,now,s,action=None):
        self.mit_sample=s;self.mit_time=now
        if self.state=='FF_RAMP_IN':
            f=float(smooth((now-self.ff_phase_t)/self.c['mit_ff_ramp_s'])[0]);self.mit_ff_ratio=self.c['mit_ff_scale']*f
            self.mit_kp=(1-f)*np.array(self.c['mit_prepare_kp'])+f*np.array(self.c['mit_kp'])
            self.mit_kd=(1-f)*np.array(self.c['mit_prepare_kd'])+f*np.array(self.c['mit_kd'])
        if self.state=='TESTING' and now-self.path_t>=self.test.duration:self.request_stop(s,now,'MIT trial completed')
        if self.state=='FF_RAMP_OUT':
            f=float(smooth((now-self.ff_phase_t)/self.c['mit_ff_ramp_s'])[0]);self.mit_ff_ratio=(1-f)*self.ff_exit_ratio
            self.mit_kp=(1-f)*self.exit_kp+f*np.array(self.c['mit_return_kp']);self.mit_kd=(1-f)*self.exit_kd+f*np.array(self.c['mit_return_kd'])
        super().tick(now,s,action)
        if self.shutdown_phase:return
        try:
            if self.state=='FF_RAMP_IN':
                self.command(self.start)
                if now-self.ff_phase_t>=self.c['mit_ff_ramp_s']:
                    self.mit_ff_ratio=self.c['mit_ff_scale'];self.state='READY'
                    if float(self.c['mit_ff_scale'])>0:self.event(f'FF_RAMP_IN_DONE：前馈已达{100*self.mit_ff_ratio:.0f}%，机械臂仍静止；READY，LB+Y执行固定轨迹。')
                    else:self.event('READY：MIT准备和前馈渐入完成；LB+Y执行固定轨迹。')
                else:self.ff_progress(now,'FF_RAMP_IN',100*float(self.c['mit_ff_scale']))
            elif self.state=='FF_RAMP_OUT':
                self.command(self.last_target)
                if now-self.ff_phase_t>=self.c['mit_ff_ramp_s']:
                    if float(self.c['mit_ff_scale'])>0:self.event('FF_RAMP_OUT_DONE：前馈已归零，PD增益不变；开始MIT回位。')
                    self.mit_ff_ratio=0.;self.returning_mit=True;super().request_stop(s,now,self.stop_reason)
                else:self.ff_progress(now,'FF_RAMP_OUT',0.)
        except Exception as e:self.fault(e)
