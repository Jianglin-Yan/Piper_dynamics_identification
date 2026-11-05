"""Nominal PiPER MuJoCo preview (legacy X available explicitly). Simplified visual geometry, not a collision certificate."""
from piper_dynamics.paths import STANDARD_MODELS
import json, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

from piper_dynamics.core.model import model_xml, numbers
ASSETS=STANDARD_MODELS


class Simulation:
    def __init__(self,cfg):
        import mujoco
        self.mj=mujoco;self.c=cfg;self.model=mujoco.MjModel.from_xml_string(model_xml(cfg.get("model","piper")));self.data=mujoco.MjData(self.model)
        self.data.qpos[:]=np.deg2rad(cfg['simulation_initial_deg']);self.data.ctrl[:]=self.data.qpos
        if cfg.get('standing_start',False):
            from piper_dynamics.real_robot.standing import standing_plan
            _,_,lower,upper=standing_plan(cfg,self.data.qpos.copy())
            self.model.jnt_range[:,0]=lower;self.model.jnt_range[:,1]=upper
        if cfg.get('current_pose_only',False):
            from piper_dynamics.real_robot.current_pose import current_pose_bounds
            lower,upper=current_pose_bounds(cfg,self.data.qpos.copy())
            # Preview the same anchored parked coordinates. This changes only this simulated model.
            self.model.jnt_range[:5,0]=lower[:5];self.model.jnt_range[:5,1]=upper[:5]
        mujoco.mj_forward(self.model,self.data);self.enabled=False;self.stopped=False;self.active=False;self.released=False;self.motion_closed=False
        self.target=self.data.qpos.copy();self.time=0.;self.native_path=None;self.native_time=0.
    def advance(self,dt):
        for _ in range(round(dt/self.model.opt.timestep)):
            if self.enabled or self.released:
                control=self.native_path.at(self.time-self.native_time)[0] if self.native_path else self.target
                self.data.ctrl[:]=control;self.mj.mj_step(self.model,self.data)
            self.time+=self.model.opt.timestep
    def snapshot(self):
        return {'time':self.time,'q':self.data.qpos.copy(),'qd':self.data.qvel.copy(),'current_A':np.zeros(6),'tau_candidate_Nm':self.data.qfrc_actuator.copy(),'torque_source':'simulation actuator with gravity compensation; not identification ground truth',
                'q_times':np.full(6,self.time),'drive_times':np.full(6,self.time),'low_times':np.full(6,self.time),
                'enabled':np.full(6,self.enabled),'faults':[0]*6,'temp_C':[25]*6,'voltage_V':[24]*6,'status':{'arm_status':0,'error_code':0},'gripper':{}}
    def check(self,s,active):
        if self.stopped:raise RuntimeError('simulation stop latched')
    def arm(self,q):
        if self.motion_closed:raise RuntimeError('arm after disable')
        self.target=q.copy();self.enabled=True;self.active=True
    def begin_native_move(self,q):
        from piper_dynamics.real_robot.trials.trajectory import transit
        self.native_path=transit(self.data.qpos.copy(),q,min(.06,self.c['max_velocity_rad_s']),self.c['max_acceleration_rad_s2'])
        self.native_time=self.time;self.target=q.copy()
    def finish_native_move(self):self.native_path=None
    def command(self,q):
        if self.stopped or self.motion_closed:raise RuntimeError('command after stop/disable')
        self.target=q.copy()
    def quick_stop(self,reason):
        self.stopped=True;self.active=False
        # Qualitative gravity/damping preview only: vendor damping gains are unknown.
        # The old frozen-pose preview incorrectly suggested a real electronic stop holds up the arm.
        self.model.body_gravcomp[:]=0
        self.model.actuator_gainprm[:]=0;self.model.actuator_biasprm[:]=0
        self.model.dof_damping[:]=np.maximum(self.model.dof_damping,[2.,2.,2.,.15,.15,.08])
    def disable(self):
        self.enabled=False;self.active=False;self.released=True;self.motion_closed=True;self.native_path=None
        self.model.body_gravcomp[:]=0
        self.model.actuator_gainprm[:]=0;self.model.actuator_biasprm[:]=0
    def confirm_disabled(self):self.active=False
    def finish(self):self.active=False
    def close(self):pass

