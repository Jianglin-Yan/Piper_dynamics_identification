"""Nominal PiPER X MuJoCo preview. Simplified visual geometry, not a collision certificate."""
import json, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ASSETS=Path(__file__).parent/'assets'

def numbers(v):return ' '.join(str(float(x)) for x in v)
def model_xml():
    nominal=json.loads((ASSETS/'nominal_x.json').read_text());urdf=ET.parse(ASSETS/'piper_x.urdf').getroot()
    joints=[urdf.find("joint[@name='joint%d']"%i) for i in range(1,7)]
    root=ET.Element('mujoco',model='PiPER X nominal simplified preview')
    ET.SubElement(root,'compiler',angle='radian',inertiafromgeom='false')
    ET.SubElement(root,'option',timestep='0.005',gravity='0 0 -9.81',integrator='implicitfast')
    visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='960',offheight='720')
    world=ET.SubElement(root,'worldbody');ET.SubElement(world,'light',pos='0 -1 2',dir='0 0 -1')
    ET.SubElement(world,'geom',type='plane',size='1 1 .01',rgba='.2 .23 .28 1',contype='0',conaffinity='0')
    parent=ET.SubElement(world,'body',name='base_link')
    ET.SubElement(parent,'geom',type='cylinder',size='.07 .05',pos='0 0 .05',rgba='.3 .32 .36 1',contype='0',conaffinity='0')
    for i,j in enumerate(joints,1):
        o=j.find('origin');xyz=np.fromstring(o.get('xyz'),sep=' ');rpy=np.fromstring(o.get('rpy'),sep=' ')
        xyzw=Rotation.from_euler('xyz',rpy).as_quat();quat=np.r_[xyzw[3],xyzw[:3]]
        body=ET.SubElement(parent,'body',name='link'+str(i),pos=numbers(xyz),quat=numbers(quat),gravcomp='1')
        lim=j.find('limit');ET.SubElement(body,'joint',name='joint'+str(i),type='hinge',axis=j.find('axis').get('xyz'),range=lim.get('lower')+' '+lim.get('upper'),damping='.1',armature='.005')
        if i==6:
            link=nominal['link6_plus_fixed_gripper'];m=link['mass_kg'];com=link['com_m_in_link6_frame'];I=np.array(link['inertia_com_kg_m2_in_link6_axes'])
        else:
            link=nominal['links']['link'+str(i)];m=link['mass_kg'];com=link['com_m_in_link_frame'];I=np.array(link['inertia_com_kg_m2_in_link_axes'])
        ET.SubElement(body,'inertial',mass=str(m),pos=numbers(com),fullinertia=numbers([I[0,0],I[1,1],I[2,2],I[0,1],I[0,2],I[1,2]]))
        ET.SubElement(body,'geom',type='sphere',size='.026',rgba='.15 .45 .75 1',contype='0',conaffinity='0')
        if i<6:
            endpoint=np.fromstring(joints[i].find('origin').get('xyz'),sep=' ')
            if np.linalg.norm(endpoint)>.01:ET.SubElement(body,'geom',type='capsule',fromto=numbers(np.r_[np.zeros(3),endpoint]),size='.018',rgba='.72 .76 .8 1',contype='0',conaffinity='0')
        else:
            ET.SubElement(body,'geom',type='box',pos='0 0 .055',size='.035 .025 .04',rgba='.25 .3 .36 1',contype='0',conaffinity='0')
            for sign in (-1,1):ET.SubElement(body,'geom',type='box',pos=f'{sign*.02} 0 .12',size='.005 .015 .03',rgba='.65 .7 .74 1',contype='0',conaffinity='0')
            ET.SubElement(body,'site',name='tip',pos='0 0 .15',size='.008',rgba='1 .45 .1 1')
        parent=body
    actuators=ET.SubElement(root,'actuator')
    for i,(kp,kd) in enumerate(zip([180,180,150,70,50,30],[12,12,10,4,3,2]),1):
        ET.SubElement(actuators,'position',joint='joint'+str(i),kp=str(kp),kv=str(kd))
    return ET.tostring(root,encoding='unicode')

class Simulation:
    def __init__(self,cfg):
        import mujoco
        self.mj=mujoco;self.c=cfg;self.model=mujoco.MjModel.from_xml_string(model_xml());self.data=mujoco.MjData(self.model)
        self.data.qpos[:]=np.deg2rad(cfg['simulation_initial_deg']);self.data.ctrl[:]=self.data.qpos
        if cfg.get('standing_start',False):
            from .standing import standing_plan
            _,_,lower,upper=standing_plan(cfg,self.data.qpos.copy())
            self.model.jnt_range[:,0]=lower;self.model.jnt_range[:,1]=upper
        if cfg.get('current_pose_only',False):
            from .current_pose import current_pose_bounds
            lower,upper=current_pose_bounds(cfg,self.data.qpos.copy())
            # Preview the same anchored parked coordinates. This changes only this simulated model.
            self.model.jnt_range[:5,0]=lower[:5];self.model.jnt_range[:5,1]=upper[:5]
        mujoco.mj_forward(self.model,self.data);self.enabled=False;self.stopped=False;self.active=False
        self.target=self.data.qpos.copy();self.time=0.;self.native_path=None;self.native_time=0.
    def advance(self,dt):
        for _ in range(round(dt/self.model.opt.timestep)):
            if self.enabled:
                control=self.native_path.at(self.time-self.native_time)[0] if self.native_path else self.target
                self.data.ctrl[:]=control;self.mj.mj_step(self.model,self.data)
            self.time+=self.model.opt.timestep
    def snapshot(self):
        return {'time':self.time,'q':self.data.qpos.copy(),'qd':self.data.qvel.copy(),'current_A':np.zeros(6),'tau_candidate_Nm':self.data.qfrc_actuator.copy(),'torque_source':'simulation actuator with gravity compensation; not identification ground truth',
                'q_times':np.full(6,self.time),'drive_times':np.full(6,self.time),'low_times':np.full(6,self.time),
                'enabled':np.full(6,self.enabled),'faults':[0]*6,'temp_C':[25]*6,'voltage_V':[24]*6,'status':{'arm_status':0,'error_code':0},'gripper':{}}
    def check(self,s,active):
        if self.stopped:raise RuntimeError('simulation stop latched')
    def arm(self,q):self.target=q.copy();self.enabled=True;self.active=True
    def begin_native_move(self,q):
        from .trajectory import transit
        self.native_path=transit(self.data.qpos.copy(),q,min(.06,self.c['max_velocity_rad_s']),self.c['max_acceleration_rad_s2'])
        self.native_time=self.time;self.target=q.copy()
    def finish_native_move(self):self.native_path=None
    def command(self,q):
        if self.stopped:raise RuntimeError('command after stop')
        self.target=q.copy()
    def quick_stop(self,reason):self.stopped=True;self.active=False;self.target=self.data.qpos.copy();self.data.qvel[:]=0
    def finish(self):self.active=False
    def close(self):pass
