"""PiPER model construction shared by analysis and the preview backend."""
import json
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from piper_dynamics.paths import STANDARD_MODELS
ASSETS=STANDARD_MODELS
def numbers(v):return ' '.join(str(float(x)) for x in v)

def model_xml(model_name="piper"):
    
    if model_name != 'piper':raise ValueError('Only standard PiPER is supported')
    meta,description='nominal_piper.json','piper.urdf'
    nominal=json.loads((ASSETS/meta).read_text());urdf=ET.parse(ASSETS/description).getroot()
    joints=[urdf.find("joint[@name='joint%d']"%i) for i in range(1,7)]
    root=ET.Element('mujoco',model=model_name+' nominal simplified preview')
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

def inertial_model(model_name="piper"):
    """Use selected URDF kinematics, with preview-only compensation/passive terms off."""
    import mujoco
    model = mujoco.MjModel.from_xml_string(model_xml(model_name))
    model.body_gravcomp[:] = 0
    model.dof_damping[:] = 0
    model.dof_armature[:] = 0
    model.dof_frictionloss[:] = 0
    model.jnt_limited[:] = 0
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    return model
