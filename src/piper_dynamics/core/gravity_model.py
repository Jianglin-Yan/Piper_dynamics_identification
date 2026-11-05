"""Linear gravity wrench for the current standard PiPER parameter revision."""
import hashlib,json
from pathlib import Path
import numpy as np
from piper_dynamics.core.model import inertial_model
from piper_dynamics.core import rigid_body_regressor
from piper_dynamics.core.torque import NM_PER_A
from piper_dynamics.paths import PROJECT_ROOT, STANDARD_MODELS, IDENTIFICATION_ARCHIVE
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

class GravityModel:
    def __init__(self,c):
        import mujoco
        self.mj=mujoco;self.model=inertial_model();self.data=mujoco.MjData(self.model)
        rg=rigid_body_regressor;self.theta=rg.ground_truth_theta(self.model).reshape(-1)
        assets={'real_robot/assets/piper.urdf':STANDARD_MODELS/'piper.urdf','real_robot/assets/nominal_piper.json':STANDARD_MODELS/'nominal_piper.json'}
        self.provenance={'model':'piper','gravity_source':c['gravity_model'],'assets':{str(PROJECT_ROOT/label):sha(p) for label,p in assets.items()},'friction_included':False,'fit_offset_included':False,'torque_reference':'official k*b*c; no extra ratio','torque_calibrated':False}
        if c['gravity_model']=='identified':
            root=Path(c.get('identification_archive',IDENTIFICATION_ARCHIVE));active=json.loads((root/'active_revision.json').read_text())
            if active.get('model')!='piper':raise ValueError('Current identified model is not ordinary PiPER')
            folder=root/active['revision_directory'];summary=json.loads((folder/'summary.json').read_text())
            for label,p in assets.items():
                if summary['model_fingerprint'][label]!=sha(p):raise ValueError('Identified/model asset hash mismatch')
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
