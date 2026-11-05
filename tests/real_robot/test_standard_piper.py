from piper_dynamics.paths import LEGACY_ROBOT_ROOT
import json,tempfile,unittest
from pathlib import Path
import numpy as np
import mujoco
from piper_dynamics.pipeline.measured_run import inertial_model,original_modules
from piper_dynamics.real_robot.trials.designed_trajectory import load_candidate
from tests.real_robot.test_multisine import multisine_config
from piper_dynamics.real_robot.standing import STANDING_DEG
class StandardPiperContract(unittest.TestCase):
 def test_firmware_pose_and_j4_axis(self):
  m=inertial_model();d=mujoco.MjData(m);d.qpos[:]=np.deg2rad([0,44.995,-59.97,0,0,0]);mujoco.mj_forward(m,d);bid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'link6');self.assertLess(np.linalg.norm(d.xpos[bid]*1000-[141.253,0,487.365]),.11);self.assertLess(abs(np.dot(d.xaxis[2],d.xaxis[3])),1e-6)
 def test_regressor_matches_independent_inverse_dynamics(self):
  m=inertial_model();d=mujoco.MjData(m);rg,_,_=original_modules();b=rg.RegressorBuilder(m,link_bodies=tuple((f'link{i}',) for i in range(1,7)));rng=np.random.default_rng(923);q=rng.uniform([-.5,.2,-2,-.8,-.8,-1],[.5,2.5,-.2,.8,.8,1],(12,6));v=rng.uniform(-.5,.5,(12,6));a=rng.uniform(-2,2,(12,6));Y=b.build(q,v,a);theta=rg.ground_truth_theta(m).reshape(-1)
  for qi,vi,ai,yi in zip(q,v,a,Y):
   d.qpos[:]=qi;d.qvel[:]=vi;mujoco.mj_forward(m,d);d.qacc[:]=ai;mujoco.mj_inverse(m,d);np.testing.assert_allclose(yi@theta,d.qfrc_inverse,atol=1e-8)
 def test_profile_uses_confirmed_firmware_j6_limit(self):
  cfg=json.loads((LEGACY_ROBOT_ROOT/'config.json').read_text());self.assertEqual(cfg['model'],'piper');self.assertEqual(cfg['lower_deg'][5],-120);self.assertEqual(cfg['upper_deg'][5],120)
 def test_foreign_model_candidate_rejected(self):
  c=multisine_config();c.update(model='piper',trajectory_scale=.5,trajectory_speed_scale=.5);b=dict(schema_version=1,model='unsupported_model',execution=dict(scale=.5,speed_scale=.5,anchor_deg=STANDING_DEG,harmonics=5,source_duration_s=60.,source_ramp_s=5.,source_frequency_hz=.2),model_sha256={},candidates={'legacy':dict(coefficients_source_rad=np.zeros((2,6,5)).tolist(),status='legacy')})
  with tempfile.TemporaryDirectory() as td:
   p=Path(td)/'bank.json';p.write_text(json.dumps(b))
   with self.assertRaisesRegex(ValueError,'model differs'):load_candidate(c,'legacy',p)
