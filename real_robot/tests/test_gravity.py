"""Independent mathematical/protocol checks and physical simulation exits."""
import ast,json,tempfile,threading,time,types,unittest
from pathlib import Path
import mujoco,numpy as np
from real_robot.gravity import (BASE,K,B,C,NM_PER_A,WIRE_TO_JOINT,GravityModel,
    GravityHardware,GravitySimulation,GravityController,configure,encode_mit)
from real_robot.run import validate_config
from real_robot.identify_offline import original_modules


def profile(source='nominal',seconds=10.,stiffness=1.):
 c=json.loads((BASE/'config.json').read_text());c.update(trajectory='gravity',standing_start=True,start_deg=[0,45,-60,0,0,0]);configure(c,source,duration=seconds,stiffness=stiffness);validate_config(c);return c

class GravityMathAndProtocol(unittest.TestCase):
 def test_official_constants_and_scaling_are_not_reduction_applied_twice(self):
  ns={};exec(compile((BASE/'assets/mit_official/constants.py').read_text(),'<official constants>','exec'),ns)
  for ours,key in [(K,'K'),(B,'B'),(C,'C')]:np.testing.assert_array_equal(ours,ns['ROBOT_JOINT_TORQUE_'+key]['piper'])
  for j in range(1,7):
   _,wire,actual=encode_mit(j,.2,1.);self.assertAlmostEqual(wire,1./WIRE_TO_JOINT[j-1]);self.assertLessEqual(abs(actual-1.),16/255*WIRE_TO_JOINT[j-1])
 def test_bytes_match_pinned_official_parser_and_installed_sdk_quantizer(self):
  from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2
  nc=C_PiperParserV2();tree=ast.parse((BASE/'assets/mit_official/parser.py').read_text());fn=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='pack_joint_mit_ctrl');ns={'nc':nc,'ArmMsgJointMitCtrl':object};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<official parser>','exec'),ns)
  rng=np.random.default_rng(12)
  for j in range(1,7):
   for _ in range(20):
    q=float(rng.uniform(-2,2));tau=float(rng.uniform(-4,4));frame,wire,_=encode_mit(j,q,tau)
    msg=types.SimpleNamespace(p_des=nc.FloatToUint(q,-12.5,12.5,16),v_des=nc.FloatToUint(0,-45,45,12),kp=0,kd=nc.FloatToUint(0,-5,5,12),t_ff=nc.FloatToUint(wire,-8,8,8))
    self.assertEqual(frame,bytes(ns['pack_joint_mit_ctrl'](None,msg)))
 def test_out_of_range_rejected_before_any_hardware_frame(self):
  h=GravityHardware.__new__(GravityHardware);sent=[];h._send_locked=lambda *args:sent.append(args);h.tx_lock=threading.Lock();h.stopped=False;h.motion_closed=False
  with self.assertRaises(ValueError):h.command_mit(np.zeros(6),np.array([100.,0,0,0,0,0]))
  self.assertFalse(sent)
  h.command_mit(np.zeros(6),np.zeros(6));self.assertEqual([x[0] for x in sent],[0x151,*range(0x15a,0x160)]);self.assertEqual(sent[0][1],[1,4,0,0xad,0,0,0,0])
  sent.clear();h.motion_closed=True
  with self.assertRaises(RuntimeError):h.command_mit(np.zeros(6),np.zeros(6))
  self.assertFalse(sent)
 def test_gravity_wrench_matches_independent_inverse_and_identified_regressor(self):
  rng=np.random.default_rng(14)
  for source in ('nominal','identified'):
   g=GravityModel(profile(source));rg,_,_=original_modules();builder=rg.RegressorBuilder(g.model,link_bodies=tuple((f'link{i}',) for i in range(1,7)))
   for _ in range(5):
    q=rng.uniform([-.5,.2,-2,-.5,-.5,-.5],[.5,2,-.2,.5,.5,.5]);expected=builder.build(q[None],np.zeros((1,6)),np.zeros((1,6)))[0]@g.theta
    np.testing.assert_allclose(g(q),expected,atol=1e-9)
 def test_simulation_has_no_hidden_gravity_after_mit_and_no_torque_after_disable(self):
  b=GravitySimulation(profile());b.arm(b.snapshot()['q']);b.command_mit(b.snapshot()['q'],np.ones(6)*.2)
  self.assertFalse(np.any(b.model.body_gravcomp));self.assertFalse(np.any(b.model.actuator_gainprm));self.assertTrue(np.any(b.data.qfrc_applied))
  b.disable();np.testing.assert_array_equal(b.data.qfrc_applied,0);self.assertFalse(b.enabled)

class GravitySequence(unittest.TestCase):
 def run_case(self,source,abort=False,stiffness=1.):
  c=profile(source,stiffness=stiffness);b=GravitySimulation(c);messages=[];k=GravityController(b,c,b.snapshot(),0,messages.append);started=False;interrupted=False;torque_seen=False;states=set()
  for n in range(30000):
   b.advance(.01);s=b.snapshot();now=s['time'];action='PREPARE' if n==10 else None
   if k.state=='READY' and not started:action='START';started=True
   if abort and k.state=='TESTING' and now-k.path_t>5 and not interrupted:action='ABORT';interrupted=True
   k.tick(now,s,action);states.add(k.state)
   if k.state=='TESTING':torque_seen=torque_seen or bool(np.any(abs(k.gravity_telemetry['sent_Nm'])>.1))
   if k.state=='FAULT':self.fail(str(k.fault_reason)+' '+str(messages[-8:]))
   if k.finished:break
  self.assertTrue(started);self.assertTrue(torque_seen);self.assertTrue(k.finished,messages[-8:]);self.assertEqual(k.state,'DONE');self.assertTrue(k.disarm_confirmed);self.assertFalse(np.any(b.snapshot()['enabled']));self.assertFalse(np.any(b.data.qfrc_applied));self.assertIn('RETURNING',states)
  if abort:self.assertTrue(k.aborted);self.assertTrue(interrupted)
 def test_nominal_complete(self):self.run_case('nominal')
 def test_identified_complete(self):self.run_case('identified')
 def test_abort_in_mit_returns_then_disables(self):self.run_case('nominal',True)
 def test_no_position_spring_still_returns_cleanly(self):self.run_case('nominal',stiffness=0.)

if __name__=='__main__':unittest.main()
