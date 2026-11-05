"""MIT-only lifecycle, raw PD encoding, FF ramps and controlled interruption."""
import ast,json,threading,types,unittest
from pathlib import Path
import numpy as np
from piper_dynamics.real_robot.control.gravity_hold import BASE,WIRE_TO_JOINT,encode_mit,uint
from piper_dynamics.real_robot.control.mit_trial import (configure,MitController,MitHardware,MitSimulation,
    bumpless_reference)
from piper_dynamics.real_robot.app import validate_config


def profile(alpha=0.,folded=False):
 c=json.loads((BASE/'config.json').read_text());c.update(trajectory='mit',standing_start=True,start_deg=[0,45,-60,0,0,0])
 # Folded start still outside the J2/J3 nominal limits, but small enough that the
 # simulated pure-PD sag stays inside the 6 deg feedback hard bound.
 if folded:c['simulation_initial_deg']=[1.,-1.,1.,1.,20.,0.]
 configure(c,alpha);validate_config(c);return c

class MitWireTests(unittest.TestCase):
 def test_nonzero_pd_bytes_match_official_encoder(self):
  from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2
  nc=C_PiperParserV2();fn=next(n for n in ast.walk(ast.parse((BASE/'assets/mit_official/parser.py').read_text())) if isinstance(n,ast.FunctionDef) and n.name=='pack_joint_mit_ctrl');ns={'nc':nc,'ArmMsgJointMitCtrl':object};exec(compile(ast.Module(body=[fn],type_ignores=[]),'<official parser>','exec'),ns)
  rng=np.random.default_rng(20)
  for j in range(1,7):
   for _ in range(10):
    q,v,kp,kd,ff=[float(x) for x in rng.uniform([-2,-1,1,.1,-2],[2,1,50,2,2])];frame,wire,_=encode_mit(j,q,ff,v,kp,kd)
    m=types.SimpleNamespace(p_des=nc.FloatToUint(q,-12.5,12.5,16),v_des=nc.FloatToUint(v,-45,45,12),kp=nc.FloatToUint(kp,0,500,12),kd=nc.FloatToUint(kd,-5,5,12),t_ff=nc.FloatToUint(wire,-8,8,8))
    self.assertEqual(frame,bytes(ns['pack_joint_mit_ctrl'](None,m)))
 def test_hardware_seeds_measured_pose_and_uses_only_mit_enable_disable(self):
  c=profile();h=MitHardware.__new__(MitHardware);h.c=c;h.stopped=False;h.motion_closed=False;h.tx_lock=threading.Lock();sent=[];h._send_locked=lambda cid,data:sent.append((cid,bytes(data)))
  q=np.deg2rad([2,-3,4,2,20,0]);h.arm(q)
  self.assertEqual([cid for cid,_ in sent],[0x151,*range(0x15a,0x160),*[0x471]*6])
  for j,(_,frame) in enumerate(sent[1:7]):
   self.assertEqual(frame,encode_mit(j+1,float(q[j]),0.,0.,c['mit_prepare_kp'][j],c['mit_prepare_kd'][j])[0])
  h.command_pd(q,np.zeros(6),np.array(c['mit_return_kp']),np.array(c['mit_return_kd']),np.zeros(6));h.disable()
  self.assertFalse(any(cid in (0x155,0x156,0x157,0x150,0x474,0x475) for cid,_ in sent))
  self.assertTrue(all(data[1]==4 and data[3]==0xad for cid,data in sent if cid==0x151))
 def test_abort_reference_preserves_requested_protocol_torque_with_zero_ff(self):
  rng=np.random.default_rng(21)
  for _ in range(50):
   q,v,qr,vr=[rng.uniform(-.2,.2,6) for _ in range(4)];kp=rng.uniform(5,15,6);kd=rng.uniform(.2,.8,6);ff=rng.uniform(-1,1,6);newkp=kp+10.;newkd=kd+.5
   nr,nv=bumpless_reference(q,v,qr,vr,kp,kd,ff,newkp,newkd)
   np.testing.assert_allclose(newkp*(nr-q)+newkd*(nv-v),kp*(qr-q)+kd*(vr-v)+ff,atol=1e-12)
 def test_invalid_command_emits_no_partial_frame(self):
  h=MitHardware.__new__(MitHardware);h.tx_lock=threading.Lock();h.stopped=False;h.motion_closed=False;sent=[];h._send_locked=lambda *args:sent.append(args)
  for kp in [np.array([10.]*5+[501.]),np.array([10.]*5+[np.nan])]:
   with self.assertRaises(ValueError):h.command_pd(np.zeros(6),np.zeros(6),kp,np.ones(6),np.zeros(6))
  self.assertFalse(sent)

class MitLifecycle(unittest.TestCase):
 def run_case(self,alpha=0.,folded=False,abort_phase=None,model='nominal'):
  c=profile(alpha,folded);c['gravity_model']=model;b=MitSimulation(c);messages=[];k=MitController(b,c,b.snapshot(),0,messages.append);started=False;aborted=False;phases=set();trace=[];last=None
  for n in range(30000):
   b.advance(.01);s=b.snapshot();now=s['time'];action='PREPARE' if n==10 else None
   if k.state=='READY' and not started:action='START';started=True
   if abort_phase and k.state==abort_phase and not aborted:
    # Let FF actually become nonzero before interruption of the entry ramp.
    if abort_phase!='FF_RAMP_IN' or now-k.ff_phase_t>.8:action='ABORT';aborted=True
   k.tick(now,s,action);phases.add(k.state)
   if k.mit_telemetry is not None:
    tag=k.mit_telemetry;trace.append((now,k.state,tag['ff_ratio'],tag['kp_protocol'].copy(),tag['ff_requested_Nm'].copy()))
    if aborted:self.assertEqual(k.mit_ff_ratio,0.);np.testing.assert_array_equal(tag['ff_requested_Nm'],0.)
   if k.state=='FAULT':self.fail(str(k.fault_reason)+' '+str(messages[-8:]))
   if k.finished:break
   last=s
  self.assertTrue(k.finished,messages[-8:]);self.assertTrue(k.disarm_confirmed);self.assertFalse(b.enabled);self.assertFalse(np.any(b.data.qfrc_applied));self.assertFalse(np.any(b.model.body_gravcomp));self.assertFalse(np.any(b.model.actuator_gainprm));self.assertNotIn('BOUNDARY_ENTRY',phases)
  if abort_phase:self.assertTrue(aborted);self.assertTrue(k.aborted)
  else:
   self.assertTrue(started);self.assertIn('FF_RAMP_OUT',phases);self.assertIn('RETURNING',phases)
   entry=[x for x in trace if x[1]=='FF_RAMP_IN'];exit_=[x for x in trace if x[1]=='FF_RAMP_OUT'];self.assertGreaterEqual(entry[-1][0]-entry[0][0],2.97);self.assertGreaterEqual(exit_[-1][0]-exit_[0][0],2.97)
   self.assertTrue(np.all(np.diff([x[2] for x in entry])>=-1e-12));self.assertTrue(np.all(np.diff([x[2] for x in exit_])<=1e-12))
  if alpha==0:
   self.assertIsNone(k.gravity)
   for x in trace:np.testing.assert_array_equal(x[4],0.)
  return k,trace
 def test_pure_pd_complete(self):self.run_case()
 def test_forward_fraction_complete(self):self.run_case(.25)
 def test_full_gravity_fraction_complete(self):self.run_case(1.)
 def test_folded_outside_nominal_start_is_mit_only(self):self.run_case(0.,True)
 def test_identified_forward_fraction_complete(self):self.run_case(.25,model='identified')
 def test_abort_all_active_phases(self):
  for phase in ('ARMING','PREPARING','FF_RAMP_IN','READY','TESTING','FF_RAMP_OUT'):
   with self.subTest(phase=phase):self.run_case(.25,abort_phase=phase)
 def test_fixed_path_is_identical_for_pd_and_ff(self):
  paths=[]
  for alpha in (0.,.25):
   c=profile(alpha);b=MitSimulation(c);k=MitController(b,c,b.snapshot(),0,lambda m:None);paths.append(k.test)
  self.assertEqual(paths[0].duration,paths[1].duration)
  for t in np.linspace(0,paths[0].duration,31):
   for a,b in zip(paths[0].at(t),paths[1].at(t)):np.testing.assert_array_equal(a,b)

class MitTorqueBudget(unittest.TestCase):
 def make(self):
  c=profile();b=MitSimulation(c);k=MitController(b,c,b.snapshot(),0,lambda m:None)
  k.mit_kp=np.array(c['mit_kp']);k.mit_kd=np.array(c['mit_kd']);k.mit_ff_ratio=0.;return c,k
 def test_velocity_spike_is_clamped_out_of_the_estimate(self):
  c,k=self.make();q=k.start.copy();k.mit_sample={'q':q.copy(),'qd':np.zeros(6)}
  k.mit_sample['qd'][1]=-3.0;k.command(q)
  self.assertEqual(k.budget_cycles,0)
 def test_single_cycle_exceedance_does_not_fault(self):
  c,k=self.make();q=k.start.copy();q[1]+=np.deg2rad(30.);k.mit_sample={'q':k.start.copy(),'qd':np.zeros(6)}
  k.command(q);self.assertEqual(k.budget_cycles,1)
  k.command(k.start.copy());self.assertEqual(k.budget_cycles,0)
 def test_sustained_exceedance_reports_joint_and_value(self):
  c,k=self.make();q=k.start.copy();q[1]+=np.deg2rad(30.);k.mit_sample={'q':k.start.copy(),'qd':np.zeros(6)}
  with self.assertRaisesRegex(ValueError,r'J2 \+\d+\.\d\d Nm vs 16\.0 Nm for 0\.10 s'):
   for _ in range(k.budget_limit):k.command(q)

 def test_test_path_covers_all_joints_and_is_long_enough(self):
  c=profile();b=MitSimulation(c);k=MitController(b,c,b.snapshot(),0,lambda m:None)
  t=np.linspace(0,k.test.duration,400);q=np.array([k.test.at(x)[0] for x in t]);rng=np.rad2deg(np.ptp(q,axis=0))
  self.assertGreaterEqual(k.test.duration,15.)
  self.assertTrue(np.all(rng>8.),rng)

if __name__=='__main__':unittest.main()
