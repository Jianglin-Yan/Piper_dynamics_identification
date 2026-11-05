import json,tempfile,threading,unittest
from pathlib import Path
import numpy as np
from piper_dynamics.real_robot.controller import Controller
from piper_dynamics.real_robot.backends.hardware import Hardware
from piper_dynamics.real_robot.app import validate_config
from piper_dynamics.real_robot.trials.validation import configure,LOADS_KG,summarize
from piper_dynamics.real_robot.recording import TestRecorder,export_test_data
from tests.real_robot.test_pilot import Fake,config


def profile(kind):
 c=config();c.update(trajectory=kind,standing_start=True);configure(c)
 c['validation_settle_s']=1.;c['validation_capture_s']=2.;return c

class PassiveFake(Fake):
 def __init__(self,c):super().__init__(c);self.now=0.;self.grip=12000;self.forced_enable=False;self.passive=c.get('passive_wrist',False)
 def snapshot(self):
  return dict(time=self.now,q=self.q.copy(),qd=self.v.copy(),current_A=np.zeros(6),enabled=np.array([self.enabled]*5+[self.forced_enable if self.passive else self.enabled]),faults=[0]*6,
   q_times=np.full(6,self.now),drive_times=np.full(6,self.now),low_times=np.full(6,self.now),status={'arm_status':0,'error_code':0,'control_mode':1},gripper={'gripper_position_raw':self.grip},temp_C=[25]*6,voltage_V=[24]*6)
 def command(self,q):
  p=np.asarray(q).copy()
  if self.passive:p[5]=self.q[5]
  super().command(p)

def make(kind):
 c=profile(kind);b=PassiveFake(c);messages=[];k=Controller(b,c,b.snapshot(),0,messages.append);return c,b,k,messages

def step(b,k,n,action=None):b.now=n*.01;k.tick(b.now,b.snapshot(),action)

class ProtocolTests(unittest.TestCase):
 def test_configs_reject_scaling_and_no_standing(self):
  for kind in ('static','reciprocal','payload'):
   c=profile(kind);validate_config(c);c['trajectory_scale']=.5
   with self.assertRaises(ValueError):validate_config(c)
  c=profile('static');c['standing_start']=False
  with self.assertRaises(ValueError):validate_config(c)
 def test_static_tracking_warning_and_independent_hard_line(self):
  for error,expected in [(12.,'READY'),(36.,'BRAKING')]:
   c,b,k,messages=make('static');b.q=k.start.copy();k.last_target=k.start.copy();k.state='READY';k.active=True;b.active=True;b.enabled=True
   for n in range(10,40):
    b.q=k.start.copy();b.q[1]+=np.deg2rad(error);b.v[:]=0.;step(b,k,n)
   self.assertEqual(k.state,expected)
   if error<25:self.assertTrue(any('TRACKING_WARNING' in m for m in messages))
 def test_feedback_margin_is_warn_only_and_targets_still_bounded(self):
  c,b,k,messages=make('static');b.q=k.start.copy();k.last_target=k.start.copy();k.state='READY';k.active=True;b.active=True;b.enabled=True
  # Measured feedback can briefly exceed the planning interior without changing command bounds.
  b.q[0]=k.upper[0]+np.deg2rad(1);k.last_target[0]=k.upper[0]
  step(b,k,10);self.assertNotEqual(k.state,'FAULT')
  q=k.start.copy();q[0]=k.upper[0]+np.deg2rad(1)
  with self.assertRaises(ValueError):k.command(q)
 def test_feedback_extension_is_from_original_limits_for_all_validation_scripts(self):
  for kind in ('static','reciprocal','payload'):
   for direction in (-1,1):
    for overshoot,should_fault in ((9.9,False),(10.1,True)):
     c,b,k,messages=make(kind);b.q=k.start.copy();k.last_target=k.start.copy();k.state='READY';k.active=True;b.active=True;b.enabled=True
     raw=c['upper_deg'][0] if direction>0 else c['lower_deg'][0]
     b.q[0]=np.deg2rad(raw+direction*overshoot)
     k.last_target[0]=k.upper[0] if direction>0 else k.lower[0]
     step(b,k,10)
     self.assertEqual(k.state=='FAULT',should_fault,(kind,direction,overshoot,messages))
 def test_test_phase_excludes_extra_startup_corridor(self):
  c,b,k,_=make('static');k.lower[1]=np.deg2rad(c['lower_deg'][1]-10.)
  k.state='PREPARING';self.assertLessEqual(k.measurement_bounds()[0][1],np.deg2rad(-10.))
  k.state='TESTING';self.assertAlmostEqual(np.rad2deg(k.measurement_bounds()[0][1]),c['lower_deg'][1]-c['measurement_angle_tolerance_deg'])
 def test_exact_time_reverse(self):
  c,b,k,_=make('reciprocal');f=k.trial.forward;r=k.trial.reverse
  for t in np.linspace(0,f.duration,61):
   q,v,a=f.at(f.duration-t);qr,vr,ar=r.at(t)
   np.testing.assert_allclose(qr,q);np.testing.assert_allclose(vr,-v);np.testing.assert_allclose(ar,a)
  np.testing.assert_allclose(r.at(r.duration)[0],k.start)
 def test_no_commands_before_prepare(self):
  c,b,k,_=make('payload');step(b,k,1,'START');self.assertFalse(b.active);self.assertFalse(b.commands)
  step(b,k,2,'PREPARE');self.assertEqual(k.state,'ARMING')
  for n in range(3,1200):
   step(b,k,n)
   if k.state=='TESTING':break
  self.assertEqual(k.state,'TESTING');self.assertTrue(b.snapshot()['enabled'][5])
 def test_static_sequence_controls_six_axes_and_returns_all(self):
  c,b,k,_=make('static');started=False;captures=set()
  for n in range(60000):
   action='PREPARE' if n==1 else None
   if k.state=='READY' and not started:action='START';started=True
   step(b,k,n,action)
   if k.trial.phase=='capture':captures.add(k.trial.capture_id)
   if k.finished:break
  self.assertTrue(k.finished);self.assertEqual(k.state,'DONE');self.assertFalse(k.fault_reason)
  self.assertEqual(len(captures),9);self.assertTrue(k.disarm_confirmed);self.assertAlmostEqual(b.q[5],0.)
  self.assertTrue(max(abs(p[5]) for p in b.commands)>np.deg2rad(70))
 def test_payload_preserves_nonzero_startup_J6_through_prepare_pose_changes_and_return(self):
  from tests.real_robot.test_standing import NativeFake
  c=profile('payload');c['simulation_initial_deg']=[3.,-3.081,3.101,3.,20.,-112.694]
  b=NativeFake(c);messages=[];k=Controller(b,c,b.snapshot(),0,messages.append)
  visited=set();angle=k.home[5];changes=set()
  for n in range(18000):
   action='PREPARE' if n==1 else None
   if k.state=='TESTING':
    t=k.trial
    if t.phase=='capture':visited.add(t.pose_id)
    if t.completed_windows>=1 and 0 not in changes:action='START';changes.add(0)
    elif t.completed_windows>=2 and 1 not in changes:action='START';changes.add(1)
    elif t.completed_windows>=3:action='ABORT'
   observation=b.snapshot();observation.update(q_times=np.full(6,n*.01),low_times=np.full(6,n*.01))
   k.tick(n*.01,observation,action)
   if k.finished:break
  self.assertTrue(k.finished);self.assertEqual(k.state,'DONE');self.assertEqual(visited,{0,1,2})
  np.testing.assert_allclose(np.array(b.commands)[:,5],angle,atol=1e-12)
  self.assertTrue(k.disarm_confirmed);self.assertFalse(b.enabled)
  q=k.home.copy();q[5]+=.001
  with self.assertRaises(ValueError):k.command(q)
 def test_payload_free_loading_next_pose_only_and_no_invented_mass(self):
  c,b,k,_=make('payload');started=False;poses=set();held_n=0;sent_next=False
  for n in range(15000):
   action='PREPARE' if n==1 else None
   if k.state=='READY' and not started:action='START';started=True
   if k.state=='TESTING':
    t=k.trial;self.assertIsNone(t.confirmed_mass)
    if t.phase=='capture':poses.add(t.pose_id)
    if t.completed_windows>=2 and not sent_next:
     held_n+=1
     self.assertEqual(t.pose_id,0)
     if held_n>=100:action='START';sent_next=True
    elif sent_next and t.pose_id==1 and t.completed_windows>=3:action='ABORT'
   step(b,k,n,action)
   if k.finished:break
  self.assertTrue(k.finished);self.assertEqual(k.state,'DONE');self.assertTrue(k.aborted)
  self.assertEqual(poses,{0,1});self.assertGreaterEqual(held_n,100)
 def test_adjustment_restarts_capture_without_fault(self):
  c,b,k,_=make('payload');started=False
  for n in range(4000):
   action='PREPARE' if n==1 else None
   if k.state=='READY' and not started:action='START';started=True
   step(b,k,n,action)
   if k.trial.phase=='capture' and k.trial.steady:break
  self.assertEqual(k.trial.phase,'capture');b.q[5]+=.1;b.grip+=2000
  step(b,k,n+1);self.assertEqual(k.trial.phase,'settling');self.assertFalse(k.trial.steady)
  self.assertFalse(k.fault_reason);self.assertAlmostEqual(b.commands[-1][5],k.home[5])
 def test_payload_unexpected_motor_disable_faults(self):
  c,b,k,_=make('payload');step(b,k,1,'PREPARE')
  for n in range(2,1000):
   step(b,k,n)
   if k.state=='TESTING':break
  b.enabled=False;step(b,k,n+1);self.assertTrue(k.fault_reason);self.assertEqual(k.state,'FAULT')
 def test_abort_all_protocols(self):
  for kind in ('static','reciprocal','payload'):
   c,b,k,_=make(kind);started=False;aborted=False
   for n in range(14000):
    action='PREPARE' if n==1 else None
    if k.state=='READY' and not started:action='START';started=True
    if k.state=='TESTING' and not aborted and (k.trial.phase=='moving' or n>2000):action='ABORT';aborted=True
    step(b,k,n,action)
    if k.finished:break
   self.assertEqual(k.state,'DONE',kind);self.assertTrue(k.aborted);self.assertTrue(k.disarm_confirmed)
 def test_recorder_accepts_enabled_wrist_and_exports_labels(self):
  c,b,k,_=make('payload');started=False
  with tempfile.TemporaryDirectory() as td:
   p=Path(td);(p/'config.json').write_text(json.dumps(c));rec=TestRecorder(p,c,'sim',version={})
   for n in range(3000):
    action='PREPARE' if n==1 else None
    if k.state=='READY' and not started:action='START';started=True
    step(b,k,n,action);rec.update(k,b.now,b.snapshot(),b.now,action)
    if k.trial.phase=='wait_load':break
   rec.close(b.now,'test');quality=export_test_data(p);self.assertGreater(quality['valid_measurement_ratio'],.99)
   with np.load(p/'test_signals.npz') as d:
    self.assertIn('trial_mass_kg',d);self.assertIn('trial_pair_time_s',d)
    self.assertTrue(d['enabled'][:,5].all());self.assertTrue(d['trial_steady_candidate'].any())

class PassiveTransportTests(unittest.TestCase):
 def test_enables_all_six_joints_releases_only_gripper_and_does_not_reset(self):
  h=Hardware.__new__(Hardware);h.c=profile('payload');h.tx_lock=threading.Lock();h.rx_lock=threading.Lock()
  h.latest={0x2a8:{'gripper_position_raw':12345}};h.stopped=False;h.motion_closed=False;h.active=False;sent=[]
  h._send_locked=lambda cid,data:sent.append((cid,bytes(data)))
  h.arm(np.deg2rad([0,45,-60,0,0,15]))
  self.assertEqual([d[0] for cid,d in sent if cid==0x471 and d[1]==2],[1,2,3,4,5,6])
  self.assertFalse(any(cid==0x471 and d[1]==1 for cid,d in sent))
  grip=[d for cid,d in sent if cid==0x159];self.assertEqual(len(grip),1);self.assertEqual(grip[0][6:],b'\0\0')
  self.assertFalse(any(cid==0x150 for cid,d in sent))
  sent.clear();h.disable();self.assertEqual([d[:2] for cid,d in sent],[bytes([j,1]) for j in range(1,7)])

class SummaryTests(unittest.TestCase):
 def test_free_wrist_geometry_change_is_not_a_same_pose_load_pair(self):
  with tempfile.TemporaryDirectory() as td:
   p=Path(td);c=profile('payload');(p/'config.json').write_text(json.dumps(c));(p/'test_manifest.json').write_text(json.dumps({'mode':'sim'}))
   rows=[]
   for capture,mass,wrist,current in [(1,0.,0.,.1),(2,.09,.1,.2)]:
    for t in np.linspace(0,2,5):
     rows.append(dict(t_monotonic_s=float(t+capture*3),quality={'valid_measurement':True},trial=dict(capture_id=capture,phase='capture',steady_candidate=True,pose_id=1,visit=0,direction='none',mass_kg=mass),feedback=dict(q=[0,0,0,0,0,wrist],current_A=[current]*6,temp_C=[25]*6,gripper={'gripper_position_raw':12000})))
   (p/'test_samples.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in rows));r=summarize(p)
   self.assertEqual(len(r['payload_differences']),1);self.assertFalse(r['payload_differences'][0]['same_geometry_for_direct_pairing'])
   np.testing.assert_allclose(r['payload_differences'][0]['delta_current_A'],.1)
   self.assertFalse(r['torque_calibrated'])

if __name__=='__main__':unittest.main()
