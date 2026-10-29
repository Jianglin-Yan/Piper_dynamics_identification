import copy, json, tempfile, time, unittest
from pathlib import Path
import numpy as np
from real_robot.controller import Controller
from real_robot.gamepad import Chords
from real_robot.trajectory import Excitation, transit, brake, check_path
from real_robot.run import validate_config

BASE=Path(__file__).resolve().parents[1]

def tick(k,now,s,action=None):
    # Fake transport publishes fresh frames at the explicitly supplied control time.
    s.setdefault('q_times',np.full(6,now));s.setdefault('low_times',np.full(6,now))
    k.tick(now,s,action)

def config():return json.loads((BASE/'config.json').read_text())
class Fake:
    def __init__(self,c):
        self.q=np.deg2rad(c['simulation_initial_deg']);self.v=np.zeros(6);self.enabled=False;self.active=False;self.commands=[];self.stops=[];self.disables=0;self.bad=False
    def snapshot(self):return {'q':self.q.copy(),'qd':self.v.copy(),'enabled':np.full(6,self.enabled)}
    def check(self,s,active):
        if self.bad:raise RuntimeError('injected feedback failure')
    def arm(self,q):self.enabled=True;self.active=True
    def command(self,q):
        self.commands.append(np.array(q));self.v=(q-self.q)/.01;self.q=np.array(q)
    def quick_stop(self,r):self.stops.append(r);self.active=False
    def disable(self):self.enabled=False;self.active=False;self.disables+=1
    def confirm_disabled(self):self.active=False
    def finish(self):self.active=False

class TestChords(unittest.TestCase):
    def setUp(self):self.c=Chords({'LB':310,'RB':311,'X':308,'Y':307})
    def test_held_on_launch_never_starts(self):
        self.assertIsNone(self.c.update({310,307},0));self.assertIsNone(self.c.update({310,307},10))
    def test_abort_priority_no_dwell(self):self.assertEqual(self.c.update({310,311,308,307},0),'ABORT')
    def test_release_required_between_prepare_start(self):
        self.c.update(set(),0)
        self.assertEqual(self.c.update({310,308},1),'PREPARE')
        self.assertIsNone(self.c.update({310,308},1.2))
        self.assertIsNone(self.c.update({310,307},2));self.assertIsNone(self.c.update({310,307},3))
        self.c.update(set(),4);self.assertEqual(self.c.update({310,307},5),'START')
    def test_extra_button_does_not_start(self):
        self.c.update(set(),0);self.assertIsNone(self.c.update({310,308,307},10))
    def test_abort_only_one_event_until_release(self):
        self.assertEqual(self.c.update({310,311},0),'ABORT');self.assertIsNone(self.c.update({310,311},1))
        self.c.update(set(),2);self.assertEqual(self.c.update({310,311},3),'ABORT')

class TestTrajectories(unittest.TestCase):
    def test_quintic_endpoints(self):
        p=transit(np.zeros(6),np.ones(6)*.2,.12,.12)
        for t,q in ((0,np.zeros(6)),(p.duration,np.ones(6)*.2)):
            x,v,a=p.at(t);np.testing.assert_allclose(x,q,atol=1e-12);np.testing.assert_allclose(v,0,atol=1e-12);np.testing.assert_allclose(a,0,atol=1e-12)
        check_path(p,np.ones(6)*-1,np.ones(6),.12,.12)
    def test_windowed_excitation_derivatives(self):
        p=Excitation(np.zeros(6),5,.05,.1,20,5)
        for t in (0,20):
            q,v,a=p.at(t);np.testing.assert_allclose(q,0,atol=1e-12);np.testing.assert_allclose(v,0,atol=1e-12);np.testing.assert_allclose(a,0,atol=1e-12)
        for t in (1.,4.,7.,16.,19.):
            h=1e-4;q,v,a=p.at(t);qp,vp,_=p.at(t+h);qm,vm,_=p.at(t-h)
            np.testing.assert_allclose((qp-qm)/(2*h),v,atol=1e-8);np.testing.assert_allclose((vp-vm)/(2*h),a,atol=1e-8)
    def test_braking_at_many_phases_is_continuous_and_bounded(self):
        rng=np.random.default_rng(11)
        for _ in range(20):
            q=rng.uniform(-.3,.3,6);v=rng.uniform(-.12,.12,6);p=brake(q,v,.12,.12)
            np.testing.assert_allclose(p.at(0)[0],q);np.testing.assert_allclose(p.at(0)[1],v)
            np.testing.assert_allclose(p.at(p.duration)[1],0,atol=1e-12)
            check_path(p,np.full(6,-1),np.ones(6),.12,.12)
    def test_boundary_brake_rejected(self):
        p=brake(np.full(6,.99),np.full(6,.12),.12,.12)
        with self.assertRaises(ValueError):check_path(p,np.full(6,-1),np.ones(6),.12,.12)

class TestController(unittest.TestCase):
    def make(self,c=None):
        c=c or config();b=Fake(c);ctrl=Controller(b,c,b.snapshot(),0,lambda x:None);return c,b,ctrl
    def step(self,b,k,t,action=None):tick(k,t,b.snapshot(),action)
    def test_no_motion_before_prepare_and_start(self):
        c,b,k=self.make();self.step(b,k,.1,'START');self.assertEqual(b.commands,[]);self.assertFalse(b.active)
        self.step(b,k,.2,'PREPARE');self.assertEqual(k.state,'READY');self.assertEqual(b.commands,[])
    def test_current_outside_limits_refuses_without_command(self):
        c=config();b=Fake(c);b.q[1]=-.05
        with self.assertRaises(ValueError):Controller(b,c,b.snapshot(),0)
        self.assertFalse(b.active);self.assertEqual(b.commands,[])
    def test_abort_before_arm_has_no_motion(self):
        c,b,k=self.make();self.step(b,k,.1,'ABORT');
        for n in range(11,40):self.step(b,k,n*.01)
        self.assertEqual(k.state,'DONE');self.assertEqual(b.commands,[])
    def test_normal_returns_to_captured_home(self):
        c,b,k=self.make();home=b.q.copy()
        for n in range(4000):
            self.step(b,k,n*.01,{1:'PREPARE',3:'START'}.get(n))
            if k.state in ('DONE','FAULT'):break
        self.assertEqual(k.state,'DONE');np.testing.assert_allclose(b.q,home,atol=1e-9)
    def test_special_start_transition_and_return(self):
        c=config();c['start_deg']=[0,45,-60,0,20,2];_,b,k=self.make(c);home=b.q.copy();started=False
        for n in range(6000):
            action='PREPARE' if n==1 else None
            if k.state=='READY' and not started:action='START';started=True
            self.step(b,k,n*.01,action)
            if k.state in ('DONE','FAULT'):break
        self.assertTrue(started);self.assertEqual(k.state,'DONE');np.testing.assert_allclose(b.q,home,atol=1e-9)
    def test_abort_each_phase_returns_home(self):
        for abort_time in (1.,5.,10.,16.,20.5):
            c,b,k=self.make();home=b.q.copy()
            for n in range(4000):
                action={1:'PREPARE',3:'START',round(abort_time*100):'ABORT'}.get(n)
                self.step(b,k,n*.01,action)
                if k.state in ('DONE','FAULT'):break
            self.assertEqual(k.state,'DONE');self.assertTrue(k.aborted);np.testing.assert_allclose(b.q,home,atol=1e-9)
    def test_bad_feedback_never_returns_or_reenables(self):
        c,b,k=self.make();self.step(b,k,.01,'PREPARE');self.step(b,k,.03,'START');b.bad=True
        before=len(b.commands);self.step(b,k,.04,'ABORT');self.assertEqual(k.state,'FAULT');self.assertEqual(len(b.stops),1)
        self.step(b,k,.05,'START');self.assertEqual(len(b.commands),before)
    def test_disable_during_ready_faults(self):
        c,b,k=self.make();k.active=True;k.state='READY';b.enabled=False;self.step(b,k,.1)
        self.assertEqual(k.state,'FAULT');self.assertEqual(len(b.stops),1)
    def test_repeated_abort_in_return_is_honored(self):
        c,b,k=self.make();self.step(b,k,.01,'PREPARE');self.step(b,k,.03,'START');self.step(b,k,.04)
        k.aborted=True;k.state='RETURNING';self.step(b,k,.05,'ABORT');self.assertEqual(k.state,'RETURNING')
    def test_large_config_refused(self):
        c=config();c['amplitude_deg']=60
        with self.assertRaises(ValueError):validate_config(c)

if __name__=='__main__':unittest.main()
