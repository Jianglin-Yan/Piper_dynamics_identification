import unittest,numpy as np
from real_robot.controller import Controller
from real_robot.standing import standing_plan,STANDING_DEG
from test_pilot import Fake,config,tick
from test_current_pose import PARK

class NativeFake(Fake):
    def __init__(self,c,move=True):super().__init__(c);self.move=move;self.native_active=False;self.native_calls=[]
    def begin_native_move(self,q):
        self.native_active=True;self.native_calls.append(q.copy())
        if self.move:self.q=q.copy();self.v[:]=0
    def finish_native_move(self):self.native_active=False
    def command(self,q):
        self.commands.append(q.copy())
        if self.move:self.v=(q-self.q)/.01;self.q=q.copy()

class TestStanding(unittest.TestCase):
    def make(self,move=True):
        c=config();c.update(standing_start=True,current_pose_only=False,start_deg=STANDING_DEG,simulation_initial_deg=PARK.copy())
        b=NativeFake(c,move);messages=[];k=Controller(b,c,b.snapshot(),0,messages.append)
        return c,b,k,messages
    def test_prepare_reaches_fixed_pose_before_ready(self):
        c,b,k,_=self.make();self.assertFalse(b.active);self.assertEqual(b.commands,[])
        for n in range(3000):
            tick(k,n*.01,b.snapshot(),'PREPARE' if n==1 else None)
            if k.state in ('READY','FAULT'):break
        self.assertEqual(k.state,'READY');self.assertEqual(len(b.native_calls),1)
        np.testing.assert_allclose(np.rad2deg(b.q),STANDING_DEG,atol=1e-9)
        goal=np.rad2deg(b.native_calls[0]);self.assertEqual(goal[1],0);self.assertEqual(goal[2],0)
        self.assertLessEqual(max(abs(goal-np.array(PARK))),3)
    def test_stuck_arm_cannot_get_ready_or_start_test(self):
        c,b,k,m=self.make(move=False)
        for n in range(1100):
            tick(k,n*.01,b.snapshot(),'PREPARE' if n==1 else 'START' if n==500 else None)
            if k.state=='FAULT':break
        self.assertEqual(k.state,'FAULT');self.assertTrue(any('position control not proven' in s for s in m));self.assertEqual(len(b.stops),1)
        self.assertFalse(any(s=='TESTING' for s in m))
    def test_trial_finishes_at_nearest_legal_park_not_standing(self):
        c,b,k,_=self.make();started=False;home=b.q.copy()
        for n in range(8000):
            action='PREPARE' if n==1 else None
            if k.state=='READY' and not started:action='START';started=True
            tick(k,n*.01,b.snapshot(),action)
            if k.state in ('DONE','FAULT'):break
        self.assertTrue(started);self.assertEqual(k.state,'DONE');np.testing.assert_allclose(b.q,k.return_home,atol=1e-9)
        np.testing.assert_allclose(k.home,home,atol=1e-9)
        np.testing.assert_allclose(k.return_home[1:3],0,atol=1e-9)
    def test_abort_while_entering_restores_native_mode_and_returns(self):
        c,b,k,_=self.make();home=b.q.copy();tick(k,.01,b.snapshot(),'PREPARE');tick(k,.02,b.snapshot())
        self.assertEqual(k.state,'BOUNDARY_ENTRY');tick(k,.03,b.snapshot(),'ABORT');self.assertFalse(b.native_active)
        for n in range(4,1000):
            tick(k,n*.01,b.snapshot())
            if k.state in ('DONE','FAULT'):break
        self.assertEqual(k.state,'DONE');np.testing.assert_allclose(b.q,k.return_home,atol=1e-9)
    def test_unknown_large_park_excursion_refused(self):
        c,b,k,_=self.make();q=k.home.copy();q[1]=np.deg2rad(-4)
        with self.assertRaises(ValueError):standing_plan(c,q)

if __name__=='__main__':unittest.main()
