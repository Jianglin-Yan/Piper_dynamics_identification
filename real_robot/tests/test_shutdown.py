import threading
import unittest
from unittest.mock import Mock
import numpy as np
from real_robot.controller import Controller
from real_robot.hardware import Hardware
from real_robot.simulation import Simulation
from real_robot.tests.test_pilot import Fake,config,tick


class DelayedDisable(Fake):
    def disable(self):self.disables+=1;self.active=False


class TestShutdown(unittest.TestCase):
    def make(self,delayed=False):
        c=config();b=(DelayedDisable if delayed else Fake)(c);events=[]
        k=Controller(b,c,b.snapshot(),0,events.append)
        return c,b,k,events

    def powered(self,b,k):
        b.enabled=True;b.active=True;k.active=True;k.armed=True

    def test_normal_and_abort_and_stop_disable_only_after_park(self):
        for stop in (None,'ABORT','STOP'):
            c,b,k,e=self.make();release=[];real_disable=b.disable
            def disable():release.append((b.q.copy(),len(b.commands)));real_disable()
            b.disable=disable
            for n in range(4000):
                action={1:'PREPARE',3:'START',525:stop}.get(n)
                tick(k,n*.01,b.snapshot(),action)
                if k.finished:break
            self.assertTrue(k.finished);self.assertTrue(k.disarm_confirmed)
            self.assertEqual(k.state,'DONE');self.assertFalse(b.enabled);self.assertFalse(b.active)
            self.assertEqual(b.stops,[]);self.assertEqual(len(release),1)
            np.testing.assert_allclose(release[0][0],k.return_home,atol=1e-9)
            self.assertEqual(len(b.commands),release[0][1])

    def test_prepare_abort_returns_without_beginning_test(self):
        c=config();c['start_deg']=[0,50,-65,0,15,0];b=Fake(c);k=Controller(b,c,b.snapshot(),0,lambda _:None)
        seen=[]
        for n in range(3000):
            tick(k,n*.01,b.snapshot(),{1:'PREPARE',120:'ABORT'}.get(n));seen.append(k.state)
            if k.finished:break
        self.assertNotIn('TESTING',seen);self.assertTrue(k.finished);self.assertFalse(b.enabled)
        np.testing.assert_allclose(b.q,k.home,atol=1e-9)

    def test_waiting_stop_disables_inherited_enable_without_moving(self):
        c,b,k,e=self.make();b.enabled=True
        for n in range(100):tick(k,n*.01,b.snapshot(),'ABORT' if n==1 else None)
        self.assertTrue(k.finished);self.assertFalse(b.enabled);self.assertEqual(b.commands,[])

    def test_sent_disable_and_stale_or_partial_feedback_are_not_confirmation(self):
        c,b,k,e=self.make(delayed=True);self.powered(b,k);k.begin_disarm(b.snapshot(),0)
        for n in range(1,310):tick(k,n*.01,b.snapshot())
        self.assertFalse(k.finished);self.assertTrue(b.enabled);self.assertGreater(b.disables,1)
        self.assertEqual(k.shutdown_phase,'DISABLE_UNCONFIRMED')
        s=b.snapshot();s['enabled'][:]=False;s['low_times']=np.zeros(6)
        k.tick(3.2,s);self.assertFalse(k.finished)
        s['low_times']=np.full(6,3.3);s['enabled'][2]=True
        k.tick(3.3,s);self.assertFalse(k.finished)
        b.enabled=False
        for n in range(340,370):tick(k,n*.01,b.snapshot())
        self.assertTrue(k.finished);self.assertTrue(k.disarm_confirmed)

    def test_disable_send_failure_does_not_exit_or_issue_position(self):
        c,b,k,e=self.make();self.powered(b,k)
        b.disable=Mock(side_effect=OSError('CAN unavailable'));k.begin_disarm(b.snapshot(),0)
        for n in range(1,310):tick(k,n*.01,b.snapshot())
        self.assertFalse(k.finished);self.assertEqual(b.commands,[]);self.assertTrue(b.enabled)
        self.assertEqual(k.shutdown_phase,'DISABLE_UNCONFIRMED')

    def test_inexact_but_stationary_park_releases_after_precision_timeout(self):
        c,b,k,e=self.make();self.powered(b,k);k.state='SETTLING_HOME';k.settle_started=0
        b.q[5]+=np.deg2rad(1.);b.command=lambda q:b.commands.append(q.copy())
        for n in range(1100):
            tick(k,n*.01,b.snapshot())
            if k.finished:break
        self.assertTrue(k.finished);self.assertFalse(b.enabled)
        self.assertTrue(any('PARK_NEAR' in x for x in e))
        self.assertAlmostEqual(np.rad2deg(k.release_q[5]-k.home[5]),1.)

    def test_fault_far_from_park_does_not_release_or_exit_or_return(self):
        c,b,k,e=self.make();self.powered(b,k);b.q[1]+=.3;k.fault('driver failure');count=len(b.commands)
        for n in range(1200):tick(k,n*.01,b.snapshot())
        self.assertEqual(k.state,'FAULT');self.assertFalse(k.finished)
        self.assertEqual(b.disables,0);self.assertEqual(len(b.commands),count)
        b.q=k.return_home.copy()
        for n in range(1200,1300):tick(k,n*.01,b.snapshot())
        self.assertTrue(k.finished);self.assertEqual(k.state,'FAULT');self.assertFalse(b.enabled)

    def test_fault_with_stale_pose_cannot_claim_a_safe_park(self):
        c,b,k,e=self.make();self.powered(b,k);k.fault('receiver failed')
        for n in range(1,100):
            s=b.snapshot();s['q_times']=np.zeros(6);s['low_times']=np.zeros(6);k.tick(n*.01,s)
        self.assertFalse(k.finished);self.assertEqual(b.disables,0)

    def test_position_drift_and_far_position_cannot_authorize_release(self):
        c,b,k,e=self.make();self.powered(b,k);k.park_for_shutdown(0,'stop')
        for n in range(1200):
            s=b.snapshot();s['q'][5]+=np.deg2rad(4.);tick(k,n*.01,s)
        self.assertFalse(k.finished);self.assertEqual(b.disables,0)

    def test_disable_is_all_arm_motors_and_latches_motion_without_reset(self):
        h=Hardware.__new__(Hardware);h.tx_lock=threading.Lock();h.stopped=False;h.active=True
        frames=[];h._send_locked=lambda cid,data:frames.append((cid,bytes(data)))
        h.disable();self.assertEqual(frames,[(0x471,bytes([7,1,0,0,0,0,0,0]))])
        for fn in (h.command,h.arm):
            with self.assertRaises(RuntimeError):fn(np.zeros(6))
        h.stopped=True;h.disable() # Fault latch cannot block a subsequent supported disable.
        self.assertTrue(all(cid==0x471 for cid,_ in frames))

    def test_disable_physics_does_not_freeze_raised_arm(self):
        b=Simulation(config());b.arm(b.snapshot()['q']);q=b.snapshot()['q'].copy();b.disable();b.advance(.1)
        self.assertFalse(np.any(b.snapshot()['enabled']))
        self.assertGreater(np.max(np.abs(b.snapshot()['q']-q)),.005)
        np.testing.assert_array_equal(b.model.actuator_gainprm,0)


if __name__=='__main__':unittest.main()
