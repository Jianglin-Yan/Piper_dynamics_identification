import copy, json, unittest
from pathlib import Path
import numpy as np
from real_robot.controller import Controller
from real_robot.hardware import Hardware
from real_robot.current_pose import current_pose_bounds
from test_pilot import Fake, config

PARK=[1.175,-2.723,2.289,4.434,19.527,12.418]

class TestCurrentPose(unittest.TestCase):
    def make(self):
        c=config();c['current_pose_only']=True;c['simulation_initial_deg']=PARK.copy();b=Fake(c)
        k=Controller(b,c,b.snapshot(),0,lambda x:None)
        return c,b,k
    def test_observed_parked_angles_are_accepted_without_motion(self):
        c,b,k=self.make();self.assertEqual(k.state,'WAITING');self.assertEqual(b.commands,[]);self.assertFalse(b.active)
        np.testing.assert_allclose(k.home,np.deg2rad(PARK))
    def test_j2_j3_excursions_have_strict_upper_bound(self):
        for j,value in ((1,-3.01),(2,3.01),(1,180.1),(2,-170.1),(0,150.1)):
            c=config();c['current_pose_only']=True;q=np.deg2rad(PARK);q[j]=np.deg2rad(value)
            with self.assertRaises(ValueError):current_pose_bounds(c,q)
    def test_mode_does_not_allow_another_joint_or_start_pose(self):
        for key,value in (('joint',2),('start_deg',PARK),('frequency_hz',.2)):
            c=config();c['current_pose_only']=True;c[key]=value
            with self.assertRaises(ValueError):current_pose_bounds(c,np.deg2rad(PARK))
    def test_j6_excitation_still_uses_normal_limits(self):
        c=config();c['current_pose_only']=True;c['simulation_initial_deg']=PARK.copy();c['simulation_initial_deg'][5]=114
        b=Fake(c)
        with self.assertRaises(ValueError):Controller(b,c,b.snapshot(),0,lambda x:None)
    def test_normal_trial_holds_all_other_targets_exactly(self):
        c,b,k=self.make();home=b.q.copy()
        for n in range(4000):
            k.tick(n*.01,b.snapshot(),{1:'PREPARE',3:'START'}.get(n))
            if k.state in ('DONE','FAULT'):break
        self.assertEqual(k.state,'DONE')
        commands=np.array(b.commands);np.testing.assert_allclose(commands[:,:5],np.tile(home[:5],(len(commands),1)),atol=1e-12)
        np.testing.assert_allclose(b.q,home,atol=1e-10)
    def test_held_joint_drift_faults_without_return(self):
        c,b,k=self.make();k.tick(.01,b.snapshot(),'PREPARE');k.tick(.03,b.snapshot(),'START');k.tick(.04,b.snapshot())
        b.q[1]+=np.deg2rad(.6);before=len(b.commands);k.tick(.05,b.snapshot())
        self.assertEqual(k.state,'FAULT');self.assertEqual(len(b.commands),before);self.assertEqual(len(b.stops),1)
    def test_held_axis_target_cannot_be_changed(self):
        c,b,k=self.make();q=k.home.copy();q[1]+=.0001
        with self.assertRaises(ValueError):k.command(q)
        self.assertEqual(b.commands,[])
    def test_abort_does_not_promote_measured_drift_to_held_target(self):
        c,b,k=self.make();home=b.q.copy();triggered=False
        for n in range(2000):
            action={1:'PREPARE',3:'START'}.get(n)
            if n==500:
                b.q[1]+=np.deg2rad(.2);b.v[1]=.01;action='ABORT';triggered=True
            k.tick(n*.01,b.snapshot(),action)
            if n==500 or k.state in ('DONE','FAULT'):break
        self.assertTrue(triggered);self.assertEqual(k.state,'BRAKING')
        commands=np.array(b.commands);np.testing.assert_allclose(commands[:,:5],np.tile(home[:5],(len(commands),1)),atol=1e-12)
    def test_current_pose_abort_returns_home(self):
        c,b,k=self.make();home=b.q.copy()
        for n in range(2000):
            k.tick(n*.01,b.snapshot(),{1:'PREPARE',3:'START',500:'ABORT'}.get(n))
            if k.state in ('DONE','FAULT'):break
        self.assertEqual(k.state,'DONE');self.assertTrue(k.aborted);np.testing.assert_allclose(b.q,home,atol=1e-10)
    def test_explicit_mode_preserves_unconfirmed_metadata(self):
        c=config();c['model_confirmed']=True;c['workspace_checked']=False;c['collision_policy_reviewed']=False;c['current_pose_only']=True
        h=Hardware.__new__(Hardware);h.c=c
        info={'firmware':'S-V1.7-3250618','collision':{'collision_levels':[0]*6},'limits':{},'acceleration_limits':{}}
        for j in range(1,7):
            info['limits'][j]={'min_deg':c['lower_deg'][j-1],'max_deg':c['upper_deg'][j-1],'max_speed_raw':300}
            info['acceleration_limits'][j]={'max_acc_raw':500}
        h.preflight(info);self.assertFalse(c['workspace_checked']);self.assertFalse(c['collision_policy_reviewed'])
        c['current_pose_only']=False
        with self.assertRaises(ValueError):h.preflight(info)

if __name__=='__main__':unittest.main()
