import json
from pathlib import Path
import unittest
import numpy as np
from real_robot.controller import Controller
from real_robot.multisine import RepositoryMultisine,reference_report
from real_robot.standing import STANDING_DEG
from real_robot.trajectory import ScaledTrajectory
from real_robot.tests.test_pilot import config,Fake,tick


def multisine_config():
    c=config();c.update(json.loads((Path(__file__).parents[1]/'multisine_profile.json').read_text()))
    c.update(standing_start=True,current_pose_only=False,start_deg=STANDING_DEG,
             simulation_initial_deg=[2.614,-2.618,2.798,5.159,19.415,7.411])
    return c


class TestMultisine(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg=multisine_config();cls.anchor=np.deg2rad(STANDING_DEG)
        cls.source=RepositoryMultisine(cls.anchor,cls.cfg)

    def test_preserves_original_source_knots_and_derivatives_away_from_ramps(self):
        indices=np.arange(3000,27000,73);t=self.source.raw.t[indices];q,v,a=self.source.at(t)
        np.testing.assert_allclose(q,self.anchor+self.source.raw.q[indices]-self.source.raw.q0,atol=1e-12)
        np.testing.assert_allclose(v,self.source.raw.qd[indices],atol=1e-10)
        np.testing.assert_allclose(a,self.source.raw.qdd[indices],atol=1e-8)

    def test_c2_rest_endpoints_and_analytic_derivatives(self):
        for t in (0.,60.):
            q,v,a=self.source.at(t);np.testing.assert_allclose(q,self.anchor,atol=1e-12)
            np.testing.assert_allclose(v,0,atol=1e-12);np.testing.assert_allclose(a,0,atol=1e-10)
        for t in (1.234,4.123,10.011,57.321):
            h=1e-5;q,v,a=self.source.at(t);qp,vp,_=self.source.at(t+h);qm,vm,_=self.source.at(t-h)
            np.testing.assert_allclose((qp-qm)/(2*h),v,atol=1e-7)
            np.testing.assert_allclose((vp-vm)/(2*h),a,atol=1e-6)

    def test_scale_keeps_time_frequency_seed_and_moves_all_six_axes(self):
        p=ScaledTrajectory(self.source,self.anchor,.5);small=ScaledTrajectory(self.source,self.anchor,.25)
        t=np.arange(0.,60.,.01);q,v,a=p.at(t);qs,vs,aas=small.at(t)
        np.testing.assert_allclose(q-self.anchor,2*(qs-self.anchor),atol=1e-14)
        np.testing.assert_allclose(v,2*vs);np.testing.assert_allclose(a,2*aas)
        self.assertTrue(np.all(np.ptp(q,axis=0)>.1));self.assertEqual(p.duration,60.)
        self.assertEqual(self.source.metadata['base_frequency_hz'],.2)
        self.assertEqual(self.source.metadata['harmonics'],5)

    def test_half_speed_preserves_path_and_scales_entire_envelope(self):
        slow=ScaledTrajectory(self.source,self.anchor,.5,.5)
        fast=ScaledTrajectory(self.source,self.anchor,.5)
        t=np.arange(0.,120.01,.01);q,v,a=slow.at(t);q0,v0,a0=fast.at(t*.5)
        np.testing.assert_allclose(q,q0);np.testing.assert_allclose(v,v0*.5)
        np.testing.assert_allclose(a,a0*.25)
        self.assertEqual(slow.duration,120.)

    def test_half_speed_report_records_actual_and_source_frequencies(self):
        slow=ScaledTrajectory(self.source,self.anchor,.5,.5)
        low=np.deg2rad(self.cfg['lower_deg'])+np.deg2rad(self.cfg['margin_deg'])
        high=np.deg2rad(self.cfg['upper_deg'])-np.deg2rad(self.cfg['margin_deg'])
        report=reference_report(slow,self.source,low,high,3.,20.)
        self.assertEqual(report['speed_scale'],.5);self.assertEqual(report['time_scale'],2.)
        self.assertEqual(report['duration_s'],120.);self.assertEqual(report['source_duration_s'],60.)
        self.assertEqual(report['ramp_s'],10.);self.assertEqual(report['base_frequency_hz'],.1)
        self.assertEqual(report['source_base_frequency_hz'],.2)
        # Slower playback samples the source phase more densely; numerical peaks
        # need not equal the earlier, coarser report to six decimal places.
        self.assertAlmostEqual(max(report['max_velocity_per_joint_rad_s']),.9701,places=3)
        self.assertAlmostEqual(max(report['max_acceleration_per_joint_rad_s2']),2.3200,places=3)

    def make(self):
        c=multisine_config();b=Fake(c);events=[];k=Controller(b,c,b.snapshot(),0,events.append)
        b.enabled=True;b.active=True;b.q=k.start.copy();b.v[:]=0
        k.active=True;k.armed=True;k.last_target=k.start.copy();k.set_path('TESTING',k.test,0)
        return c,b,k,events

    def test_reference_speed_acceleration_exceedance_is_only_warning(self):
        c=multisine_config();c.update(test_max_velocity_rad_s=.01,test_max_acceleration_rad_s2=.01)
        b=Fake(c);k=Controller(b,c,b.snapshot(),0,lambda _:None)
        self.assertEqual(k.state,'WAITING');self.assertTrue(k.reference_plan['warnings'])
        self.assertEqual(k.reference_plan['hard_violations'],[])

    def test_above_reference_envelope_and_sustained_stop_threshold_do_not_abort(self):
        c,b,k,e=self.make()
        for n in range(30):
            s=b.snapshot();s['q']=k.last_target.copy();s['qd'][0]=2.6;tick(k,n*.01,s)
        self.assertEqual(k.state,'TESTING');self.assertEqual(b.stops,[])
        self.assertTrue(k.envelope_exceeded)

    def test_tracking_error_above_old_pilot_limit_does_not_abort(self):
        c,b,k,e=self.make();s=b.snapshot();s['q'][5]+=np.deg2rad(6.)
        tick(k,.01,s);self.assertEqual(k.state,'TESTING');self.assertEqual(b.stops,[])

    def test_independent_absolute_speed_and_tracking_lines_still_fault(self):
        # The absolute speed line is a sustained-feedback line; one sample is not a runaway.
        c,b,k,e=self.make()
        for n in range(12):
            s=b.snapshot();s['q']=k.last_target.copy();s['qd'][0]=3.1;tick(k,n*.01,s)
        self.assertEqual(k.state,'FAULT');self.assertEqual(len(b.stops),1)
        c,b,k,e=self.make()
        for n in range(25):
            s=b.snapshot();s['q'][5]+=np.deg2rad(26.);tick(k,n*.01,s)
        self.assertEqual(k.state,'BRAKING');self.assertEqual(len(b.stops),0)

    def test_real_feedback_failure_not_downgraded_to_warning(self):
        c,b,k,e=self.make();b.bad=True;tick(k,.01,b.snapshot())
        self.assertEqual(k.state,'FAULT');self.assertEqual(len(b.stops),1)


if __name__=='__main__':unittest.main()
