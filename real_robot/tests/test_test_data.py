import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np
from real_robot.trajectory import Excitation,ScaledTrajectory
from real_robot.test_data import TestRecorder,export_test_data
from real_robot.run import validate_config
from real_robot.tests.test_pilot import config


class TestScaling(unittest.TestCase):
    def test_anchor_derivatives_and_duration(self):
        anchor=np.array([.1,.5,-1.,.2,.3,.4]);p=Excitation(anchor,5,.08,.1,20,5)
        for scale in (.25,.5,1.):
            scaled=ScaledTrajectory(p,anchor,scale);self.assertEqual(scaled.duration,p.duration)
            for t in (0.,1.2,7.,17.8,20.):
                q,v,a=p.at(t);qs,vs,aas=scaled.at(t)
                np.testing.assert_allclose(qs-anchor,scale*(q-anchor),atol=1e-15)
                np.testing.assert_allclose(vs,scale*v);np.testing.assert_allclose(aas,scale*a)
                np.testing.assert_allclose(qs[:5],anchor[:5])

    def test_invalid_scale_rejected_before_devices(self):
        for scale in (0.,-.1,1.01,np.nan,np.inf):
            c=config();c['trajectory_scale']=scale
            with self.assertRaises(ValueError):validate_config(c)


class TestDataset(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.path=Path(self.tmp.name);self.c=config();self.c['trajectory_scale']=.25
        self.ctrl=SimpleNamespace(state='READY',path_t=0.,test=ScaledTrajectory(Excitation(np.zeros(6),5,.08,.1,20,5),np.zeros(6),.25),start=np.zeros(6),last_target=np.zeros(6),stop_reason=None)
        self.r=TestRecorder(self.path,self.c,'sim',version={'commit':'test','dirty':False})
    def tearDown(self):
        if not self.r.closed:self.r.close(2.,'teardown')
        self.tmp.cleanup()
    def snapshot(self,t):
        return {'time':t,'q':np.zeros(6),'qd':np.full(6,2*t),'current_A':np.ones(6)*.1,
                'tau_candidate_Nm':np.ones(6)*.2,'q_times':np.full(6,t),'drive_times':np.full(6,t),
                'low_times':np.full(6,t),'enabled':np.ones(6,bool),'faults':[0]*6,
                'status':{'arm_status':0,'error_code':0},'temp_C':[25]*6,'voltage_V':[24]*6}
    def step(self,t):self.r.update(self.ctrl,t,self.snapshot(t),t+.001)
    def begin(self):self.ctrl.state='TESTING';self.step(0.)
    def test_only_testing_samples_not_transition_or_braking(self):
        self.step(-1.);self.begin()
        for n in range(1,21):self.step(n*.01)
        self.ctrl.state='BRAKING';self.ctrl.stop_reason='LB+RB';self.step(.21)
        self.ctrl.state='RETURNING';self.step(.3);self.r.close(.4,'done')
        rows=[json.loads(x) for x in (self.path/'test_samples.jsonl').read_text().splitlines()]
        self.assertEqual(len(rows),20);self.assertTrue(all(x['state']=='TESTING' for x in rows))
        self.assertEqual(rows[0]['t_test_s'],.01)
        manifest=json.loads((self.path/'test_manifest.json').read_text())
        self.assertEqual(manifest['windows'][0]['end_monotonic_s'],.21)
        self.assertEqual(manifest['windows'][0]['end_reason'],'LB+RB')
        self.assertFalse(manifest['suitable_for_identification'])
    def test_raw_frames_are_selected_by_half_open_test_window(self):
        self.begin();self.step(.01);self.ctrl.state='RETURNING';self.step(.2);self.r.close(.3,'done')
        frames=[{'t_monotonic_s':t,'can_id':'0x251','data_hex':'00'*8} for t in (-.1,0.,.15,.2,.3)]
        (self.path/'raw_can.jsonl').write_text(''.join(json.dumps(x)+'\n' for x in frames))
        summary=export_test_data(self.path)
        self.assertEqual(summary['raw_test_frame_count'],2)
        selected=[json.loads(x)['t_monotonic_s'] for x in (self.path/'test_raw_can.jsonl').read_text().splitlines()]
        self.assertEqual(selected,[0.,.15])
    def test_acceleration_estimate_uses_measured_velocity_not_reference(self):
        self.begin()
        for n in range(1,61):self.step(n*.01)
        self.ctrl.state='RETURNING';self.step(.61);self.r.close(.7,'done');export_test_data(self.path)
        with np.load(self.path/'test_signals.npz') as d:
            self.assertTrue(d['qdd_est_valid'].any())
            np.testing.assert_allclose(d['qdd_est'][d['qdd_est_valid']],2.,atol=1e-10)
            self.assertGreater(np.max(np.abs(d['qdd_ref'][:,5]-2)),1.)
            self.assertTrue(np.all(~d['qdd_est_valid'][:5]))
    def test_gap_cannot_acquire_valid_acceleration(self):
        self.begin()
        for n in list(range(1,20))+list(range(40,70)):self.step(n*.01)
        self.ctrl.state='RETURNING';self.step(.7);self.r.close(.8,'done');export_test_data(self.path)
        with np.load(self.path/'test_signals.npz') as d:
            around_gap=(d['t_uniform_monotonic_s']>.2)&(d['t_uniform_monotonic_s']<.39)
            self.assertFalse(d['qdd_est_valid'][around_gap].any())
            self.assertTrue(np.isnan(d['qdd_est'][around_gap]).all())
    def test_stale_data_retained_but_marked_invalid(self):
        self.begin();s=self.snapshot(.2);s['drive_times'][:]=.01
        self.r.update(self.ctrl,.2,s,.201);self.r.close(.3,'aborted')
        row=json.loads((self.path/'test_samples.jsonl').read_text())
        self.assertFalse(row['quality']['valid_measurement']);self.assertEqual(row['feedback']['drive_times'],[.01]*6)
    def test_zero_samples_has_explicit_empty_dataset(self):
        self.r.close(1.,'not_started');summary=export_test_data(self.path)
        self.assertEqual(summary['sample_count'],0)
        with np.load(self.path/'test_signals.npz') as d:self.assertEqual(d['q'].shape,(0,6))


if __name__=='__main__':unittest.main()
