import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from piper_dynamics.real_robot.trials.designed_trajectory import coefficients, fourier, motion, load_candidate
from piper_dynamics.real_robot.trials.multisine import RepositoryMultisine
from piper_dynamics.real_robot.trials.trajectory import ScaledTrajectory
from tests.real_robot.test_multisine import multisine_config
from piper_dynamics.real_robot.standing import STANDING_DEG


class TestDesignedTrajectory(unittest.TestCase):
    def test_analytic_source_matches_existing_random_generator(self):
        c=multisine_config(); anchor=np.deg2rad(STANDING_DEG)
        p=RepositoryMultisine(anchor,c)
        ab=coefficients(c['multisine_seed'],np.deg2rad(c['multisine_generation_lower_deg']),
                        np.deg2rad(c['multisine_generation_upper_deg']))
        t=p.raw.t[::73];x,v,a,_=fourier(t,ab)
        np.testing.assert_allclose(x+p.raw.q0,p.raw.q[::73],atol=1e-14)
        np.testing.assert_allclose(v,p.raw.qd[::73],atol=1e-13)
        np.testing.assert_allclose(a,p.raw.qdd[::73],atol=1e-12)

    def test_candidate_runtime_interpolation_matches_optimizer_motion(self):
        c=multisine_config();anchor=np.deg2rad(STANDING_DEG)
        rng=np.random.default_rng(12);ab=rng.normal(0,.08,(2,6,5))
        c['multisine_coefficients']=ab.tolist()
        p=ScaledTrajectory(RepositoryMultisine(anchor,c),anchor,.65,.4)
        t=np.array([0.,.423,3.517,12.501,25.913,80.515,145.713,150.])
        expected=motion(t,ab,anchor,scale=.65,speed=.4)
        for actual,desired in zip(p.at(t),expected[:3]):
            np.testing.assert_allclose(actual,desired,atol=1e-8)

    def test_analytic_jerk_and_rest_endpoints_with_independent_scales(self):
        rng=np.random.default_rng(2);ab=rng.normal(0,.1,(2,6,5));anchor=np.zeros(6)
        for scale,speed in [( .35,.25),(.8,.65)]:
            for t in [0.,60/speed]:
                q,v,a,_=motion(t,ab,anchor,scale,speed)
                np.testing.assert_allclose(q,anchor,atol=1e-12)
                np.testing.assert_allclose(v,0,atol=1e-12)
                np.testing.assert_allclose(a,0,atol=1e-12)
            for t in [.741,12.431,60/speed-1.123]:
                h=1e-4;q,v,a,j=motion(t,ab,anchor,scale,speed)
                qp,vp,ap,_=motion(t+h,ab,anchor,scale,speed)
                qm,vm,am,_=motion(t-h,ab,anchor,scale,speed)
                np.testing.assert_allclose((qp-qm)/(2*h),v,atol=1e-7)
                np.testing.assert_allclose((vp-vm)/(2*h),a,atol=1e-7)
                np.testing.assert_allclose((ap-am)/(2*h),j,atol=1e-6)

    def test_loader_rejects_unreviewed_scale_and_anchor_before_hardware(self):
        c=multisine_config();c.update(trajectory_scale=.65,trajectory_speed_scale=.4)
        data=dict(schema_version=1,execution=dict(scale=.65,speed_scale=.4,anchor_deg=STANDING_DEG,
                  harmonics=5,source_duration_s=60.,source_ramp_s=5.,source_frequency_hz=.2),
                  model_sha256={},candidates={'test':dict(coefficients_source_rad=np.zeros((2,6,5)).tolist(),status='test')})
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'bank.json';path.write_text(json.dumps(data))
            load_candidate(c,'test',path)
            c['trajectory_speed_scale']=.5
            with self.assertRaisesRegex(ValueError,'screened'):load_candidate(c,'test',path)
            c['trajectory_speed_scale']=.4;c['start_deg']=[0,50,-60,0,0,0]
            with self.assertRaisesRegex(ValueError,'anchor'):load_candidate(c,'test',path)


if __name__=='__main__':unittest.main()
