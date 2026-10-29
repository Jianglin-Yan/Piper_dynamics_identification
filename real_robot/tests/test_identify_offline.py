import json
import struct
import tempfile
import unittest
from pathlib import Path

import numpy as np

from real_robot.identify_offline import (
    friction_matrix, inertial_model, interpolate_with_gaps,
    load_measurements, original_modules,
)


class TestOfflineIdentification(unittest.TestCase):
    def test_interpolation_does_not_call_a_large_gap_valid(self):
        grid=np.array([-.1,.005,.05,.095,.11])
        values,valid=interpolate_with_gaps([0,.01,.09,.1],[0,1,9,10],grid)
        np.testing.assert_array_equal(valid,[False,True,False,True,False])
        self.assertAlmostEqual(values[2],5.)

    def test_friction_columns_belong_to_only_one_axis(self):
        v=np.arange(24).reshape(4,6)/100
        F=friction_matrix(v)
        theta=np.zeros(18);theta[3]=2;theta[4]=3;theta[5]=4
        pred=np.einsum('nij,j->ni',F,theta)
        np.testing.assert_allclose(pred[:,1],2*v[:,1]+3*np.tanh(v[:,1]/.02)+4)
        np.testing.assert_allclose(pred[:,[0,2,3,4,5]],0)

    def test_x_regressor_matches_inverse_dynamics_and_original_ls_predicts(self):
        import mujoco
        rg,identification,_=original_modules()
        nominal=inertial_model();theta=rg.ground_truth_theta(nominal).reshape(-1)
        builder=rg.RegressorBuilder(inertial_model(),link_bodies=tuple((f'link{i}',) for i in range(1,7)))
        rng=np.random.default_rng(927)
        q=rng.uniform(-.5,.5,(60,6));qd=rng.uniform(-.3,.3,(60,6));qdd=rng.uniform(-1,1,(60,6))
        Y=builder.build(q,qd,qdd)
        data=mujoco.MjData(nominal);tau=[]
        for a,b,c in zip(q,qd,qdd):
            data.qpos[:]=a;data.qvel[:]=b;data.qacc[:]=c
            mujoco.mj_inverse(nominal,data);tau.append(data.qfrc_inverse.copy())
        tau=np.asarray(tau)
        np.testing.assert_allclose(np.einsum('nij,j->ni',Y,theta),tau,atol=1e-9)
        solution=identification.solve_parameters(Y[:40].reshape(-1,60),tau[:40].reshape(-1),ridge=0)
        predicted=np.einsum('nij,j->ni',Y[40:],solution['theta'])
        np.testing.assert_allclose(predicted,tau[40:],atol=1e-8)

    def synthetic_run(self,root):
        (root/'test_manifest.json').write_text(json.dumps({'mode':'hardware','windows':[
            {'start_monotonic_s':100.,'end_monotonic_s':101.}]}))
        frames=[]
        for n in range(200):
            t=n*.005;angle=round(t*t*180/np.pi*1000)
            for pair in range(3):
                frames.append(dict(t_monotonic_s=100+t,can_id=hex(0x2a5+pair),
                    data_hex=struct.pack('>ii',angle,angle).hex()))
            for j in range(6):
                frames.append(dict(t_monotonic_s=100+t,can_id=hex(0x251+j),
                    data_hex=struct.pack('>hhi',round(2*t*1000),100,0).hex()))
        (root/'test_raw_can.jsonl').write_text(''.join(json.dumps(f)+'\n' for f in frames))
        t=np.arange(.01,1,.01);valid=np.ones(len(t),bool);valid[49]=False
        np.savez(root/'test_signals.npz',t_test_s=t,valid_measurement=valid,
                 qdd_ref=np.ones((len(t),6))*777)

    def test_acceleration_comes_from_measured_velocity_and_masks_stale_support(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);self.synthetic_run(root)
            d,_=load_measurements(root,window=11)
            np.testing.assert_allclose(d['qdd'][d['valid']],2,atol=1e-9)
            self.assertFalse(d['valid'][(d['t']>.46)&(d['t']<.54)].any())
            self.assertTrue(d['valid'][(d['t']>.2)&(d['t']<.3)].all())

    def test_out_of_window_frame_refused(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);self.synthetic_run(root)
            with (root/'test_raw_can.jsonl').open('a') as f:
                f.write(json.dumps(dict(t_monotonic_s=101.,can_id='0x251',data_hex='00'*8))+'\n')
            with self.assertRaisesRegex(ValueError,'out-of-window'):
                load_measurements(root,window=11)

    def test_simulated_samples_refused_as_hardware_identification(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            (root/'test_manifest.json').write_text(json.dumps({'mode':'sim','windows':[{}]}))
            with self.assertRaisesRegex(ValueError,'real-hardware'):
                load_measurements(root)


if __name__=='__main__':unittest.main()
