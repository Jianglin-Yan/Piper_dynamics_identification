"""Adapt the repository's original swevers_trajectory to the hardware test session.

The original six harmonic signals are retained, translated to the prepared pose,
and multiplied by a C2 start/stop envelope. No silent frequency or time scaling.
"""
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import BPoly
from excitation import swevers_trajectory
from .trajectory import smooth


class RepositoryMultisine:
    supports_batch=True
    def __init__(self,anchor,cfg):
        self.anchor=np.asarray(anchor,float);self.duration=cfg['duration_s'];self.ramp=cfg['ramp_s']
        lo=np.deg2rad(cfg['multisine_generation_lower_deg']);hi=np.deg2rad(cfg['multisine_generation_upper_deg'])
        raw=swevers_trajectory(lo,hi,duration=self.duration,dt=cfg['multisine_sample_dt_s'],
            base_freq=cfg['multisine_base_frequency_hz'],harmonics=cfg['multisine_harmonics'],
            amplitude_frac=cfg['multisine_amplitude_fraction'],seed=cfg['multisine_seed'])
        if 'multisine_coefficients' in cfg:
            from .designed_trajectory import fourier
            ab=np.asarray(cfg['multisine_coefficients'],float)
            if ab.shape!=(2,6,cfg['multisine_harmonics']) or not np.all(np.isfinite(ab)):
                raise ValueError('Invalid frozen Fourier coefficients')
            delta,v,a,_=fourier(raw.t,ab,raw.base_freq)
            raw.q=raw.q0+delta;raw.qd=v;raw.qdd=a
        self.raw=raw;self.braking_amax=cfg['braking_max_acceleration_rad_s2']
        # Quintic Hermite interpolation preserves source position/velocity/acceleration
        # at every original 500 Hz knot; BPoly control points are built in one batch.
        h=np.diff(raw.t)[:,None];q0,q1=raw.q[:-1],raw.q[1:];v0,v1=raw.qd[:-1],raw.qd[1:]
        a0,a1=raw.qdd[:-1],raw.qdd[1:]
        coefficients=np.stack([q0,q0+h*v0/5,q0+2*h*v0/5+h*h*a0/20,
                               q1-2*h*v1/5+h*h*a1/20,q1-h*v1/5,q1])
        self.spline=BPoly(coefficients,raw.t,extrapolate=False)
        self.d1=self.spline.derivative();self.d2=self.spline.derivative(2)
        source=Path(__file__).resolve().parents[1]/'excitation.py'
        self.metadata={'source':'excitation.py:swevers_trajectory','source_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),
            'base_frequency_hz':raw.base_freq,'harmonics':raw.harmonics,'amplitude_fraction':cfg['multisine_amplitude_fraction'],
            'seed':cfg['multisine_seed'],'source_sample_dt_s':cfg['multisine_sample_dt_s'],
            'generation_lower_deg':cfg['multisine_generation_lower_deg'],'generation_upper_deg':cfg['multisine_generation_upper_deg'],
            'source_center_rad':raw.q0.tolist(),'execution_anchor_rad':self.anchor.tolist(),
            'adaptation':'q_anchor + scale * C2_envelope * (q_source - q_source_center)',
            'duration_s':self.duration,'ramp_s':self.ramp,'time_scale':1.}
        if 'trajectory_candidate' in cfg:
            self.metadata['legacy_formula_source_sha256']=self.metadata.pop('source_sha256')
            self.metadata['source']='designed_trajectory.py:fourier / frozen candidate coefficients'
            self.metadata['source_sha256']=hashlib.sha256((Path(__file__).parent/'designed_trajectory.py').read_bytes()).hexdigest()
            self.metadata['seed']=None
            self.metadata['candidate']=cfg['trajectory_candidate']
            self.metadata['coefficients_source_rad']=cfg['multisine_coefficients']

    def at(self,t):
        t=np.clip(np.asarray(t,float),0.,self.duration);r=self.ramp
        up,du,ddu=smooth(t/r);down,dd,ddd=smooth((self.duration-t)/r)
        du=du/r;ddu=ddu/r**2;dd=-dd/r;ddd=ddd/r**2
        e=up*down;de=du*down+up*dd;dde=ddu*down+2*du*dd+up*ddd
        delta=self.spline(t)-self.raw.q0;v=self.d1(t);a=self.d2(t)
        return (self.anchor+e[...,None]*delta,
                de[...,None]*delta+e[...,None]*v,
                dde[...,None]*delta+2*de[...,None]*v+e[...,None]*a)


def reference_report(path,nominal,lower,upper,vmax,amax):
    t=np.linspace(0.,path.duration,max(1001,int(path.duration/.002)+1))
    q,v,a=path.at(t)
    report={**nominal.metadata,'scale':path.scale,
        'speed_scale':path.speed_scale,'time_scale':1/path.speed_scale,
        'source_duration_s':nominal.duration,'duration_s':path.duration,
        'source_ramp_s':nominal.ramp,'ramp_s':nominal.ramp/path.speed_scale,
        'source_base_frequency_hz':nominal.raw.base_freq,'base_frequency_hz':nominal.raw.base_freq*path.speed_scale,
        'moving_joints':[1,2,3,4,5,6],
        'q_min_deg':np.rad2deg(q.min(0)).tolist(),'q_max_deg':np.rad2deg(q.max(0)).tolist(),
        'max_velocity_per_joint_rad_s':np.abs(v).max(0).tolist(),
        'max_acceleration_per_joint_rad_s2':np.abs(a).max(0).tolist(),
        'test_velocity_limit_rad_s':vmax,'test_acceleration_limit_rad_s2':amax,
        'reference_lower_deg':np.rad2deg(lower).tolist(),'reference_upper_deg':np.rad2deg(upper).tolist()}
    # brake() uses this duration and a monotonic stop displacement q+v*T/2.
    stop_time=np.maximum(.5,2*np.abs(v).max(axis=1)/nominal.braking_amax)
    stop_q=q+v*stop_time[:,None]/2
    report['sampled_brake_endpoints_within_bounds']=bool(np.all(stop_q>=lower) and np.all(stop_q<=upper))
    report['braking_check_note']='Dense sampled reference and monotonic stop endpoints; no collision or tracking-error certificate.'
    h=hashlib.sha256()
    h.update(json.dumps(nominal.metadata,sort_keys=True).encode())
    for x in (t,q,v,a):h.update(np.asarray(x,dtype='<f8').tobytes())
    report['reference_sha256']=h.hexdigest()
    report['constraint_violations']=[]
    if np.any(q<lower-1e-9) or np.any(q>upper+1e-9):report['constraint_violations'].append('joint position')
    if np.abs(v).max()>vmax+1e-8:report['constraint_violations'].append('reference velocity')
    if np.abs(a).max()>amax+1e-8:report['constraint_violations'].append('reference acceleration')
    if not report['sampled_brake_endpoints_within_bounds']:report['constraint_violations'].append('braking corridor')
    report['hard_violations']=[x for x in report['constraint_violations'] if x in ('joint position','braking corridor')]
    report['warnings']=[x for x in report['constraint_violations'] if x not in report['hard_violations']]
    report['reference_limit_policy']='warn; not an emergency-stop criterion'
    return report


def save_reference(path,trajectory,report):
    (path/'reference_plan.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    t=np.arange(0.,trajectory.duration+.0001,.01);q,v,a=trajectory.at(t)
    np.savez_compressed(path/'reference_trajectory.npz',t=t,q_ref=q,qd_ref=v,qdd_ref=a)

class TrajectoryRejected(ValueError):
    def __init__(self,report):
        self.reference_report=report
        super().__init__('Multisine reference rejected: '+', '.join(report['hard_violations'])+'; see reference_plan.json')
