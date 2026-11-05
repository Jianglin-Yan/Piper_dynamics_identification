"""Frozen Fourier candidates and analytic derivatives. No robot/SDK imports."""
from piper_dynamics.paths import model_reference_path
import hashlib
import json
from pathlib import Path
import numpy as np


def coefficients(seed, lower, upper, amplitude_fraction=.6, harmonics=5):
    """Exactly reproduce excitation.swevers_trajectory's source coefficients."""
    rng = np.random.default_rng(seed)
    a = rng.uniform(-1, 1, (6, harmonics))
    b = rng.uniform(-1, 1, (6, harmonics))
    budget = amplitude_fraction * (np.asarray(upper)-lower)/2
    factor = budget/(np.abs(a).sum(1)+np.abs(b).sum(1))
    return np.stack([a*factor[:, None], b*factor[:, None]])


def fourier(t, ab, frequency=.2):
    """Displacement and first three source-time derivatives; angles in radians."""
    t = np.asarray(t, float)
    ab = np.asarray(ab, float)
    w = 2*np.pi*frequency*np.arange(1, ab.shape[-1]+1)
    phase = t[..., None]*w
    s, c = np.sin(phase), np.cos(phase)
    a, b = ab
    return tuple(np.einsum('...k,jk->...j', x, aa)+np.einsum('...k,jk->...j', y, bb)
                 for x, y, aa, bb in ((s, c, a, b), (c, -s, a*w, b*w),
                                      (-s, -c, a*w*w, b*w*w), (-c, s, a*w**3, b*w**3)))


def envelope(t, duration=60., ramp=5.):
    def edge(x):
        z = np.clip(x, 0, 1)
        active = (x >= 0) & (x <= 1)
        return (10*z**3-15*z**4+6*z**5,
                (30*z**2-60*z**3+30*z**4)*active,
                (60*z-180*z**2+120*z**3)*active,
                (60-360*z+360*z**2)*active)
    u = edge(np.asarray(t)/ramp)
    d = edge((duration-np.asarray(t))/ramp)
    u = [x/ramp**i for i, x in enumerate(u)]
    d = [x*(-1/ramp)**i for i, x in enumerate(d)]
    return (u[0]*d[0], u[1]*d[0]+u[0]*d[1],
            u[2]*d[0]+2*u[1]*d[1]+u[0]*d[2],
            u[3]*d[0]+3*u[2]*d[1]+3*u[1]*d[2]+u[0]*d[3])


def motion(t, ab, anchor, scale=.5, speed=.5, frequency=.2, duration=60., ramp=5.):
    source_t = np.clip(np.asarray(t)*speed, 0, duration)
    x, v, a, j = fourier(source_t, ab, frequency)
    e, de, dde, ddde = [x[..., None] for x in envelope(source_t, duration, ramp)]
    return (np.asarray(anchor)+scale*e*x,
            scale*speed*(de*x+e*v),
            scale*speed**2*(dde*x+2*de*v+e*a),
            scale*speed**3*(ddde*x+3*dde*v+3*de*a+e*j))


def motion_identity(ab, execution):
    fields=('scale','speed_scale','anchor_deg','source_duration_s','source_ramp_s',
            'source_frequency_hz','harmonics')
    payload=dict(execution={key:execution[key] for key in fields},coefficients_source_rad=np.asarray(ab).tolist())
    return hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(',',':')).encode()).hexdigest()


def load_candidate(cfg, name, bank_path):
    """Freeze the reviewed scale/pose/model before any CAN connection is opened."""
    bank_path = Path(bank_path).resolve()
    bank_bytes = bank_path.read_bytes()
    bank = json.loads(bank_bytes)
    if bank.get('schema_version') != 1:
        raise ValueError('Unknown trajectory bank schema')
    if name not in bank['candidates']:
        raise ValueError('Unknown candidate '+name+'; available: '+', '.join(bank['candidates']))
    candidate = bank['candidates'][name]
    p = {**bank['execution'], **candidate.get('execution',{})}
    for key, expected in [('trajectory_scale', p['scale']), ('trajectory_speed_scale', p['speed_scale'])]:
        if not np.isclose(cfg.get(key, 1.), expected, atol=1e-12, rtol=0):
            raise ValueError(f'{name} was screened at {key}={expected}; use that value or redesign/revalidate')
    if not np.allclose(cfg['start_deg'], p['anchor_deg'], atol=1e-9, rtol=0):
        raise ValueError('Candidate standing anchor differs from its screened anchor')
    for relative, expected in bank['model_sha256'].items():
        source = model_reference_path(relative)
        if hashlib.sha256(source.read_bytes()).hexdigest() != expected:
            raise ValueError('Candidate model changed; redesign/revalidate: '+relative)
    bank_model=bank.get('model')
    if bank_model is not None and bank_model!=cfg.get('model','piper'):
        raise ValueError('Trajectory bank model differs from confirmed robot; re-evaluate with the current model')
    ab = np.asarray(candidate['coefficients_source_rad'], float)
    if ab.shape != (2, 6, p['harmonics']) or not np.all(np.isfinite(ab)):
        raise ValueError('Malformed candidate Fourier coefficients')
    motion_sha=motion_identity(ab,p)
    if candidate.get('motion_sha256',motion_sha)!=motion_sha:
        raise ValueError('Candidate motion changed since screening; redesign/revalidate')
    cfg.update(multisine_coefficients=ab.tolist(), duration_s=p['source_duration_s'],
               ramp_s=p['source_ramp_s'], multisine_base_frequency_hz=p['source_frequency_hz'],
               multisine_harmonics=p['harmonics'],
               trajectory_candidate=dict(id=name, bank_sha256=hashlib.sha256(bank_bytes).hexdigest(),
                                         motion_sha256=motion_sha, bank_path=str(bank_path),
                                         design_status=candidate['status']))
    return candidate
