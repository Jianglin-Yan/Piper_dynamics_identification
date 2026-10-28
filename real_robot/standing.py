"""Staged entry from a narrowly tolerated folded park to a fixed test posture."""
import numpy as np
from .current_pose import current_pose_bounds

STANDING_DEG = [0.,45.,-60.,0.,0.,0.]

def standing_plan(cfg,home):
    profile=dict(cfg);profile.update(joint=6,start_deg=None)
    current_pose_bounds(profile,home) # Same <=3 degree parked exception; no broad limit override.
    raw_lower=np.deg2rad(cfg['lower_deg']);raw_upper=np.deg2rad(cfg['upper_deg'])
    interior_lower=raw_lower+np.deg2rad(cfg['margin_deg'])
    interior_upper=raw_upper-np.deg2rad(cfg['margin_deg'])
    target=np.deg2rad(cfg.get('standing_start_deg',STANDING_DEG))
    if target.shape!=(6,) or not np.all(np.isfinite(target)) or np.any(target<interior_lower) or np.any(target>interior_upper):
        raise ValueError('Standing target must be inside every joint safety interval')
    # A legal, nearby native MOVE J endpoint for crossing the parked zero boundary.
    boundary=np.maximum(raw_lower,np.minimum(raw_upper,home))
    if np.max(np.abs(boundary-home))>np.deg2rad(3.)+1e-9:raise ValueError('Boundary entry exceeds 3 degrees')
    # Session corridor includes only the observed park, legal interior and small feedback allowance.
    lower=np.minimum(interior_lower,home-np.deg2rad(.5))
    upper=np.maximum(interior_upper,home+np.deg2rad(.5))
    return target,boundary,lower,upper
