"""Narrow current-pose policy for the measured folded PiPER X J6 pilot.

No robot parameter changes. Only J2-low / J3-high parked feedback excursions
up to 3 degrees can be anchored; their commanded targets stay unchanged.
"""
import numpy as np

MAX_PARK_OVERSHOOT_DEG = 3.0
HOLD_TOLERANCE_DEG = .5
ARM_POSE_TOLERANCE_DEG = .1


def current_pose_bounds(cfg, home):
    if cfg['joint'] != 6 or cfg['start_deg'] is not None:
        raise ValueError('--from-current is restricted to J6 with start_deg=null')
    if cfg['amplitude_deg'] > 5 or cfg['frequency_hz'] > .1:
        raise ValueError('--from-current only permits amplitude <=5 deg and frequency <=0.1 Hz')
    raw_lower = np.deg2rad(cfg['lower_deg'])
    raw_upper = np.deg2rad(cfg['upper_deg'])
    overshoot = np.maximum(raw_lower-home, home-raw_upper)
    allowance = np.deg2rad(MAX_PARK_OVERSHOOT_DEG)
    for j in range(6):
        if overshoot[j] <= 1e-9:
            continue
        allowed = (j == 1 and home[j] < raw_lower[j]) or (j == 2 and home[j] > raw_upper[j])
        if not allowed or overshoot[j] > allowance+1e-9:
            raise ValueError(f'J{j+1} parked excursion {np.rad2deg(overshoot[j]):.3f} deg is outside the reviewed current-pose policy')
    # Held axes have a measurement drift corridor; commands must stay at home exactly.
    lower = home-np.deg2rad(HOLD_TOLERANCE_DEG)
    upper = home+np.deg2rad(HOLD_TOLERANCE_DEG)
    lower[5] = raw_lower[5]+np.deg2rad(cfg['margin_deg'])
    upper[5] = raw_upper[5]-np.deg2rad(cfg['margin_deg'])
    if not lower[5] <= home[5] <= upper[5]:
        raise ValueError('J6 startup angle is outside its normal safety interior')
    return lower, upper
