"""Narrow current-pose policy for the measured folded PiPER J6 pilot.

No robot parameter changes. All six parked feedback excursions
up to the configured startup allowance can be anchored; their commanded targets stay unchanged.
"""

import numpy as np

MAX_PARK_OVERSHOOT_DEG = 10.0
HOLD_TOLERANCE_DEG = 2.0
ARM_POSE_TOLERANCE_DEG = 0.5


def validate_startup_pose(cfg, home):
    home = np.asarray(home, float)
    if home.shape != (6,) or not np.all(np.isfinite(home)):
        raise ValueError("Invalid startup joint feedback")
    raw_lower = np.deg2rad(cfg["lower_deg"])
    raw_upper = np.deg2rad(cfg["upper_deg"])
    overshoot = np.maximum(0.0, np.maximum(raw_lower - home, home - raw_upper))
    limit = cfg.get("startup_limit_tolerance_deg", MAX_PARK_OVERSHOOT_DEG)
    if not np.isfinite(limit) or not 0 < limit <= 15:
        raise ValueError("Invalid startup_limit_tolerance_deg")
    if np.any(overshoot > np.deg2rad(limit) + 1e-9):
        j = int(np.argmax(overshoot))
        raise ValueError(
            f"J{j + 1} parked excursion {np.rad2deg(overshoot[j]):.3f} deg exceeds configured startup allowance {limit:g} deg"
        )
    return overshoot


def current_pose_bounds(cfg, home):
    if cfg["joint"] != 6 or cfg["start_deg"] is not None:
        raise ValueError("--from-current is restricted to J6 with start_deg=null")
    if cfg["amplitude_deg"] > 5 or cfg["frequency_hz"] > 0.1:
        raise ValueError("--from-current only permits amplitude <=5 deg and frequency <=0.1 Hz")
    validate_startup_pose(cfg, home)
    raw_lower = np.deg2rad(cfg["lower_deg"])
    raw_upper = np.deg2rad(cfg["upper_deg"])
    # Held axes have a measurement drift corridor; commands must stay at home exactly.
    lower = home - np.deg2rad(HOLD_TOLERANCE_DEG)
    upper = home + np.deg2rad(HOLD_TOLERANCE_DEG)
    lower[5] = raw_lower[5] + np.deg2rad(cfg["margin_deg"])
    upper[5] = raw_upper[5] - np.deg2rad(cfg["margin_deg"])
    if not lower[5] <= home[5] <= upper[5]:
        raise ValueError("J6 startup angle is outside its normal safety interior")
    return lower, upper
