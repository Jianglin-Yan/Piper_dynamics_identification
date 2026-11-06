"""Staged entry from a narrowly tolerated folded park to a fixed test posture."""

import numpy as np

from piper_dynamics.real_robot.current_pose import validate_startup_pose

STANDING_DEG = [0.0, 45.0, -60.0, 0.0, 0.0, 0.0]


def standing_plan(cfg, home):
    validate_startup_pose(cfg, home)
    raw_lower = np.deg2rad(cfg["lower_deg"])
    raw_upper = np.deg2rad(cfg["upper_deg"])
    interior_lower = raw_lower + np.deg2rad(cfg["margin_deg"])
    interior_upper = raw_upper - np.deg2rad(cfg["margin_deg"])
    target = np.deg2rad(cfg.get("standing_start_deg", STANDING_DEG))
    target_lower = interior_lower.copy()
    target_upper = interior_upper.copy()
    if cfg.get("hold_start_wrist"):
        target = target.copy()
        target[5] = home[5]
        target_lower[5] = raw_lower[5]
        target_upper[5] = raw_upper[5]
    if cfg.get("passive_wrist", False):
        target = target.copy()
        target[5] = home[5]
        allowance = np.deg2rad(cfg.get("startup_limit_tolerance_deg", 10.0))
        target_lower[5] = raw_lower[5] - allowance
        target_upper[5] = raw_upper[5] + allowance
    if (
        target.shape != (6,)
        or not np.all(np.isfinite(target))
        or np.any(target < target_lower)
        or np.any(target > target_upper)
    ):
        raise ValueError("Standing target must be inside every joint safety interval")
    # A legal, nearby native MOVE J endpoint for crossing the parked zero boundary.
    boundary = np.maximum(raw_lower, np.minimum(raw_upper, home))
    # Only measured startup tolerance expands; commanded entry remains at legal bounds.
    # Session corridor includes only the observed park, legal interior and small feedback allowance.
    lower = np.minimum(interior_lower, home - np.deg2rad(2.0))
    upper = np.maximum(interior_upper, home + np.deg2rad(2.0))
    return target, boundary, lower, upper
