"""Open-ended low-stiffness drag trial.

The operator starts the recording with LB+Y and ends it with LB+X.  During the
trial the MIT position reference follows the measured joint positions, so the
PD spring does not pull the arm back; the gravity feedforward carries the load
and the small Kd only damps motion.  The end action ramps the feedforward out
and returns through the existing MIT return path.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np

from piper_dynamics.real_robot.trials.trajectory import Segment


class DragTrial:
    kind = "drag"

    def __init__(self, start, cfg):
        self.config = cfg
        self.active_joints = [1, 2, 3, 4, 5, 6]
        self.passive_joints = []
        # Only used for bookkeeping (velocity is zero and the trial is open ended).
        self.reference = Segment(np.asarray(start, float), np.asarray(start, float), 3600.0)
        self.plan = {
            "schema_version": 1,
            "trajectory_type": "drag",
            "mode": "low-stiffness gravity-compensated hold; position reference follows measured q",
            "controlled_joints": self.active_joints,
            "kp_protocol": list(cfg["mit_kp"]),
            "kd_protocol": list(cfg["mit_kd"]),
            "ff_scale": float(cfg["mit_ff_scale"]),
            "ff_ramp_s": float(cfg["mit_ff_ramp_s"]),
            "gravity_model": None,
            "start_policy": "LB+X prepares and ramps Kp/FF; LB+Y starts recording",
            "end_policy": "LB+X while TESTING ramps FF out and returns through MIT",
            "open_ended": True,
            "q_reference": "measured joint position each control cycle",
            "v_reference": "zero",
            "gripper_policy": "unchanged; no gripper commands",
            "time_policy": "operator controls the recording window with LB+Y and LB+X",
        }
        self.phase = "not_started"
        self.test_t = None

    def begin(self, controller, now):
        self.test_t = now
        self.phase = "testing"
        controller.event("DRAG：零力拖动开始记录；推完按 LB+X 结束并回位。")

    def tick(self, controller, now, snapshot, action):
        # Follow the measured pose: no fixed-position restoring force.
        controller.command(np.asarray(snapshot["q"], float))

    def metadata(self, snapshot, now):
        qd = np.asarray(snapshot.get("qd", np.zeros(6)), float)
        return {
            "protocol": "drag",
            "phase": self.phase,
            "pose_id": 1,
            "direction": "drag",
            "repeat": 1,
            "joint": 0,
            "speed_round": 0,
            "speed_bound_rad_s": 0.0,
            "capture_id": 0,
            "steady_candidate": bool(np.max(np.abs(qd)) < 0.02),
        }

    def save(self, path):
        path = Path(path)
        self.plan["source_sha256"] = {
            name: hashlib.sha256((Path(__file__).parent.parent / name).read_bytes()).hexdigest()
            for name in ("controller.py", "control/mit_mode.py", "trials/drag.py")
        }
        (path / "reference_plan.json").write_text(
            json.dumps(self.plan, ensure_ascii=False, indent=2) + "\n"
        )
