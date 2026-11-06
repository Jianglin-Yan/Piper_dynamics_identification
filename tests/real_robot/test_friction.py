import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from piper_dynamics.paths import LEGACY_ROBOT_ROOT
from piper_dynamics.real_robot.trials.friction import FrictionTrial, configure

BASE = LEGACY_ROBOT_ROOT


def load_config():
    return json.loads((BASE / "config.json").read_text())


def make(**overrides):
    cfg = load_config()
    configure(cfg)
    cfg.update(overrides)
    start = np.deg2rad([0.0, 45.0, -60.0, 0.0, 0.0, 0.0])
    lower = np.deg2rad(cfg["lower_deg"])
    upper = np.deg2rad(cfg["upper_deg"])
    return cfg, start, FrictionTrial(start, cfg, lower, upper)


class FrictionTrialTests(unittest.TestCase):
    def test_default_sequence_covers_six_joints_and_speeds(self):
        cfg, start, trial = make(
            friction_joints=[1, 6, 4, 5, 2, 3],
            friction_speeds_rad_s=[0.005, 0.01, 0.02, 0.05, 0.1, 0.2],
            friction_repeats=2,
        )
        self.assertEqual(trial.joints, [1, 6, 4, 5, 2, 3])
        self.assertGreater(trial.reference.duration, 60 * 60)
        labels = trial.reference.labels
        self.assertEqual(len(labels), 6 * 2 * 6 * 6)
        self.assertEqual({lab["joint"] for lab in labels}, {1, 2, 3, 4, 5, 6})
        self.assertEqual(
            {lab["direction"] for lab in labels}, {"hold", "forward", "reverse", "return"}
        )
        self.assertEqual({lab["repeat"] for lab in labels}, {1, 2})
        self.assertEqual(labels[0]["phase"], "hold_0")
        self.assertEqual(labels[1]["direction"], "forward")

    def test_start_and_end_are_the_standing_pose(self):
        cfg, start, trial = make(
            friction_joints=[6], friction_speeds_rad_s=[0.05, 0.1], friction_repeats=1
        )
        self.assertEqual(len(trial.reference.parts), 12)
        np.testing.assert_allclose(trial.reference.at(0.0)[0], start, atol=1e-9)
        np.testing.assert_allclose(
            trial.reference.at(trial.reference.duration)[0], start, atol=1e-9
        )

    def test_each_segment_respects_its_speed_and_limits(self):
        cfg, start, trial = make(
            friction_joints=[2], friction_speeds_rad_s=[0.01, 0.2], friction_repeats=1
        )
        for part, label in zip(trial.reference.parts, trial.reference.labels):
            peak = np.max(np.abs(part.v)) if hasattr(part, "v") else 0.0
            self.assertLessEqual(peak, label["speed"] + 1e-6)

    def test_metadata_tracks_joint_speed_direction(self):
        cfg, start, trial = make(
            friction_joints=[4], friction_speeds_rad_s=[0.02, 0.05], friction_repeats=1
        )
        trial.test_t = 0.0
        trial.last_index = 1  # forward at first speed
        meta = trial.metadata({}, 0.1)
        self.assertEqual(meta["protocol"], "friction")
        self.assertEqual(meta["joint"], 4)
        self.assertEqual(meta["direction"], "forward")
        self.assertEqual(meta["speed_round"], 1)
        self.assertAlmostEqual(meta["speed_bound_rad_s"], 0.02)

    def test_save_writes_reference_plan(self):
        cfg, start, trial = make(
            friction_joints=[1], friction_speeds_rad_s=[0.01, 0.02], friction_repeats=1
        )
        with tempfile.TemporaryDirectory() as tmp:
            trial.save(tmp)
            plan = json.loads((Path(tmp) / "reference_plan.json").read_text())
            self.assertEqual(plan["trajectory_type"], "friction")
            self.assertEqual(plan["joint_order"], [1])
            self.assertEqual(plan["repeats"], 1)
            self.assertGreater(plan["duration_s"], 0)


if __name__ == "__main__":
    unittest.main()
