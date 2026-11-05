"""两个入口只负责把固定参数拼进 argv；真正的参数校验仍在 run.validate_config。"""

import unittest

from piper_dynamics.real_robot.entries.mit import build_argv as mit_argv
from piper_dynamics.real_robot.entries.movej import build_argv as movej_argv


class TestEntries(unittest.TestCase):
    def test_mit_injects_standing_and_trajectory(self):
        self.assertEqual(mit_argv(["--sim"]), ["--standing", "--trajectory", "mit", "--sim"])
        self.assertEqual(
            mit_argv(["--hardware", "--ff-scale", "0.1"]),
            ["--standing", "--trajectory", "mit", "--hardware", "--ff-scale", "0.1"],
        )

    def test_mit_rejects_trajectory_override(self):
        with self.assertRaises(SystemExit):
            mit_argv(["--trajectory", "gravity"])

    def test_movej_maps_trial(self):
        self.assertEqual(
            movej_argv(["--sim", "--trial", "static"]),
            ["--standing", "--trajectory", "static", "--sim"],
        )
        self.assertEqual(movej_argv(["--sim"]), ["--standing", "--trajectory", "static", "--sim"])
        self.assertEqual(
            movej_argv(["--sim", "--trial=payload"]),
            ["--standing", "--trajectory", "payload", "--sim"],
        )

    def test_movej_rejects_unknown_or_override(self):
        with self.assertRaises(SystemExit):
            movej_argv(["--trial", "mit"])
        with self.assertRaises(SystemExit):
            movej_argv(["--trajectory", "mit"])


if __name__ == "__main__":
    unittest.main()
