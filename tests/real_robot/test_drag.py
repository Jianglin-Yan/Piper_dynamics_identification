"""Drag trial configuration, reference-following and LB+X end mapping."""

import json
import types
import unittest
from pathlib import Path

import numpy as np

from piper_dynamics.real_robot.app import validate_config
from piper_dynamics.real_robot.control.mit_trial import configure_drag
from piper_dynamics.real_robot.controller import Controller
from piper_dynamics.real_robot.control.mit_trial import MitSimulation
from piper_dynamics.real_robot.trials.drag import DragTrial

BASE = Path(__file__).resolve().parents[2] / "src/piper_dynamics/real_robot"


def profile():
    c = json.loads((BASE / "profiles/config.json").read_text())
    c.update(trajectory="drag", standing_start=True, start_deg=[0, 45, -60, 0, 0, 0])
    configure_drag(c)
    validate_config(c)
    return c


class DragConfigureTests(unittest.TestCase):
    def test_test_kp_is_two_and_feedforward_is_full(self):
        c = profile()
        self.assertEqual(list(c["mit_kp"]), [2.0] * 6)
        self.assertEqual(list(c["mit_prepare_kp"]), [10.0] * 6)
        self.assertEqual(list(c["mit_return_kp"]), [10.0] * 6)
        self.assertEqual(c["mit_ff_scale"], 1.0)
        self.assertEqual(c["mit_ff_ramp_s"], 3.0)
        self.assertEqual(c["gravity_model"], "nominal")


class DragDampingTests(unittest.TestCase):
    def test_drag_uses_light_damping_and_wide_monitors(self):
        c = profile()
        self.assertEqual(list(c["mit_kd"]), [0.2] * 6)
        self.assertEqual(list(c["mit_prepare_kd"]), [0.8] * 6)
        self.assertEqual(list(c["mit_return_kd"]), [0.8] * 6)
        self.assertEqual(c["feedback_speed_stop_rad_s"], 2.0)
        self.assertEqual(c["feedback_speed_stop_duration_s"], 0.2)
        self.assertEqual(c["feedback_speed_emergency_rad_s"], 2.8)
        self.assertEqual(c["max_current_feedback_A"], [4.0, 4.0, 4.0, 3.0, 3.0, 3.0])


class DragStartupTests(unittest.TestCase):
    def test_parked_pose_outside_j2_j3_limits_is_accepted(self):
        c = profile()
        c["simulation_initial_deg"] = [0.0, -3.0, 3.0, 0.0, 20.0, 0.0]
        validate_config(c)
        b = MitSimulation(c)
        k = Controller(b, c, b.snapshot(), 0, lambda m: None)
        home = b.snapshot()["q"]
        self.assertTrue(np.all(home >= k.lower - 1e-9))
        self.assertTrue(np.all(home <= k.upper + 1e-9))


class DragTrialTests(unittest.TestCase):
    def make(self):
        c = profile()
        start = np.deg2rad([0.0, 45.0, -60.0, 0.0, 0.0, 0.0])
        return c, start, DragTrial(start, c)

    def test_tick_commands_the_measured_position(self):
        c, start, trial = self.make()
        commands, events = [], []
        fake = types.SimpleNamespace(command=commands.append, event=events.append)
        trial.begin(fake, 0.0)
        measured = start + np.deg2rad([1, -2, 3, -1, 2, -3])
        trial.tick(fake, 1.0, {"q": measured, "qd": np.zeros(6)}, None)
        np.testing.assert_allclose(commands[-1], measured)
        self.assertEqual(trial.metadata({"qd": np.zeros(6)}, 1.0)["protocol"], "drag")
        self.assertTrue(events and "零力拖动" in events[0])

    def test_lb_x_during_testing_requests_the_ramped_drag_end(self):
        calls = []
        fake = types.SimpleNamespace(
            drag=True,
            state="TESTING",
            request_stop=lambda s, now, reason: calls.append((reason, now)),
        )
        self.assertTrue(Controller._handle_actions(fake, 2.0, {}, "PREPARE"))
        self.assertEqual(calls, [("Drag test completed", 2.0)])

    def test_lb_x_in_waiting_is_not_the_drag_end(self):
        calls = []
        fake = types.SimpleNamespace(
            drag=True,
            state="WAITING",
            request_stop=lambda s, now, reason: calls.append(reason),
        )
        self.assertTrue(Controller._handle_actions(fake, 2.0, {}, "PREPARE"))
        self.assertEqual(calls, [])


if __name__ == "__main__":
    unittest.main()
