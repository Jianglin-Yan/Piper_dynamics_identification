import contextlib
import io
import unittest
from unittest.mock import patch

import numpy as np

from piper_dynamics.real_robot.backends.hardware import Hardware
from piper_dynamics.real_robot.controller import Controller
from piper_dynamics.real_robot.monitoring import CurrentMonitor, SpeedMonitor, validate_speed_limits
from tests.real_robot.test_pilot import Fake, config, tick


class TestMonitors(unittest.TestCase):
    def test_recorded_speed_spike_does_not_stop(self):
        monitor = SpeedMonitor(config())
        for n, v in enumerate(
            [
                -0.009,
                -0.076,
                0.013,
                -0.021,
                -0.009,
                -0.015,
                -0.001,
                -0.001,
                0.007,
                0.001,
                -0.001,
                -0.189,
            ]
        ):
            qd = np.zeros(6)
            qd[4] = v
            self.assertIsNone(monitor.update(n * 0.01, qd)["level"])

    def test_warning_and_sustained_stop_are_distinct(self):
        monitor = SpeedMonitor(config())
        v = np.zeros(6)
        v[4] = 0.4
        self.assertEqual(monitor.update(0, v)["level"], "warning")
        self.assertIsNone(monitor.update(0.2, v)["level"])
        v[4] = 0.65
        self.assertNotEqual(monitor.update(0.3, v)["level"], "stop")
        self.assertNotEqual(monitor.update(0.44, v)["level"], "stop")
        self.assertEqual(monitor.update(0.45, v)["level"], "stop")

    def test_clear_and_different_joint_do_not_accumulate(self):
        monitor = SpeedMonitor(config())
        v = np.zeros(6)
        v[4] = 0.65
        monitor.update(0, v)
        v[:] = 0
        monitor.update(0.1, v)
        v[4] = 0.65
        self.assertNotEqual(monitor.update(0.2, v)["level"], "stop")
        v[:] = 0
        v[1] = 0.65
        self.assertNotEqual(monitor.update(0.3, v)["level"], "stop")

    def test_single_extreme_sample_is_not_an_emergency(self):
        monitor = SpeedMonitor(config())
        v = np.zeros(6)
        v[4] = 1.3
        self.assertNotEqual(monitor.update(0, v)["level"], "emergency")
        v[4] = 0
        self.assertIsNone(monitor.update(0.01, v)["level"])

    def test_extreme_speed_requires_sustained_feedback(self):
        c = config()
        monitor = SpeedMonitor(c)
        v = np.zeros(6)
        v[4] = 1.3
        duration = c["feedback_speed_emergency_duration_s"]
        self.assertNotEqual(monitor.update(0, v)["level"], "emergency")
        self.assertNotEqual(monitor.update(duration - 0.01, v)["level"], "emergency")
        self.assertEqual(monitor.update(duration, v)["level"], "emergency")

    def test_extreme_speed_that_drops_below_threshold_never_faults(self):
        c = config()
        monitor = SpeedMonitor(c)
        v = np.zeros(6)
        v[4] = 1.3
        for n in range(5):
            monitor.update(n * 0.01, v)
        v[4] = 0.4
        monitor.update(0.05, v)
        v[4] = 1.3
        self.assertNotEqual(monitor.update(0.06, v)["level"], "emergency")

    def test_current_warning_fires_before_the_stop_line(self):
        m = CurrentMonitor(config())
        v = np.zeros(6)
        v[4] = 1.6
        self.assertEqual(m.update(0, v)["level"], "warning")
        for n in range(1, 30):
            self.assertNotEqual(m.update(n * 0.01, v)["level"], "stop")

    def test_current_transient_sustained_and_emergency(self):
        m = CurrentMonitor(config())
        v = np.zeros(6)
        v[4] = 2.6
        self.assertEqual(m.update(0, v)["level"], "warning")
        self.assertNotEqual(m.update(0.29, v)["level"], "stop")
        self.assertEqual(m.update(0.3, v)["level"], "stop")
        v[4] = 0
        m.update(0.31, v)
        v[4] = 2.6
        self.assertNotEqual(m.update(0.32, v)["level"], "stop")
        v[4] = 3.9
        level = None
        for n in range(6):
            level = m.update(0.33 + n * 0.01, v)["level"]
        self.assertEqual(level, "emergency")

    def test_invalid_threshold_order_refused(self):
        c = config()
        c["feedback_speed_stop_rad_s"] = 0.2
        with self.assertRaises(ValueError):
            validate_speed_limits(c)


class TestControlledStop(unittest.TestCase):
    def make(self):
        c = config()
        b = Fake(c)
        events = []
        k = Controller(b, c, b.snapshot(), 0, events.append)
        b.enabled = True
        b.active = True
        k.active = True
        k.armed = True
        k.state = "TESTING"
        k.path = k.test
        k.path_t = 0
        return c, b, k, events

    def test_sustained_speed_brakes_reference_returns_and_disables(self):
        c, b, k, events = self.make()
        for n in range(16):
            t = 1 + n * 0.01
            b.q = k.last_target.copy()
            s = b.snapshot()
            s["qd"][5] = 0.65
            tick(k, t, s)
        self.assertEqual(k.state, "BRAKING")
        self.assertEqual(b.stops, [])
        self.assertLess(np.max(np.abs(k.path.at(0)[1])), c["max_velocity_rad_s"])
        # The fictitious .65 sample must never become a commanded .65 velocity.
        for n in range(1, 600):
            t = 1.15 + n * 0.01
            tick(k, t, b.snapshot())
            if k.finished:
                break
        self.assertEqual(k.state, "DONE")
        self.assertFalse(b.enabled)
        self.assertFalse(b.active)
        self.assertEqual(b.stops, [])
        count = len(b.commands)
        tick(k, t + 0.01, b.snapshot(), "START")
        self.assertEqual(len(b.commands), count)
        self.assertTrue(any("停止：" in e for e in events))

    def test_manual_stop_waiting_sends_no_commands(self):
        c = config()
        b = Fake(c)
        k = Controller(b, c, b.snapshot(), 0, lambda _: None)
        k.request_stop(b.snapshot(), 0.1, "operator stop")
        for n in range(11, 40):
            tick(k, n * 0.01, b.snapshot())
        self.assertEqual(k.state, "DONE")
        self.assertEqual(b.stops, [])
        self.assertEqual(b.commands, [])

    def test_invalid_feedback_during_braking_still_uses_fault_stop(self):
        c, b, k, events = self.make()
        k.request_stop(b.snapshot(), 0.1, "test stop")
        self.assertEqual(k.state, "BRAKING")
        b.bad = True
        tick(k, 0.11, b.snapshot())
        self.assertEqual(k.state, "FAULT")
        self.assertEqual(len(b.stops), 1)

    def test_sustained_tracking_error_uses_controlled_stop(self):
        c, b, k, events = self.make()
        for n in range(25):
            s = b.snapshot()
            s["q"][5] += np.deg2rad(36.0)
            tick(k, 1 + n * 0.01, s)
        self.assertEqual(k.state, "BRAKING")
        self.assertEqual(b.stops, [])

    def test_sustained_emergency_current_uses_controlled_stop(self):
        c, b, k, events = self.make()
        for n in range(35):
            t = 1 + n * 0.01
            b.q = k.last_target.copy()
            s = b.snapshot()
            s["qd"][:] = 0
            s["current_A"] = np.zeros(6)
            s["current_A"][4] = 2.6
            tick(k, t, s)
        self.assertEqual(k.state, "BRAKING")
        self.assertEqual(b.stops, [])

    def test_current_limit_uses_return_when_feedback_healthy(self):
        c, b, k, events = self.make()
        for n in range(35):
            t = 1 + n * 0.01
            b.q = k.last_target.copy()
            s = b.snapshot()
            s["qd"][:] = 0
            s["current_A"] = np.zeros(6)
            s["current_A"][4] = 2.6
            tick(k, t, s)
        self.assertEqual(k.state, "BRAKING")
        self.assertEqual(b.stops, [])

    def test_bad_hold_path_falls_back_instead_of_issuing_out_of_bounds_target(self):
        c, b, k, events = self.make()
        k.last_target[5] = k.upper[5]
        b.q = k.last_target.copy()
        k.path = type(
            "Moving", (), {"at": lambda _, t: (b.q.copy(), np.ones(6) * 0.1, np.zeros(6))}
        )()
        before = len(b.commands)
        k.request_stop(b.snapshot(), 0.1, "test stop")
        self.assertEqual(k.state, "FAULT")
        self.assertEqual(len(b.commands), before)
        self.assertEqual(len(b.stops), 1)

    def test_arming_requires_stable_can_mode_and_fresh_enable(self):
        c = config()
        b = Fake(c)
        k = Controller(b, c, b.snapshot(), 0, lambda _: None)
        k.activate(b.snapshot(), 0, "TEST")

        def feed(t, mode, stamp):
            s = b.snapshot()
            s["status"] = {"control_mode": mode}
            s["low_times"] = np.full(6, stamp)
            tick(k, t, s)

        feed(0.01, 0, 0.01)
        self.assertEqual(k.state, "ARMING")
        feed(0.02, 1, -1)
        self.assertEqual(k.state, "ARMING")
        feed(0.03, 1, 0.03)
        feed(0.08, 1, 0.08)
        self.assertEqual(k.state, "ARMING")
        feed(0.14, 1, 0.14)
        self.assertEqual(k.state, "TESTING")

    def test_lb_rb_ignores_recorded_motor_speed_transient_for_brake_velocity(self):
        c, b, k, events = self.make()
        b.q = k.last_target.copy()
        s = b.snapshot()
        s["qd"][4] = -0.189
        tick(k, 1, s, "ABORT")
        self.assertEqual(k.state, "BRAKING")
        self.assertEqual(b.stops, [])
        self.assertLess(np.max(np.abs(k.path.at(0)[1])), c["max_velocity_rad_s"])

    def test_stable_park_uses_position_window_despite_noisy_speed(self):
        c, b, k, events = self.make()
        k.state = "SETTLING_HOME"
        k.settle_started = 0
        for n in range(60):
            s = b.snapshot()
            s["qd"][:] = 0.1
            tick(k, n * 0.01, s)
            if k.state == "DISABLING":
                break
        self.assertEqual(k.state, "DISABLING")
        self.assertEqual(b.stops, [])

    def test_position_drift_cannot_be_reported_as_settled_park(self):
        c, b, k, events = self.make()
        k.state = "SETTLING_HOME"
        k.settle_started = 0
        for n in range(60):
            s = b.snapshot()
            s["q"][5] += np.deg2rad(0.18) * np.sin(n * 0.3)
            s["qd"][:] = 0
            tick(k, n * 0.01, s)
        self.assertEqual(k.state, "SETTLING_HOME")
        self.assertEqual(b.stops, [])

    def test_extreme_speed_keeps_fault_stop(self):
        c, b, k, events = self.make()
        for n in range(12):
            s = b.snapshot()
            s["qd"][4] = 1.3
            tick(k, 1 + n * 0.01, s)
        self.assertEqual(k.state, "FAULT")
        self.assertEqual(len(b.stops), 1)

    def test_single_extreme_speed_sample_does_not_fault(self):
        c, b, k, events = self.make()
        s = b.snapshot()
        s["qd"][4] = 1.3
        tick(k, 1, s)
        s = b.snapshot()
        tick(k, 1.01, s)
        self.assertEqual(k.state, "TESTING")
        self.assertEqual(len(b.stops), 0)


class TestExplicitRecovery(unittest.TestCase):
    """验证恢复确认先于配置读取，且复位不会隐式使能电机。"""
    def test_requires_support_confirmation_before_any_file_or_device(self):
        """未确认机械臂支撑时，保留拒绝恢复及不读取配置的断言。"""
        from piper_dynamics.real_robot.app import main

        with (
            patch("sys.argv", ["pilot", "--recover"]),
            patch("piper_dynamics.real_robot.app.Path.read_text", side_effect=AssertionError("read config")),
            contextlib.redirect_stderr(io.StringIO()),
        ):
            with self.assertRaises(SystemExit) as error:
                main()
        self.assertEqual(error.exception.code, 2)

    def test_one_reset_and_no_subsequent_position_or_enable(self):
        import threading

        h = Hardware.__new__(Hardware)
        h.tx_lock = threading.Lock()
        h.active = False
        h.stopped = False
        frames = []
        h._send_locked = lambda cid, data: frames.append((cid, bytes(data)))
        h.reset_supported()
        self.assertEqual(frames, [(0x150, bytes([2, 0, 0, 0, 0, 0, 0, 0]))])
        with self.assertRaises(RuntimeError):
            h.arm(np.zeros(6))
        with self.assertRaises(RuntimeError):
            h.command(np.zeros(6))
        with self.assertRaises(RuntimeError):
            h.reset_supported()
        h.active = True
        with self.assertRaises(RuntimeError):
            h.reset_supported()
        self.assertEqual(len(frames), 1)

    def test_adapter_cannot_finish_or_close_a_powered_motion_session(self):
        from unittest.mock import Mock

        h = Hardware.__new__(Hardware)
        h.active = True
        h.motion_started = True
        h.disable_acknowledged = False
        h.quick_stop = Mock()
        with self.assertRaisesRegex(RuntimeError, "disable not confirmed"):
            h.finish()
        with self.assertRaisesRegex(RuntimeError, "Refusing adapter close"):
            h.close()


class TestStopPreview(unittest.TestCase):
    def test_electronic_stop_preview_does_not_freeze_against_gravity(self):
        from piper_dynamics.real_robot.backends.simulation import Simulation

        c = config()
        b = Simulation(c)
        b.arm(b.snapshot()["q"])
        initial = b.snapshot()["q"].copy()
        b.quick_stop("injected hard fault")
        b.advance(0.5)
        self.assertGreater(np.max(np.abs(b.snapshot()["q"] - initial)), 0.01)
        self.assertTrue(np.all(b.snapshot()["enabled"]))


if __name__ == "__main__":
    unittest.main()
