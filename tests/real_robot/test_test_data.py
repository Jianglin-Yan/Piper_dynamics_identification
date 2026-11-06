import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from piper_dynamics.real_robot.app import validate_config
from piper_dynamics.real_robot.recording import TestRecorder, export_test_data
from piper_dynamics.real_robot.trials.trajectory import Excitation, ScaledTrajectory
from tests.real_robot.test_pilot import config


class TestScaling(unittest.TestCase):
    def test_anchor_derivatives_and_duration(self):
        anchor = np.array([0.1, 0.5, -1.0, 0.2, 0.3, 0.4])
        p = Excitation(anchor, 5, 0.08, 0.1, 20, 5)
        for scale in (0.25, 0.5, 1.0):
            scaled = ScaledTrajectory(p, anchor, scale)
            self.assertEqual(scaled.duration, p.duration)
            for t in (0.0, 1.2, 7.0, 17.8, 20.0):
                q, v, a = p.at(t)
                qs, vs, aas = scaled.at(t)
                np.testing.assert_allclose(qs - anchor, scale * (q - anchor), atol=1e-15)
                np.testing.assert_allclose(vs, scale * v)
                np.testing.assert_allclose(aas, scale * a)
                np.testing.assert_allclose(qs[:5], anchor[:5])

    def test_invalid_scale_rejected_before_devices(self):
        for scale in (0.0, -0.1, 1.01, np.nan, np.inf):
            c = config()
            c["trajectory_scale"] = scale
            with self.assertRaises(ValueError):
                validate_config(c)

    def test_speed_scale_changes_time_and_derivatives_without_changing_range(self):
        anchor = np.array([0.1, 0.5, -1.0, 0.2, 0.3, 0.4])
        source = Excitation(anchor, 5, 0.08, 0.1, 20, 5)
        fast = ScaledTrajectory(source, anchor, 0.5)
        slow = ScaledTrajectory(source, anchor, 0.5, 0.5)
        self.assertEqual(slow.duration, 40.0)
        for t in (0.0, 1.2, 9.0, 16.0, 29.0, 40.0):
            q, v, a = fast.at(t * 0.5)
            qs, vs, aas = slow.at(t)
            np.testing.assert_allclose(qs, q)
            np.testing.assert_allclose(vs, v * 0.5)
            np.testing.assert_allclose(aas, a * 0.25)
        for t in (0.0, 40.0):
            q, v, a = slow.at(t)
            np.testing.assert_allclose(q, anchor, atol=1e-12)
            np.testing.assert_allclose(v, 0, atol=1e-12)
            np.testing.assert_allclose(a, 0, atol=1e-12)

    def test_invalid_speed_scale_rejected_before_devices(self):
        for scale in (0.0, -0.1, 1.01, np.nan, np.inf):
            c = config()
            c["trajectory_speed_scale"] = scale
            with self.assertRaises(ValueError):
                validate_config(c)
            with self.assertRaises(ValueError):
                ScaledTrajectory(None, np.zeros(6), 0.5, scale)


class TestDataset(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name)
        self.c = config()
        self.c["trajectory_scale"] = 0.25
        self.ctrl = SimpleNamespace(
            state="READY",
            path_t=0.0,
            test=ScaledTrajectory(Excitation(np.zeros(6), 5, 0.08, 0.1, 20, 5), np.zeros(6), 0.25),
            start=np.zeros(6),
            last_target=np.zeros(6),
            stop_reason=None,
        )
        self.r = TestRecorder(self.path, self.c, "sim", version={"commit": "test", "dirty": False})

    def tearDown(self):
        if not self.r.closed:
            self.r.close(2.0, "teardown")
        self.tmp.cleanup()

    def snapshot(self, t):
        return {
            "time": t,
            "q": np.zeros(6),
            "qd": np.full(6, 2 * t),
            "current_A": np.ones(6) * 0.1,
            "tau_candidate_Nm": np.ones(6) * 0.2,
            "q_times": np.full(6, t),
            "drive_times": np.full(6, t),
            "low_times": np.full(6, t),
            "enabled": np.ones(6, bool),
            "faults": [0] * 6,
            "status": {"arm_status": 0, "error_code": 0},
            "temp_C": [25] * 6,
            "voltage_V": [24] * 6,
        }

    def step(self, t):
        self.r.update(self.ctrl, t, self.snapshot(t), t + 0.001)

    def begin(self):
        self.ctrl.state = "TESTING"
        self.step(0.0)

    def test_only_testing_samples_not_transition_or_braking(self):
        self.step(-1.0)
        self.begin()
        for n in range(1, 21):
            self.step(n * 0.01)
        self.ctrl.state = "BRAKING"
        self.ctrl.stop_reason = "LB+RB"
        self.step(0.21)
        self.ctrl.state = "RETURNING"
        self.step(0.3)
        self.r.close(0.4, "done")
        rows = [json.loads(x) for x in (self.path / "test_samples.jsonl").read_text().splitlines()]
        self.assertEqual(len(rows), 20)
        self.assertTrue(all(x["state"] == "TESTING" for x in rows))
        self.assertEqual(rows[0]["t_test_s"], 0.01)
        manifest = json.loads((self.path / "test_manifest.json").read_text())
        self.assertEqual(manifest["windows"][0]["end_monotonic_s"], 0.21)
        self.assertEqual(manifest["windows"][0]["end_reason"], "LB+RB")
        self.assertFalse(manifest["suitable_for_identification"])

    def test_raw_frames_are_selected_by_half_open_test_window(self):
        self.begin()
        self.step(0.01)
        self.ctrl.state = "RETURNING"
        self.step(0.2)
        self.r.close(0.3, "done")
        frames = [
            {"t_monotonic_s": t, "can_id": "0x251", "data_hex": "00" * 8}
            for t in (-0.1, 0.0, 0.15, 0.2, 0.3)
        ]
        (self.path / "raw_can.jsonl").write_text("".join(json.dumps(x) + "\n" for x in frames))
        summary = export_test_data(self.path)
        self.assertEqual(summary["raw_test_frame_count"], 2)
        selected = [
            json.loads(x)["t_monotonic_s"]
            for x in (self.path / "test_raw_can.jsonl").read_text().splitlines()
        ]
        self.assertEqual(selected, [0.0, 0.15])

    def test_can_loss_invalidates_affected_snapshot_and_marks_capture_quality(self):
        self.ctrl.state = "TESTING"
        for t, count in [(0.0, 5), (0.01, 5), (0.02, 7), (0.03, 7)]:
            snapshot = self.snapshot(t)
            snapshot["can_rx_dropped_total"] = count
            self.r.update(self.ctrl, t, snapshot, t + 0.001)
        self.ctrl.state = "RETURNING"
        self.step(0.04)
        self.r.close(0.05, "done")
        rows = [json.loads(x) for x in (self.path / "test_samples.jsonl").read_text().splitlines()]
        self.assertEqual([r["quality"]["valid_measurement"] for r in rows], [True, False, True])
        frames = [
            {"t_monotonic_s": t, "can_id": "0x251", "data_hex": "00" * 8, "rx_dropped_delta": count}
            for t, count in [(-1.0, 5), (0.02, 2), (0.05, 3)]
        ]
        (self.path / "raw_can.jsonl").write_text("".join(json.dumps(x) + "\n" for x in frames))
        summary = export_test_data(self.path)
        self.assertTrue(summary["can_loss_detected"])
        self.assertEqual(summary["can_rx_dropped_frames_detected_during_test"], 2)
        self.assertEqual(summary["can_loss_affected_snapshot_count"], 1)

    def test_acceleration_estimate_uses_measured_velocity_not_reference(self):
        self.begin()
        for n in range(1, 61):
            self.step(n * 0.01)
        self.ctrl.state = "RETURNING"
        self.step(0.61)
        self.r.close(0.7, "done")
        export_test_data(self.path)
        with np.load(self.path / "test_signals.npz") as d:
            self.assertTrue(d["qdd_est_valid"].any())
            np.testing.assert_allclose(d["qdd_est"][d["qdd_est_valid"]], 2.0, atol=1e-10)
            self.assertGreater(np.max(np.abs(d["qdd_ref"][:, 5] - 2)), 1.0)
            self.assertTrue(np.all(~d["qdd_est_valid"][:5]))

    def test_gap_cannot_acquire_valid_acceleration(self):
        self.begin()
        for n in list(range(1, 20)) + list(range(40, 70)):
            self.step(n * 0.01)
        self.ctrl.state = "RETURNING"
        self.step(0.7)
        self.r.close(0.8, "done")
        export_test_data(self.path)
        with np.load(self.path / "test_signals.npz") as d:
            around_gap = (d["t_uniform_monotonic_s"] > 0.2) & (d["t_uniform_monotonic_s"] < 0.39)
            self.assertFalse(d["qdd_est_valid"][around_gap].any())
            self.assertTrue(np.isnan(d["qdd_est"][around_gap]).all())

    def test_stale_data_retained_but_marked_invalid(self):
        self.begin()
        s = self.snapshot(0.2)
        s["drive_times"][:] = 0.01
        self.r.update(self.ctrl, 0.2, s, 0.201)
        self.r.close(0.3, "aborted")
        row = json.loads((self.path / "test_samples.jsonl").read_text())
        self.assertFalse(row["quality"]["valid_measurement"])
        self.assertEqual(row["feedback"]["drive_times"], [0.01] * 6)

    def test_zero_samples_has_explicit_empty_dataset(self):
        self.r.close(1.0, "not_started")
        summary = export_test_data(self.path)
        self.assertEqual(summary["sample_count"], 0)
        with np.load(self.path / "test_signals.npz") as d:
            self.assertEqual(d["q"].shape, (0, 6))

    def test_speed_scale_and_stretched_duration_are_recorded(self):
        self.c["trajectory_speed_scale"] = 0.5
        self.ctrl.test = ScaledTrajectory(self.ctrl.test.path, np.zeros(6), 0.25, 0.5)
        self.begin()
        self.step(0.1)
        self.r.close(0.2, "interrupted")
        manifest = json.loads((self.path / "test_manifest.json").read_text())
        self.assertEqual(manifest["windows"][0]["trajectory_speed_scale"], 0.5)
        self.assertEqual(manifest["windows"][0]["planned_duration_s"], 40.0)
        row = json.loads((self.path / "test_samples.jsonl").read_text())
        self.assertEqual(row["trajectory_speed_scale"], 0.5)


if __name__ == "__main__":
    unittest.main()
