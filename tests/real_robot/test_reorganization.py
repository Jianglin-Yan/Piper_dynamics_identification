"""Behavior contracts for configuration, compatibility and complete session exits."""

import importlib
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

from piper_dynamics.paths import PROJECT_ROOT, ROBOT_PROFILES
from piper_dynamics.real_robot.backends.mit import MitSimulation
from piper_dynamics.real_robot.cli import build_parser
from piper_dynamics.real_robot.configuration import build_config, validate_config
from piper_dynamics.real_robot.control.mit_trial import configure
from piper_dynamics.real_robot.controller import Controller


class ConfigurationContracts(unittest.TestCase):
    def friction_config(self, *extra):
        parser = build_parser()
        args = parser.parse_args(["--sim", "--standing", "--trajectory", "friction", *extra])
        return build_config(args, parser)

    def test_default_friction_keeps_the_reviewed_nonascending_order(self):
        cfg = self.friction_config()
        self.assertEqual(cfg["friction_joints"], [1, 6, 4, 5, 2, 3])
        self.assertTrue(cfg["standing_start"])

    def test_explicit_joint_order_is_preserved(self):
        cfg = self.friction_config("--friction-joints", "6", "2", "1")
        self.assertEqual(cfg["friction_joints"], [6, 2, 1])

    def test_invalid_joint_lists_are_rejected(self):
        cfg = self.friction_config()
        for joints in [[], [1, 1], [0], [7], [True], [1.2], ["1"]]:
            with self.subTest(joints=joints):
                candidate = {**cfg, "friction_joints": joints}
                with self.assertRaises(ValueError):
                    validate_config(candidate)

    def test_current_pose_friction_has_effective_current_pose_configuration(self):
        cfg = self.friction_config("--friction-start", "current")
        self.assertFalse(cfg["standing_start"])
        self.assertIsNone(cfg["start_deg"])

    def test_mit_profile_retains_the_current_gains_and_independent_windows(self):
        parser = build_parser()
        args = parser.parse_args(["--sim", "--standing", "--trajectory", "mit"])
        cfg = build_config(args, parser)
        for key in ["mit_kp", "mit_prepare_kp", "mit_return_kp"]:
            self.assertEqual(cfg[key], [10.0] * 6)
        for key in ["mit_kd", "mit_prepare_kd", "mit_return_kd"]:
            self.assertEqual(cfg[key], [0.8] * 6)
        self.assertEqual(cfg["mit_prepare_tolerance_deg"], 8.0)
        self.assertEqual(cfg["mit_park_tolerance_deg"], 8.0)
        self.assertEqual(cfg["park_release_tolerance_deg"], 3.0)

    def test_legacy_module_names_share_the_canonical_objects(self):
        aliases = {
            "controller": "piper_dynamics.real_robot.controller",
            "run": "piper_dynamics.real_robot.app",
            "hardware": "piper_dynamics.real_robot.backends.hardware",
            "identify_offline": "piper_dynamics.pipeline.measured_run",
        }
        for old, canonical in aliases.items():
            with self.subTest(module=old):
                self.assertIs(
                    importlib.import_module("real_robot." + old), importlib.import_module(canonical)
                )


class FrozenControlBehavior(unittest.TestCase):
    """Compare to sampled observations from commit 4dc8d1a, not another controller."""

    def check_case(self, alpha, abort):
        baseline = json.loads((PROJECT_ROOT / "tests/fixtures/control_behavior.json").read_text())
        expected = baseline["cases"][f"{alpha}:{abort}"]
        checkpoints = {entry["tick"]: entry["row"] for entry in expected["samples"]}
        cfg = json.loads((ROBOT_PROFILES / "config.json").read_text())
        cfg.update(trajectory="mit", standing_start=True, start_deg=[0, 45, -60, 0, 0, 0])
        configure(cfg, alpha)
        backend = MitSimulation(cfg)
        events = []
        controller = Controller(backend, cfg, backend.snapshot(), 0, events.append)
        started = interrupted = False
        for tick in range(expected["ticks"]):
            backend.advance(0.01)
            sample = backend.snapshot()
            now = sample["time"]
            action = "PREPARE" if tick == 10 else None
            if controller.state == "READY" and not started:
                action = "START"
                started = True
            if abort and controller.state == abort and not interrupted:
                action = "ABORT"
                interrupted = True
            controller.tick(now, sample, action)
            if tick in checkpoints:
                row = checkpoints[tick]
                self.assertEqual(controller.state, row[1])
                values = [now, *sample["q"], *controller.last_target, controller.mit_ff_ratio]
                np.testing.assert_allclose(values, [row[0], *row[2:]], rtol=0, atol=1e-9)
        self.assertTrue(controller.finished)
        self.assertTrue(controller.disarm_confirmed)
        self.assertEqual(events, expected["events"])

    def test_pure_pd_normal(self):
        self.check_case(0.0, None)

    def test_feedforward_normal(self):
        self.check_case(0.25, None)

    def test_preparation_abort(self):
        self.check_case(0.25, "PREPARING")

    def test_testing_abort(self):
        self.check_case(0.25, "TESTING")


class SessionLifecycleContracts(unittest.TestCase):
    def run_session(self, scripted):
        with tempfile.TemporaryDirectory() as folder:
            command = [
                sys.executable,
                "-m",
                "piper_dynamics.real_robot.app",
                "--sim",
                "--standing",
                "--trajectory",
                "mit",
                "--ff-scale",
                ".25",
                "--headless",
                "--scripted",
                scripted,
                "--max-seconds",
                "30",
                "--output",
                folder,
            ]
            env = {
                **os.environ,
                "PYTHONPATH": str(PROJECT_ROOT / "src"),
                "PYTHONDONTWRITEBYTECODE": "1",
            }
            run = subprocess.run(
                command, cwd=PROJECT_ROOT, env=env, capture_output=True, text=True, timeout=45
            )
            self.assertEqual(run.returncode, 0, run.stderr + run.stdout[-2000:])
            results = list(Path(folder).glob("*/result.json"))
            self.assertEqual(len(results), 1)
            result = json.loads(results[0].read_text())
            self.assertEqual(result["state"], "DONE")
            self.assertTrue(result["completed"])
            self.assertTrue(result["motors_disabled_confirmed"])
            self.assertFalse(any(result["final_enabled"]))
            self.assertEqual(result["aborted"], scripted == "abort")
            manifest = json.loads((results[0].parent / "test_manifest.json").read_text())
            self.assertEqual(
                manifest["scope"], "TESTING only; prepare, settle, brake, return, disable excluded"
            )
            self.assertGreater(manifest["sample_count"], 0)

    def test_complete_headless_cli_session(self):
        self.run_session("normal")

    def test_aborted_headless_cli_session(self):
        self.run_session("abort")
