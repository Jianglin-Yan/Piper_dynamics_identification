"""Bounds must detect intermediate peaks without sampling long paths in control ticks."""

import unittest
from unittest.mock import patch

import numpy as np

from piper_dynamics.real_robot.trials.trajectory import Segment, brake, check_path, transit
from piper_dynamics.real_robot.trials.validation import Reverse, Waypoints


class PathLimits(unittest.TestCase):
    def test_closed_form_matches_original_endpoint_system(self):
        rng = np.random.default_rng(33)
        for _ in range(30):
            q0, q1, v0, a0 = rng.normal(size=(4, 6))
            T = rng.uniform(0.5, 8)
            p = Segment(q0, q1, T, v0, a0)
            c0 = q0
            c1 = v0 * T
            c2 = a0 * T * T / 2
            tail = np.linalg.solve(
                np.array([[1, 1, 1], [3, 4, 5], [6, 12, 20]], float),
                np.array([q1 - c0 - c1 - c2, -c1 - 2 * c2, -2 * c2]),
            )
            np.testing.assert_allclose(p.c, np.array([c0, c1, c2, *tail]), atol=2e-12)

    def test_extrema_agree_with_dense_independent_polynomial_evaluation(self):
        rng = np.random.default_rng(34)
        grid = np.linspace(0, 1, 20001)
        for rest in (True, False):
            for _ in range(10):
                q0, q1 = rng.uniform(-1, 1, (2, 6))
                T = rng.uniform(1, 8)
                p = Segment(
                    q0,
                    q1,
                    T,
                    None if rest else rng.uniform(-0.5, 0.5, 6),
                    None if rest else rng.uniform(-0.5, 0.5, 6),
                )
                q = np.polynomial.polynomial.polyval(grid, p.c)
                vc = np.arange(1, 6)[:, None] * p.c[1:] / T
                v = np.polynomial.polynomial.polyval(grid, vc)
                ac = np.arange(1, 5)[:, None] * vc[1:] / T
                a = np.polynomial.polynomial.polyval(grid, ac)
                actual = (q.min(axis=1), q.max(axis=1), abs(v).max(axis=1), abs(a).max(axis=1))
                for proved, sampled in zip(p._limits, actual):
                    np.testing.assert_allclose(proved, sampled, atol=5e-6, rtol=1e-7)

    def test_interior_position_speed_and_acceleration_peaks_are_checked(self):
        p = Segment(np.zeros(6), np.zeros(6), 2, v0=np.full(6, 0.1))
        # Same endpoints do not make an initially moving braking path safe.
        with self.assertRaisesRegex(ValueError, "joint safety"):
            check_path(p, np.full(6, -1), np.full(6, 0.01), 1, 1)
        p = Segment(np.zeros(6), np.ones(6), 10)
        with self.assertRaisesRegex(ValueError, "velocity"):
            check_path(p, np.full(6, -1), np.full(6, 2), 0.18, 1)
        with self.assertRaisesRegex(ValueError, "acceleration"):
            check_path(p, np.full(6, -1), np.full(6, 2), 1, 0.05)

    def test_segment_composite_and_reverse_checks_never_sample_time_points(self):
        poses = np.array([[0] * 6, [0.2] * 6, [-0.2] * 6])
        p = Waypoints(poses, 0.04, 0.08)
        with patch.object(Segment, "at", side_effect=AssertionError("Control-loop dense sampling")):
            for path in (
                p,
                p.parts[0],
                Reverse(p),
                brake(np.zeros(6), np.full(6, 0.05), 0.12, 0.12),
            ):
                check_path(path, np.full(6, -1), np.ones(6), 0.12, 0.12)

    def test_unsafe_component_is_not_hidden_by_composite_or_reverse(self):
        p = Waypoints(np.array([[0] * 6, [1.1] * 6, [0] * 6]), 0.12, 0.12)
        for path in (p, Reverse(p)):
            with self.assertRaisesRegex(ValueError, "joint safety"):
                check_path(path, np.full(6, -1), np.ones(6), 0.12, 0.12)


if __name__ == "__main__":
    unittest.main()
