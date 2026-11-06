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
    """生成现有站立拖动配置，供离线配置和阶段机测试使用。"""
    c = json.loads((BASE / "profiles/config.json").read_text())
    c.update(trajectory="drag", standing_start=True, start_deg=[0, 45, -60, 0, 0, 0])
    configure_drag(c)
    validate_config(c)
    return c


class DragConfigureTests(unittest.TestCase):
    """验证拖动测试增益、前馈和默认名义模型的配置契约。"""
    def test_test_kp_is_two_and_feedforward_is_full(self):
        """验证测试Kp=2、前馈100%，准备与回位Kp保持10。"""
        c = profile()
        self.assertEqual(list(c["mit_kp"]), [2.0] * 6)
        self.assertEqual(list(c["mit_prepare_kp"]), [10.0] * 6)
        self.assertEqual(list(c["mit_return_kp"]), [10.0] * 6)
        self.assertEqual(c["mit_ff_scale"], 1.0)
        self.assertEqual(c["mit_ff_ramp_s"], 3.0)
        self.assertEqual(c["gravity_model"], "nominal")


class DragDampingTests(unittest.TestCase):
    """验证现有测试阻尼与手推监视阈值，不改变基础阶段增益。"""
    def test_drag_uses_light_damping_and_wide_monitors(self):
        """验证Kd=0.15及既有速度/电流监视值，准备回位Kd保持0.8。"""
        c = profile()
        self.assertEqual(list(c["mit_kd"]), [0.15] * 6)
        self.assertEqual(list(c["mit_prepare_kd"]), [0.8] * 6)
        self.assertEqual(list(c["mit_return_kd"]), [0.8] * 6)
        self.assertEqual(c["feedback_speed_stop_rad_s"], 2.0)
        self.assertEqual(c["feedback_speed_stop_duration_s"], 0.2)
        self.assertEqual(c["feedback_speed_emergency_rad_s"], 2.8)
        self.assertEqual(c["max_current_feedback_A"], [4.0, 4.0, 4.0, 3.0, 3.0, 3.0])


class DragStartupTests(unittest.TestCase):
    """验证停靠姿态被拖动命令范围覆盖，并保留现有1°内缩规则。"""
    def test_parked_pose_outside_j2_j3_limits_is_accepted(self):
        """验证略越界停靠位仍被覆盖，同时固件其余边界维持现有内缩1°。"""
        c = profile()
        c["simulation_initial_deg"] = [0.0, -3.0, 3.0, 0.0, 20.0, 0.0]
        validate_config(c)
        b = MitSimulation(c)
        k = Controller(b, c, b.snapshot(), 0, lambda m: None)
        home = b.snapshot()["q"]
        self.assertTrue(np.all(home >= k.lower - 1e-9))
        self.assertTrue(np.all(home <= k.upper + 1e-9))
        self.assertAlmostEqual(np.rad2deg(k.upper[3]), 99.0, places=6)
        self.assertAlmostEqual(np.rad2deg(k.lower[0]), -149.0, places=6)


class DragTrialTests(unittest.TestCase):
    """验证拖动跟随实测位置，以及LB+X在各阶段的结束映射。"""
    def make(self):
        """构造不连接硬件的拖动任务和起始关节姿态。"""
        c = profile()
        start = np.deg2rad([0.0, 45.0, -60.0, 0.0, 0.0, 0.0])
        return c, start, DragTrial(start, c)

    def test_tick_commands_the_measured_position(self):
        """验证测试周期发送实测姿态，不产生固定位置恢复目标。"""
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
        """验证TESTING阶段LB+X请求既有渐变结束与回位流程。"""
        calls = []
        fake = types.SimpleNamespace(
            drag=True,
            state="TESTING",
            request_stop=lambda s, now, reason: calls.append((reason, now)),
        )
        self.assertTrue(Controller._handle_actions(fake, 2.0, {}, "PREPARE"))
        self.assertEqual(calls, [("Drag test completed", 2.0)])

    def test_lb_x_in_waiting_is_not_the_drag_end(self):
        """验证WAITING阶段LB+X不会被误解释为拖动结束。"""
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
