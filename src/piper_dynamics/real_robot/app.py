"""PiPER session orchestration; configuration, control and storage have explicit owners."""

import datetime
import json
import signal
import sys
import time
from pathlib import Path

import numpy as np

from piper_dynamics.paths import LEGACY_ROBOT_ROOT as BASE
from piper_dynamics.real_robot.cli import build_parser, handle_input_only, validate_arguments
from piper_dynamics.real_robot.configuration import build_config, validate_config
from piper_dynamics.real_robot.controller import Controller
from piper_dynamics.real_robot.gamepad import Gamepad
from piper_dynamics.real_robot.recording import TestRecorder, export_test_data
from piper_dynamics.real_robot.ui import Printer
from piper_dynamics.utils.serialization import serializable


class Session:
    """Resources and lifecycle of one PiPER experiment, with no global session state."""

    def __init__(self, args, config):
        "Own one session: resources, diagnostics and controller; control math stays outside."
        self.args = args
        self.config = config
        self.selected = config["trajectory"]
        self.path = self.args.output / datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.path.mkdir(parents=True)
        (self.path / "config.json").write_text(
            json.dumps(self.config, ensure_ascii=False, indent=2)
        )
        self.backend = None
        self.pad = None
        self.viewer = None
        self.controller = None
        self.lease = None
        self.recorder = None
        self.result = {
            "mode": "sim"
            if self.args.sim
            else "recovery"
            if self.args.recover
            else "read-only"
            if self.args.read_only
            else "hardware",
            "completed": False,
        }
        self.events = (self.path / "events.jsonl").open("w", buffering=1)
        self.samples = (self.path / "samples.jsonl").open("w", buffering=65536)
        self.log_failure = None
        self.ui = Printer(self.emit)
        self.print_header()
        from piper_dynamics.real_robot.operator_exit import OperatorExit

        self.operator_exit = OperatorExit(self.emit)
        self.old_handlers = {
            s: signal.signal(s, self.operator_exit.signal) for s in (signal.SIGINT, signal.SIGTERM)
        }
        self.sim_note = "实机"

    def emit(self, msg):
        "Write a session event and remember logging failures."
        try:
            print(msg, flush=True)
        except OSError:
            pass
        try:
            self.events.write(
                json.dumps({"t_monotonic_s": time.monotonic(), "message": msg}, ensure_ascii=False)
                + "\n"
            )
        except OSError as error:
            self.log_failure = str(error)

    def record(self, row):
        "Write one full-session diagnostic sample."
        if (
            self.controller is not None
            and getattr(self.controller, "mit_telemetry", None) is not None
        ):
            row["mit"] = self.controller.mit_telemetry
        try:
            self.samples.write(json.dumps(row, default=serializable) + "\n")
        except OSError as error:
            self.log_failure = str(error)

    def print_header(self):
        "Describe the selected trial once at session startup."
        if self.selected == "drag":
            self.ui.header(
                "MIT 零力拖动 · 重力补偿",
                [
                    "控制方式：MIT（Kp 2，前馈 100%）",
                    "任务：LB+X 准备到站立位，LB+Y 开始记录；先静置，再手推；LB+X 结束并回位",
                ],
            )
        elif self.selected == "mit":
            self.ui.header(
                "MIT 固定轨迹 · 重力前馈验证",
                [
                    "控制方式：MIT（PD＋重力前馈）",
                    f"前馈比例：{100 * self.config['mit_ff_scale']:.0f}%",
                    "任务：五姿态固定轨迹，核对辨识模型的重力补偿",
                ],
            )
        elif self.selected == "gravity":
            self.ui.header(
                "MIT 重力补偿试验",
                [
                    "控制方式：MIT（PD＋重力前馈）",
                    f"前馈比例：{100 * self.config['mit_ff_scale']:.0f}%",
                    "任务：重力补偿保持",
                ],
            )
        elif self.selected == "multisine":
            self.ui.header(
                "MOVE J 六轴多正弦",
                [
                    "控制方式：MOVE J（固件位置控制）",
                    f"任务：{100 * float(self.config.get('trajectory_scale', 1)):.0f}% 幅度六轴多正弦",
                ],
            )
        else:
            names = {
                "static": "静态验证",
                "reciprocal": "正反对照",
                "payload": "负载测试",
                "pilot": "J6 试采",
            }
            self.ui.header(
                f"MOVE J {names.get(self.config['trajectory'], self.config['trajectory'])}",
                [
                    "控制方式：MOVE J（固件位置控制）",
                    f"任务：{names.get(self.config['trajectory'], self.config['trajectory'])}",
                ],
            )

    def prepare_backend(self):
        "Select transport while retaining the original preflight/recovery behavior."
        if self.args.sim:
            return self.prepare_simulation()
        return self.prepare_hardware()

    def prepare_simulation(self):
        "Construct the existing trial-specific MuJoCo backend."
        if self.config.get("passive_wrist", False):
            from piper_dynamics.real_robot.backends.validation_simulation import (
                ValidationSimulation as Simulation,
            )
        elif self.selected in ("mit", "drag"):
            from piper_dynamics.real_robot.control.mit_trial import MitSimulation as Simulation
        elif self.selected == "gravity":
            from piper_dynamics.real_robot.control.gravity_hold import (
                GravitySimulation as Simulation,
            )
        else:
            from piper_dynamics.real_robot.backends.simulation import Simulation
        self.backend = Simulation(self.config)
        self.sim_note = "仿真预览（未连接机械臂）"
        return True

    def hardware_type(self):
        "Select the current hardware command implementation."
        if self.selected in ("mit", "drag"):
            from piper_dynamics.real_robot.control.mit_trial import MitHardware as Hardware
        elif self.selected == "gravity":
            from piper_dynamics.real_robot.control.gravity_hold import GravityHardware as Hardware
        else:
            from piper_dynamics.real_robot.backends.hardware import Hardware
        return Hardware

    def acquire_channel(self):
        "Hold the existing exclusive CAN session lock."
        if self.args.hardware or self.args.recover:
            import fcntl
            import re

            if not re.fullmatch("[A-Za-z0-9_-]+", self.config["channel"]):
                raise ValueError("Invalid CAN channel name")
            self.lease = open("/tmp/piper-pilot-" + self.config["channel"] + ".lock", "w")
            fcntl.flock(self.lease, fcntl.LOCK_EX | fcntl.LOCK_NB)

    def review_previous_run(self):
        "Apply the existing operator recovery-confirmation policy."
        if self.args.hardware:
            from piper_dynamics.real_robot.session_history import (
                previous_controller_session,
                recovery_notice,
            )

            self.history = previous_controller_session(
                [BASE.parent / "runs", self.args.output], self.config["channel"], exclude=self.path
            )
            self.result["previous_controller_session"] = self.history
            if self.history:
                self.emit(
                    "STARTUP_HISTORY：" + self.history["reason"] + " 记录：" + self.history["run"]
                )
            self.pending = self.path.parent / ".recovery_pending"
            notice = (
                None
                if self.args.preflight_only
                else "上次控制会话以故障/电子急停结束，未记录到成功恢复。"
                if self.pending.exists()
                else recovery_notice(self.history)
            )
            if notice:
                self.ui.warn(notice)
                if sys.stdin.isatty():
                    self.ui.event(
                        "请人工确认机械臂已支撑、无电子急停。按 Enter 继续，Ctrl+C 退出。"
                    )
                    try:
                        input()
                    except EOFError:
                        raise RuntimeError("未确认恢复条件，退出")
                    self.result["operator_confirmed_after_recovery_warning"] = True
                else:
                    self.ui.warn("非交互运行：不等待确认，请自行核实")

    def resume_after_warning(self, s):
        "Perform the existing one-reset recovery after operator confirmation."
        if (
            self.result.get("operator_confirmed_after_recovery_warning")
            and self.history
            and self.history.get("requires_recovery")
        ):
            self.backend.reset_once()
            time.sleep(1.0)
            s = self.backend.snapshot()
            if (
                np.any(s["enabled"])
                or s["status"].get("arm_status") != 0
                or s["status"].get("error_code") != 0
            ):
                raise RuntimeError(
                    "恢复未确认成功：六轴仍使能或故障码未清零；不自动重试，请断电重启后再运行"
                )
            (self.path.parent / ".recovery_pending").unlink(missing_ok=True)
            self.emit("RECOVERED：已按确认发送一次重置，六轴失能、故障码为零；继续本次任务。")
        return s

    def explicit_recovery(self, s):
        "Run the explicit supported recovery without enabling motion."
        if "S-V1.7-3250618" not in self.info["firmware"]:
            raise ValueError("Recovery not reviewed for this firmware")
        if np.max(np.abs(s["qd"])) > 0.1:
            raise ValueError("Robot moving: recovery refused")
        self.result["before"] = s
        self.emit("RECOVER：已明确确认机械臂蜷缩有支撑；仅发送一次恢复/重置，不使能、不发送目标。")
        self.backend.reset_supported()
        time.sleep(1.0)
        after = self.backend.snapshot()
        self.result["after"] = after
        if (
            np.any(after["enabled"])
            or after["status"].get("arm_status") != 0
            or after["status"].get("error_code") != 0
            or any(after["faults"])
        ):
            raise RuntimeError("恢复未确认成功；不自动重试。")
        (self.path.parent / ".recovery_pending").unlink(missing_ok=True)
        self.emit("RECOVERED：六轴已失能、故障码为零。恢复命令结束；运动需要另行运行并由手柄触发。")
        self.result.update(completed=True, state="RECOVERED")

    def prepare_hardware(self):
        "Discover, check and preflight hardware before creating the controller."
        Hardware = self.hardware_type()
        self.acquire_channel()
        self.review_previous_run()
        self.backend = Hardware(self.config, self.path)
        self.info = self.backend.discover()
        (self.path / "hardware.json").write_text(
            json.dumps(self.info, ensure_ascii=False, indent=2)
        )
        self.emit(
            "只读查询完成："
            + next(
                (x for x in self.info["firmware"].split("\x00") if x.startswith("S-V")), "未知固件"
            )
            + "；详细参数已保存 hardware.json"
        )
        s = self.backend.snapshot()
        self.backend.check(s, False)
        s = self.resume_after_warning(s)
        if self.args.recover:
            self.explicit_recovery(s)
            return False
        if self.args.read_only:
            self.samples.write(json.dumps(s, default=serializable) + "\n")
            self.result.update(completed=True, state="READ_ONLY")
            return
        self.backend.preflight(self.info)
        (self.path / "effective_config.json").write_text(
            json.dumps(self.config, ensure_ascii=False, indent=2)
        )
        return True

    def prepare_inputs(self):
        "Open operator input and optional simulation viewer."
        if not self.args.scripted and (not self.args.preflight_only) and (not self.args.auto):
            self.pad = Gamepad(self.args.pad)
        if self.args.viewer:
            if not self.args.sim:
                raise ValueError("viewer is for simulation only")
            import mujoco.viewer

            self.viewer = mujoco.viewer.launch_passive(self.backend.model, self.backend.data)
            self.viewer.cam.distance = 1.3
            self.viewer.cam.lookat[:] = [0, 0, 0.3]
            self.viewer.cam.azimuth = 135
            self.viewer.cam.elevation = -25

    def prepare_controller(self):
        "Construct the unchanged controller variant and initial snapshot."
        s = self.backend.snapshot()
        now = 0.0 if self.args.sim else time.monotonic()
        ctrl_type = Controller
        if self.selected in ("mit", "drag"):
            from piper_dynamics.real_robot.control.mit_trial import MitController

            ctrl_type = MitController
        elif self.selected == "gravity":
            from piper_dynamics.real_robot.control.gravity_hold import GravityController

            ctrl_type = GravityController
        self.controller = ctrl_type(self.backend, self.config, s, now, self.ui.event)
        self.controller.ui = self.ui
        return s

    def save_reference(self):
        "Save the existing reference plan and trajectory metadata."
        if self.selected in ("gravity", "mit", "drag"):
            (self.path / "reference_plan.json").write_text(
                json.dumps(self.controller.reference_plan, ensure_ascii=False, indent=2)
            )
        if self.controller.reference_plan and self.controller.multisine:
            from piper_dynamics.real_robot.trials.multisine import save_reference

            save_reference(self.path, self.controller.test, self.controller.reference_plan)
            limits = self.config.get("reported_motor_max_acceleration_rad_s2")
            if limits and np.any(
                np.array(self.controller.reference_plan["max_acceleration_per_joint_rad_s2"])
                > limits
            ):
                self.emit(
                    "FIRMWARE_PLANNER_WARNING：参考加速度高于固件返回的规划配置；仅提示，固件可能平滑/限速。不会修改固件配置。"
                )
        if self.controller.trial:
            self.controller.trial.save(self.path)

    def save_session_policy(self):
        "Persist startup/return policy without changing its historical fields."
        self.result["home_rad"] = self.controller.home.tolist()
        self.result["return_home_rad"] = self.controller.return_home.tolist()
        self.result["current_pose_only"] = self.args.from_current
        self.result["standing_start"] = self.args.standing
        (self.path / "session_policy.json").write_text(
            json.dumps(
                {
                    "current_pose_only": self.args.from_current,
                    "home_deg": np.rad2deg(self.controller.home).tolist(),
                    "return_home_deg": np.rad2deg(self.controller.return_home).tolist(),
                    "return_home_delta_deg": np.rad2deg(
                        self.controller.return_home - self.controller.home
                    ).tolist(),
                    "measurement_bounds_reference": "firmware/config intersection expanded outward; preparation/return also accept startup corridor",
                    "measurement_angle_tolerance_deg": self.config[
                        "measurement_angle_tolerance_deg"
                    ],
                    "test_measurement_lower_deg": (
                        np.asarray(self.config["lower_deg"])
                        - self.config["measurement_angle_tolerance_deg"]
                    ).tolist(),
                    "test_measurement_upper_deg": (
                        np.asarray(self.config["upper_deg"])
                        + self.config["measurement_angle_tolerance_deg"]
                    ).tolist(),
                    "measurement_lower_deg": np.rad2deg(
                        self.controller.measurement_bounds()[0]
                    ).tolist(),
                    "measurement_upper_deg": np.rad2deg(
                        self.controller.measurement_bounds()[1]
                    ).tolist(),
                    "command_lower_deg": np.rad2deg(self.controller.lower).tolist(),
                    "command_upper_deg": np.rad2deg(self.controller.upper).tolist(),
                    "held_command_joints": [1, 2, 3, 4, 5] if self.args.from_current else [],
                    "moving_joint": self.config["joint"]
                    if self.config["trajectory"] == "pilot"
                    else None,
                    "moving_joints": [1, 2, 3, 4, 5, 6]
                    if self.config["trajectory"] in ("multisine", "mit", "drag", "gravity")
                    else [self.config["joint"]],
                    "shutdown": "park_then_confirm_six_axis_disabled",
                    "park_release_tolerance_deg": self.config["park_release_tolerance_deg"],
                    "disable_confirm_s": self.config["disable_confirm_s"],
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        if self.controller.trial:
            policy = json.loads((self.path / "session_policy.json").read_text())
            policy.update(
                controlled_joints=self.controller.trial.active_joints,
                passive_joints=self.controller.trial.passive_joints,
                moving_joints=[1, 2, 3, 4, 5]
                if self.config.get("hold_start_wrist")
                else self.controller.trial.active_joints,
                gripper_control=self.controller.trial.plan["gripper_policy"],
                fixed_at_startup_joints=[6] if self.config.get("hold_start_wrist") else [],
                return_pose_policy="J1–J5 park; J6 holds startup angle"
                if self.config.get("hold_start_wrist")
                else "all six joints park",
            )
            (self.path / "session_policy.json").write_text(
                json.dumps(policy, ensure_ascii=False, indent=2)
            )

    def read_action(self, now):
        "Read input and select scripted/automatic actions in the original priority order."
        if self.viewer and (not self.viewer.is_running()) and (not self.viewer_closed):
            self.viewer_closed = True
            self.controller.request_stop(self.backend.snapshot(), now, "Viewer closed")
        action = None
        if self.pad and (not self.input_failed):
            try:
                action = self.pad.poll(time.monotonic())
            except OSError as e:
                self.input_failed = True
                action = "STOP"
                self.emit("INPUT_STOP：" + str(e))
        if self.args.auto:
            if self.step == 10:
                action = "PREPARE"
            elif self.controller.state == "READY" and (not self.scripted_started):
                action = "START"
                self.scripted_started = True
        if self.args.scripted:
            if self.step == 10:
                action = "PREPARE"
            elif self.controller.state == "READY" and (not self.scripted_started):
                action = "START"
                self.scripted_started = True
            elif (
                self.args.scripted == "normal"
                and self.controller.trial
                and (self.controller.state == "TESTING")
            ):
                action = self.controller.trial.scripted_action()
            elif (
                self.args.scripted == "abort"
                and self.controller.state == "TESTING"
                and (now - self.controller.path_t >= 5.2)
                and (not self.scripted_aborted)
            ):
                action = "ABORT"
                self.scripted_aborted = True
        if self.controller.shutdown_phase and action in ("PREPARE", "START"):
            action = None
        if action:
            self.ui.event(("自动：" if self.args.auto else "手柄：") + action)
        return action

    def sample_feedback(self):
        "Advance simulation or read hardware and preserve feedback annotations."
        if self.args.sim:
            self.backend.advance(self.dt)
        s = self.backend.snapshot()
        dropped = s.get("can_rx_dropped_total", 0)
        if dropped != self.last_rx_dropped:
            self.emit(
                f"CAN_RX_WARNING：接收累计丢帧{dropped}；记录数据缺口，接收继续。最新反馈超时仍会停止。"
            )
            self.last_rx_dropped = dropped
        now = s["time"] if self.args.sim else time.monotonic()
        if (
            self.args.scripted == "speed-stop"
            and self.controller.state == "TESTING"
            and (5.2 <= now - self.controller.path_t < 5.5)
        ):
            s["qd"] = s["qd"].copy()
            s["qd"][self.config["joint"] - 1] = self.config["feedback_speed_stop_rad_s"] + 0.05
        return (s, now)

    def tick_controller(self, s, now, action):
        "Check the loop watchdog and execute exactly one controller tick."
        self.operator_exit.update(self.controller, s, now)
        if time.monotonic() - self.start_wall > self.args.max_seconds and (
            not self.timeout_handled
        ):
            self.timeout_handled = True
            self.controller.request_stop(s, now, "Session timeout")
        loop_gap = time.monotonic() - self.last_wall
        if (
            not self.args.sim
            and self.controller.active
            and (loop_gap > self.config["loop_watchdog_s"])
        ):
            self.controller.fault(
                f"Control loop late: gap={loop_gap * 1000:.1f} ms, threshold={self.config['loop_watchdog_s'] * 1000:.1f} ms, previous_state={self.last_tick_state}, previous_tick={self.last_tick_duration * 1000:.1f} ms"
            )
        self.last_wall = time.monotonic()
        tick_started = self.last_wall
        self.last_tick_state = self.controller.state
        self.controller.tick(now, s, action)
        self.last_tick_duration = time.monotonic() - tick_started
        return loop_gap

    def report_progress(self, now):
        "Report trial milestones and feedforward ramps."
        if (
            self.controller.state == "TESTING"
            and self.controller.trial
            and (getattr(self.controller.trial, "kind", None) == "friction")
        ):
            lab = self.controller.trial.reference.labels[max(self.controller.trial.last_index, 0)]
            if lab["phase"] == "forward":
                key = (lab["joint"], lab["speed"], lab["repeat"])
                if key != self.last_friction_key:
                    self.last_friction_key = key
                    self.ui.status(
                        f"摩擦采集 J{lab['joint']} {lab['speed']:g} rad/s 第{lab['repeat']}次 正向"
                    )
        elif (
            self.controller.state == "TESTING"
            and getattr(self.controller.test, "ends", None) is not None
        ):
            idx = min(
                int(
                    np.searchsorted(
                        self.controller.test.ends,
                        max(0.0, now - self.controller.path_t),
                        side="right",
                    )
                ),
                len(self.controller.test.parts) - 1,
            )
            if idx != self.last_step:
                self.last_step = idx
                self.ui.status(f"移动到期望位置 {idx + 1}/{len(self.controller.test.parts)}")
        else:
            self.last_step = -1
        if (
            self.controller.active
            and (not self.controller.shutdown_phase)
            and (self.controller.state in ("FF_RAMP_IN", "FF_RAMP_OUT"))
            and (now - self.last_status >= 1.0)
        ):
            self.last_status = now
            scale = float(self.config.get("mit_ff_scale", 0.0))
            if scale > 0:
                pct = 100 * self.controller.mit_ff_ratio / scale
                if 0.5 < pct < 99.5:
                    peak = (
                        float(np.max(np.abs(self.controller.mit_telemetry["ff_requested_Nm"])))
                        if self.controller.mit_telemetry
                        else 0.0
                    )
                    self.ui.status(
                        f"力矩渐变 {scale:.2f}  进度 {pct:.0f}%  当前最大 |FF| {peak:.2f} Nm"
                    )

    def record_feedback(self, s, now, action, loop_gap):
        "Record TESTING data and full-session diagnostics with existing failure handling."
        try:
            self.recorder.update(
                self.controller, now, s, now if self.args.sim else time.monotonic(), action
            )
        except OSError as e:
            self.log_failure = str(e)
            self.controller.request_stop(s, now, "Test data logging unavailable: " + str(e))
        tracking_target = (
            self.controller.last_target
            if not self.controller.shutdown_phase
            else np.full(6, np.nan)
        )
        self.record(
            {
                "t_control": now,
                "control_tick_wall_s": self.last_tick_duration,
                "control_loop_gap_s": loop_gap,
                "state": self.controller.state,
                "shutdown_phase": self.controller.shutdown_phase,
                "action": action,
                "q_target": tracking_target,
                "feedback": s,
            }
        )
        if self.log_failure:
            self.controller.request_stop(s, now, "Logging unavailable: " + self.log_failure)

    def update_viewer(self, s, now):
        "Refresh simulation text and display without changing control state."
        if self.viewer and self.viewer.is_running():
            j = self.config["joint"] - 1
            if self.controller.trial:
                tag = self.controller.trial.metadata(s, now)
                mass = "-" if tag["mass_kg"] is None else f"{tag['mass_kg']:.3f} kg"
                self.viewer.set_texts(
                    (
                        None,
                        None,
                        f"SIM ONLY | {self.config['trajectory']} | {self.controller.state} | {tag['phase']}\nPose {tag['pose_id']} | load {mass} | {('J6 passive' if self.config['passive_wrist'] else '6-axis control')}\nLB+X prepare | LB+Y start/next pose | LB+RB abort",
                        f"J2: {np.rad2deg(s['q'][1]):.2f} deg\nJ3: {np.rad2deg(s['q'][2]):.2f} deg",
                    )
                )
            else:
                self.viewer.set_texts(
                    (
                        None,
                        None,
                        "SIM ONLY | "
                        + self.config["trajectory"]
                        + " | scale "
                        + str(self.config["trajectory_scale"])
                        + " | speed "
                        + str(self.config["trajectory_speed_scale"])
                        + " | "
                        + self.controller.state
                        + "\nLB+X prepare | LB+Y start | LB+RB abort",
                        f"J{j + 1}: {np.rad2deg(s['q'][j]):.2f} deg\nTarget: {np.rad2deg(self.controller.last_target[j]):.2f} deg",
                    )
                )
            self.viewer.sync()

    def pace_loop(self):
        "Advance the step and preserve real-time versus scripted playback timing."
        self.step += 1
        if not self.args.scripted or self.args.viewer:
            self.deadline += self.dt
            remaining = self.deadline - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            else:
                self.deadline = time.monotonic()

    def run_loop(self):
        "Run the ordered acquisition/control/recording loop."
        self.recorder = TestRecorder(self.path, self.config, "sim" if self.args.sim else "hardware")
        self.dt = 1 / self.config["command_hz"]
        self.deadline = time.monotonic()
        self.start_wall = self.deadline
        self.step = 0
        self.last_wall = self.deadline
        self.scripted_started = False
        self.scripted_aborted = False
        self.timeout_handled = False
        self.input_failed = False
        self.viewer_closed = False
        self.last_rx_dropped = 0
        self.last_tick_duration = 0.0
        self.last_tick_state = self.controller.state
        self.last_status = -1000000000.0
        self.last_step = -1
        self.last_friction_key = None
        while not self.controller.finished:
            now = self.step * self.dt if self.args.sim else time.monotonic()
            action = self.read_action(now)
            s, now = self.sample_feedback()
            loop_gap = self.tick_controller(s, now, action)
            self.report_progress(now)
            self.record_feedback(s, now, action, loop_gap)
            self.update_viewer(s, now)
            self.pace_loop()
        self.result.update(
            completed=self.controller.state == "DONE",
            state=self.controller.state,
            aborted=self.controller.aborted,
            last_q=self.backend.snapshot()["q"].tolist(),
            max_requested_velocity=self.config["max_velocity_rad_s"],
            stop_reason=self.controller.stop_reason,
        )

    def close_recording(self):
        "Close the test window and capture diagnostic errors."
        if self.recorder:
            try:
                self.recorder.close(self.backend.snapshot()["time"], "loop_ended")
            except OSError as e:
                self.log_failure = str(e)
        if self.controller and self.controller.fault_reason:
            self.result["fault_reason"] = self.controller.fault_reason
        if self.log_failure:
            self.result["log_error"] = self.log_failure

    def wait_for_disabled(self):
        "On exceptional exit keep monitoring until the existing disarm contract is met."
        if (
            self.backend
            and self.controller
            and (not self.controller.finished)
            and (not self.args.preflight_only)
        ):
            if not self.controller.fault_reason:
                self.controller.fault("Unexpected loop termination")
            self.emit("SHUTDOWN_MONITOR：等待安全停放及六轴失能确认后，才关闭通信。")
            while not self.controller.finished:
                if self.args.sim:
                    self.backend.advance(0.01)
                observation = self.backend.snapshot()
                shutdown_now = observation["time"] if self.args.sim else time.monotonic()
                self.operator_exit.update(self.controller, observation, shutdown_now)
                self.controller.tick(shutdown_now, observation)
                self.record(
                    {
                        "t_control": shutdown_now,
                        "state": self.controller.state,
                        "shutdown_phase": self.controller.shutdown_phase,
                        "feedback": observation,
                    }
                )
                time.sleep(0.01)

    def record_outcome(self):
        "Save actual shutdown confirmation and final feedback."
        if self.controller and (not self.args.preflight_only):
            self.result.update(
                state=self.controller.state,
                completed=self.controller.state == "DONE"
                and self.controller.disarm_confirmed
                and (not self.log_failure),
                motors_disabled_confirmed=self.controller.disarm_confirmed,
                shutdown_phase=self.controller.shutdown_phase,
                operator_supported_disarm=self.controller.operator_supported_disarm,
                operator_power_off_confirmed=self.controller.operator_power_off_confirmed,
                release_q_rad=None
                if self.controller.release_q is None
                else self.controller.release_q.tolist(),
                final_enabled=self.backend.snapshot()["enabled"].tolist(),
                last_q=self.backend.snapshot()["q"].tolist(),
            )
            if self.controller.fault_reason:
                self.result["fault_reason"] = self.controller.fault_reason
            if self.log_failure:
                self.result["log_error"] = self.log_failure

    def close_viewer(self):
        "Keep the completed preview visible, then close it."
        if self.viewer and self.controller and self.controller.finished:
            self.emit("仿真已结束，窗口保持显示。关闭窗口即可退出。")
            view_deadline = time.monotonic() + 60
            while (
                self.viewer.is_running()
                and (not self.operator_exit.requested)
                and (time.monotonic() < view_deadline)
            ):
                self.viewer.sync()
                time.sleep(0.02)
        if self.viewer:
            self.viewer.close()

    def close_transport(self):
        "Close input and transport after shutdown monitoring."
        if self.pad:
            self.pad.close()
        if self.backend:
            if (
                not self.args.sim
                and self.controller
                and self.controller.operator_power_off_confirmed
            ):
                self.backend.close(allow_unconfirmed_power_off=True)
            else:
                self.backend.close()
            if not self.args.sim:
                self.result.update(
                    can_rx_dropped_total=self.backend.rx_dropped,
                    can_raw_log_dropped_total=self.backend.raw_log_dropped,
                    can_receive_buffer_bytes=self.backend.receive_buffer_bytes,
                )
                if self.backend.error and "logging" in self.backend.error:
                    self.log_failure = self.backend.error
                    self.result.update(completed=False, log_error=self.log_failure)

    def export_results(self):
        "Export trial datasets and summaries with their existing field names."
        if self.recorder:
            try:
                self.result["test_data"] = export_test_data(self.path)
                if (
                    self.controller
                    and self.controller.trial
                    and (self.controller.trial.kind == "friction")
                ):
                    plan = self.controller.trial.plan
                    self.result["friction_protocol"] = {
                        "kind": "friction",
                        "joints": plan["joint_order"],
                        "speeds_rad_s": plan["speeds_rad_s"],
                        "repeats": plan["repeats"],
                        "segments": plan["segments"],
                        "planned_duration_s": plan["duration_s"],
                        "automatic_sequence_completed": self.controller.trial.completed,
                        "torque_calibrated": False,
                    }
                    self.emit(
                        "FRICTION_SAVED："
                        + str(plan["segments"])
                        + "段，计划"
                        + f"{plan['duration_s']:.0f}"
                        + "秒；标签（关节／速度档／方向／重复）已写入 test_signals.npz，详见 reference_plan.json。"
                    )
                elif self.controller and self.controller.trial:
                    from piper_dynamics.real_robot.trials.validation import summarize

                    report = summarize(self.path)
                    self.result["validation_protocol"] = {
                        "kind": self.controller.trial.kind,
                        "automatic_sequence_completed": self.controller.trial.phase == "complete",
                        "complete_captures": sum((x["complete"] for x in report["captures"])),
                        "partial_captures": sum((not x["complete"] for x in report["captures"])),
                        "passive_joints": self.controller.trial.passive_joints,
                        "torque_calibrated": False,
                    }
                    self.emit(
                        "VALIDATION_SAVED："
                        + str(sum((x["complete"] for x in report["captures"])))
                        + "段完整静态采集（另含"
                        + str(sum((not x["complete"] for x in report["captures"])))
                        + "段中断记录）；标签／姿态／温度和夹爪信息已保存，详见validation_summary.json。"
                    )
                csv_note = (
                    "test_signals.csv / " if (self.path / "test_signals.csv").exists() else ""
                )
                self.emit(
                    "DATA_SAVED："
                    + str(self.result["test_data"]["sample_count"])
                    + " 条测试快照；"
                    + csv_note
                    + "test_signals.npz，详见 test_quality.json。"
                )
            except Exception as e:
                self.result["data_export_error"] = str(e)
                self.result["completed"] = False
                self.emit("DATA_EXPORT_FAILED：" + str(e))

    def release_resources(self):
        "Release channel/signal resources and write the session result."
        if self.lease:
            self.lease.close()
        for s, h in self.old_handlers.items():
            signal.signal(s, h)
        if self.result.get("state") == "FAULT" and not self.args.sim:
            (self.path.parent / ".recovery_pending").write_text(str(self.path) + "\n")
        elif self.result.get("state") == "DONE" and not self.args.sim:
            (self.path.parent / ".recovery_pending").unlink(missing_ok=True)
        self.samples.close()
        self.events.close()
        (self.path / "result.json").write_text(
            json.dumps(self.result, ensure_ascii=False, indent=2, default=serializable)
        )
        print("Run files:", self.path, flush=True)

    def handle_error(self, e):
        "Record a failure and preserve the original controller/transport stop behavior."
        self.result["error"] = str(e)
        self.emit("STOPPED：" + str(e))
        if hasattr(e, "reference_report"):
            (self.path / "reference_plan.json").write_text(
                json.dumps(e.reference_report, ensure_ascii=False, indent=2) + "\n"
            )
        if self.controller:
            self.controller.fault(e)
        if self.backend and self.backend.active:
            self.backend.quick_stop(str(e))
        self.result["state"] = "FAULT" if self.controller else "PREFLIGHT_REJECTED"

    def finish(self):
        "Complete shutdown, export and resource release in the original order."
        self.close_recording()
        self.wait_for_disabled()
        self.record_outcome()
        self.close_viewer()
        self.close_transport()
        self.export_results()
        self.release_resources()

    def run(self):
        "Own the session lifecycle; all returns still pass through shutdown handling."
        try:
            if not self.prepare_backend():
                return
            self.prepare_inputs()
            s = self.prepare_controller()
            self.save_reference()
            self.save_session_policy()
            if self.args.preflight_only:
                self.samples.write(
                    json.dumps(
                        {"feedback": s, "home_rad": self.controller.home}, default=serializable
                    )
                    + "\n"
                )
                self.result.update(completed=True, state="PREFLIGHT_ONLY")
                return
            self.run_loop()
        except Exception as error:
            self.handle_error(error)
        finally:
            self.finish()
        if (
            self.result.get("state") in ("FAULT", "PREFLIGHT_REJECTED")
            or self.result.get("log_error")
            or self.result.get("data_export_error")
        ):
            raise SystemExit(2)


def main(argv=None):
    """Parse and validate an experiment before entering its session lifecycle."""
    parser = build_parser()
    args = parser.parse_args(argv)
    validate_arguments(args, parser)
    if handle_input_only(args):
        return
    config = build_config(args, parser)
    Session(args, config).run()


if __name__ == "__main__":
    main()

__all__ = ["Session", "main", "validate_config", "Path", "serializable"]
