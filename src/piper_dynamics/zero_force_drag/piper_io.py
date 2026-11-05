"""Narrow adapter for the official PiPER SDK V2/V3 MIT interface."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .controller import DragCommand


@dataclass(frozen=True)
class Feedback:
    q: np.ndarray
    qd: np.ndarray
    received_at: float


class PiperMITIO:
    def __init__(self, can_name: str, protocol: str = "v3"):
        try:
            from piper_sdk import C_PiperInterface_V2, C_PiperInterface_V3
        except ImportError as exc:
            raise RuntimeError("live control requires the official piper_sdk package") from exc
        if protocol not in ("v2", "v3"):
            raise ValueError("protocol must be v2 or v3")
        self.protocol = protocol
        self.arm = (C_PiperInterface_V3 if protocol == "v3" else C_PiperInterface_V2)(can_name)
        self.arm.ConnectPort()
        self._last_joint_stamp = 0.0
        self._last_speed_stamp = 0.0
        self._joint_update = 0.0
        self._speed_update = 0.0

    def read(self, max_age_s: float) -> Feedback:
        joint = self.arm.GetArmJointMsgs()
        speed = self.arm.GetArmHighSpdInfoMsgs()
        now = time.monotonic()
        if joint.time_stamp <= 0 or speed.time_stamp <= 0:
            raise RuntimeError("waiting for joint and motor feedback")
        if joint.time_stamp != self._last_joint_stamp:
            self._joint_update = now
            self._last_joint_stamp = joint.time_stamp
        if speed.time_stamp != self._last_speed_stamp:
            self._speed_update = now
            self._last_speed_stamp = speed.time_stamp
        if (not self._joint_update or not self._speed_update or
                now - self._joint_update > max_age_s or now - self._speed_update > max_age_s):
            raise RuntimeError("joint or motor feedback is stale")
        status = self.arm.GetArmStatus()
        if status.time_stamp <= 0 or int(status.arm_status.arm_status) != 0:
            raise RuntimeError("arm status is unavailable or reports a fault")
        q = np.deg2rad(np.array([getattr(joint.joint_state, f"joint_{i}") for i in range(1, 7)], dtype=float) * 0.001)
        qd = np.array([getattr(speed, f"motor_{i}").motor_speed for i in range(1, 7)], dtype=float) * 0.001
        if not np.all(np.isfinite(q)) or not np.all(np.isfinite(qd)):
            raise RuntimeError("invalid SDK feedback")
        return Feedback(q, qd, now)

    def enter_mit(self) -> None:
        # V3 (firmware >= 1.8-8) moved MOVE M from 0x04 to 0x06.
        move_m = 0x06 if self.protocol == "v3" else 0x04
        self.arm.MotionCtrl_2(ctrl_mode=0x01, move_mode=move_m, is_mit_mode=0xAD)

    def send(self, command: DragCommand) -> None:
        for i in range(6):
            self.arm.JointMitCtrl(i + 1, float(command.position_rad[i]),
                                  float(command.velocity_rad_s[i]),
                                  float(command.kp[i]), float(command.kd[i]),
                                  float(command.torque_nm[i]))

    def stop(self) -> None:
        # Stop motion without silently changing to a holding mode or sending
        # a stale torque. The robot must be physically supported on exit.
        self.arm.MotionCtrl_1(emergency_stop=0x01)
