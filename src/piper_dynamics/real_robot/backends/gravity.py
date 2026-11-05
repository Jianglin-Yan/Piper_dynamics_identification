"""Existing gravity-hold preview transports; hardware entry remains retired."""

import time

import numpy as np

from piper_dynamics.core.torque import NM_PER_A
from piper_dynamics.real_robot.backends.hardware import Hardware
from piper_dynamics.real_robot.backends.simulation import Simulation
from piper_dynamics.real_robot.protocol.mit import encode_mit


class GravityHardware(Hardware):
    def command(self, q):
        if getattr(self, "mit_active", False):
            self.position_started = time.monotonic()
        self.mit_active = False
        super().command(q)

    def exit_mit(self, q):
        self.command(q)

    def command_mit(self, q, torque):
        frames = [encode_mit(j + 1, float(q[j]), float(torque[j]))[0] for j in range(6)]
        with self.tx_lock:
            if self.stopped or self.motion_closed:
                raise RuntimeError("MIT command after stop/disable")
            if not getattr(self, "mit_active", False):
                self.mit_started = time.monotonic()
            self.mit_active = True
            self._send_locked(0x151, [1, 4, 0, 0xAD, 0, 0, 0, 0])
            for j, frame in enumerate(frames):
                self._send_locked(0x15A + j, frame)

    def check(self, s, active):
        super().check(s, active)
        if (
            active
            and getattr(self, "mit_active", False)
            and time.monotonic() - self.mit_started > 1.0
        ):
            if s["status"].get("motion_mode") != 4:
                raise RuntimeError("MIT mode not confirmed by firmware feedback")
        if (
            active
            and not getattr(self, "mit_active", False)
            and hasattr(self, "position_started")
            and time.monotonic() - self.position_started > 1.0
        ):
            if s["status"].get("motion_mode") != 1:
                raise RuntimeError("MOVE J return mode not confirmed by firmware feedback")


class GravitySimulation(Simulation):
    def __init__(self, c):
        super().__init__(c)
        self.mit_active = False
        self.mit_output = np.zeros(6)
        self.position_gain = self.model.actuator_gainprm.copy()
        self.position_bias = self.model.actuator_biasprm.copy()
        self.position_gravcomp = self.model.body_gravcomp.copy()

    def exit_mit(self, q):
        self.data.qfrc_applied[:] = 0.0
        self.mit_output[:] = 0.0
        self.mit_active = False
        self.model.actuator_gainprm[:] = self.position_gain
        self.model.actuator_biasprm[:] = self.position_bias
        self.model.body_gravcomp[:] = self.position_gravcomp
        super().command(q)

    def command_mit(self, q, torque):
        if self.stopped or self.motion_closed:
            raise RuntimeError("MIT command after stop/disable")
        self.mit_active = True
        self.model.body_gravcomp[:] = 0.0
        self.model.actuator_gainprm[:] = 0.0
        self.model.actuator_biasprm[:] = 0.0
        self.mit_output = np.array(
            [encode_mit(j + 1, float(q[j]), float(torque[j]))[2] for j in range(6)]
        )
        self.data.qfrc_applied[:] = self.mit_output

    def snapshot(self):
        s = super().snapshot()
        s["status"].update(control_mode=1, motion_mode=4 if self.mit_active else 1)
        if self.mit_active:
            s.update(
                tau_candidate_Nm=self.mit_output.copy(),
                current_A=self.mit_output / NM_PER_A,
                torque_source="simulation, explicit gravity and 8-bit MIT torque quantization; not hardware evidence",
            )
        return s

    def disable(self):
        self.data.qfrc_applied[:] = 0.0
        self.mit_output[:] = 0.0
        super().disable()

    def quick_stop(self, reason):
        self.data.qfrc_applied[:] = 0.0
        self.mit_output[:] = 0.0
        super().quick_stop(reason)
