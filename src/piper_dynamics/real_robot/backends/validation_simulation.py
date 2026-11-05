"""Passive-wrist motion preview for the three diagnostic entry points only."""

import numpy as np

from piper_dynamics.real_robot.backends.simulation import Simulation


class ValidationSimulation(Simulation):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.model.actuator_gainprm[5, :] = 0
        self.model.actuator_biasprm[5, :] = 0
        # Illustrative passive dry friction, not a holding servo or identified
        # hardware value. Without any stiction the simplified geared wrist
        # creeps for minutes under tiny nominal COM asymmetry.
        self.model.dof_frictionloss[5] = cfg.get("simulation_passive_wrist_friction_Nm", 0.01)
        self.gravity_data = self.mj.MjData(self.model)

    def advance(self, dt):
        for _ in range(round(dt / self.model.opt.timestep)):
            if self.enabled or self.released:
                control = (
                    self.native_path.at(self.time - self.native_time)[0]
                    if self.native_path
                    else self.target
                )
                self.data.ctrl[:] = control
                self.data.qfrc_applied[:] = 0
                if self.enabled and not self.released and not self.stopped:
                    # Preserve the existing ideal gravity-compensated J1–J5
                    # servo preview, while restoring gravity on passive J6.
                    self.gravity_data.qpos[:] = self.data.qpos
                    self.gravity_data.qvel[:] = 0
                    self.mj.mj_forward(self.model, self.gravity_data)
                    self.data.qfrc_applied[5] = -self.gravity_data.qfrc_bias[5]
                self.mj.mj_step(self.model, self.data)
            self.time += self.model.opt.timestep

    def snapshot(self):
        s = super().snapshot()
        s["enabled"] = np.array([self.enabled] * 5 + [False])
        s["gripper"] = {
            "gripper_position_raw": 0,
            "status": 0,
            "simulation_fixed_visual_only": True,
        }
        s["torque_source"] = (
            "diagnostic motion preview only; no physical load/current calibration evidence"
        )
        return s
