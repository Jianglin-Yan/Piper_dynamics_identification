"""MIT command/ramp policy; the common controller owns lifecycle and limits."""

import numpy as np

from piper_dynamics.real_robot.trials.trajectory import smooth


class MitControl:
    def _mit_command(self, q):
        """按当前阶段发送六轴MIT命令，并记录前馈、PD和协议量化候选值。

        使用当前命令范围及持续超预算规则；估计速度限幅不改实际反馈或命令，力矩未标定。"""
        from piper_dynamics.core.torque import WIRE_TO_JOINT
        from piper_dynamics.real_robot.protocol.mit import encode_mit

        q = np.asarray(q, float)
        s = self.mit_sample
        v = self.reference_velocity(self.mit_time)
        if q.shape != (6,):
            raise ValueError("MIT reference must be six-dimensional")
        outside = (q < self.lower - 1e-9) | (q > self.upper + 1e-9)
        if np.any(outside):
            j = int(np.argmax(outside))
            raise ValueError(
                f"MIT reference outside command bounds: J{j + 1} "
                f"{np.rad2deg(q[j]):.3f} deg outside "
                f"[{np.rad2deg(self.lower[j]):.3f}, {np.rad2deg(self.upper[j]):.3f}] deg"
            )
        g = np.zeros(6) if self.gravity is None or self.mit_ff_ratio == 0 else self.gravity(s["q"])
        ff = self.mit_ff_ratio * g
        vlim = float(self.config["mit_estimate_velocity_limit_rad_s"])
        qd = np.clip(s["qd"], -vlim, vlim)
        pd = (self.mit_kp * (q - s["q"]) + self.mit_kd * (v - qd)) * WIRE_TO_JOINT
        est = pd + ff
        caps = np.asarray(self.config["mit_joint_torque_estimate_caps_Nm"], float)
        over = np.abs(est) > caps
        if np.any(over):
            self.budget_cycles += 1
            j = int(np.argmax(np.abs(est) / caps))
            if self.budget_cycles == 1:
                self.event(
                    f"BUDGET_WARNING：J{j + 1} 力矩估计 {est[j]:+.2f} Nm 超过 {caps[j]:.1f} Nm；连续 {self.config['mit_budget_exceed_duration_s']:.2f} s 才故障。"
                )
            if self.budget_cycles >= self.budget_limit:
                raise ValueError(
                    f"MIT PD+FF estimate over budget: J{j + 1} {est[j]:+.2f} Nm vs {caps[j]:.1f} Nm for {self.config['mit_budget_exceed_duration_s']:.2f} s"
                )
        else:
            self.budget_cycles = 0
        self.backend.command_pd(q, v, self.mit_kp, self.mit_kd, ff)
        self.last_target = q.copy()
        self.mit_telemetry = {
            "phase": self.state,
            "ff_ratio": self.mit_ff_ratio,
            "q_ref": q.copy(),
            "v_ref": v.copy(),
            "kp_protocol": self.mit_kp.copy(),
            "kd_protocol": self.mit_kd.copy(),
            "g_Nm": g,
            "ff_requested_Nm": ff,
            "wire_t_ref": ff / WIRE_TO_JOINT,
            "quantized_ff_Nm": np.array(
                [
                    encode_mit(
                        j + 1,
                        float(q[j]),
                        float(ff[j]),
                        float(v[j]),
                        float(self.mit_kp[j]),
                        float(self.mit_kd[j]),
                    )[2]
                    for j in range(6)
                ]
            ),
            "pd_estimate_Nm": pd,
            "total_estimate_Nm": est,
        }

    def _mit_tick_pre(self, now, s):
        if self.state == "FF_RAMP_IN":
            f = float(smooth((now - self.ff_phase_t) / self.config["mit_ff_ramp_s"])[0])
            self.mit_ff_ratio = self.config["mit_ff_scale"] * f
            self.mit_kp = (1 - f) * np.array(self.config["mit_prepare_kp"]) + f * np.array(
                self.config["mit_kp"]
            )
            self.mit_kd = (1 - f) * np.array(self.config["mit_prepare_kd"]) + f * np.array(
                self.config["mit_kd"]
            )
        if self.state == "TESTING" and (not self.drag) and now - self.path_t >= self.test.duration:
            self.request_stop(s, now, "MIT trial completed")
        if self.state == "FF_RAMP_OUT":
            f = float(smooth((now - self.ff_phase_t) / self.config["mit_ff_ramp_s"])[0])
            self.mit_ff_ratio = (1 - f) * self.ff_exit_ratio
            self.mit_kp = (1 - f) * self.exit_kp + f * np.array(self.config["mit_return_kp"])
            self.mit_kd = (1 - f) * self.exit_kd + f * np.array(self.config["mit_return_kd"])

    def _mit_tick_post(self, now, s):
        try:
            if self.state == "RELEASING":
                f = float(smooth((now - self.release_t) / float(self.config["mit_release_s"]))[0])
                self.mit_kp = np.array(self.config["mit_kp"]) * (1 - f)
                self.mit_kd = np.array(self.config["mit_kd"]) * (1 - f)
                self.command(self.return_home)
                if f >= 1.0 - 1e-9:
                    self.begin_disarm(s, now)
            elif self.state == "FF_RAMP_IN":
                self.command(self.start)
                if now - self.ff_phase_t >= self.config["mit_ff_ramp_s"]:
                    self.mit_ff_ratio = self.config["mit_ff_scale"]
                    self.state = "READY"
                    if float(self.config["mit_ff_scale"]) > 0:
                        self.event(f"渐变完成：前馈已达{100 * self.mit_ff_ratio:.0f}%")
                    self.event("READY  到达准备位置")
            elif self.state == "FF_RAMP_OUT":
                self.command(self.last_target)
                if now - self.ff_phase_t >= self.config["mit_ff_ramp_s"]:
                    if float(self.config["mit_ff_scale"]) > 0:
                        self.event("渐变完成：前馈已归零")
                    self.mit_ff_ratio = 0.0
                    self.returning_mit = True
                    self._request_stop_base(s, now, self.stop_reason)
        except Exception as e:
            self.fault(e)

    def _mit_finish_settle(self, now, s):
        if self.state == "SETTLING_HOME":
            if self.position_stable(
                s, now, self.return_home, self.config["mit_park_tolerance_deg"]
            ):
                release = float(self.config.get("mit_release_s", 0.0))
                # The release assumes the park is physically supported; the simulator has
                # no support, so it keeps the previous direct-disarm behaviour.
                if release > 0 and getattr(self.backend, "supports_stiffness_release", True):
                    self.state = "RELEASING"
                    self.release_t = now
                    self.event(f"释放：{release:g}秒内缓慢撤去刚度，机械臂落到启动支撑位置")
                else:
                    self.begin_disarm(s, now)
            elif now - self.settle_started > self.config["settle_timeout_s"]:
                raise ValueError("Timed out settling at target")
            return
        if self.state != "SETTLING_READY":
            return self._finish_settle_base(now, s)
        self.command(self.start)
        if self.position_stable(s, now, self.start, self.config["mit_prepare_tolerance_deg"]):
            self.state = "FF_RAMP_IN"
            self.ff_phase_t = now
            self.ff_print_t = now
            scale = float(self.config["mit_ff_scale"])
            if scale <= 0:
                self.event("渐变：纯PD（0%），机械臂静止3秒")
            else:
                self.event(
                    f"渐变：前馈 0→{100 * scale:.0f}%，用时{self.config['mit_ff_ramp_s']:g}秒，期间机械臂静止"
                )
        elif now - self.settle_started > self.config["settle_timeout_s"]:
            raise ValueError("MIT preparation failed to settle; no gain auto-escalation")

    def _mit_request_stop(self, s, now, reason):
        self.now = now
        if (
            self.finished
            or self.shutdown_phase
            or self.state in ("BRAKING", "RETURNING", "SETTLING_HOME")
        ):
            return
        if not self.active:
            return self._request_stop_base(s, now, reason)
        if reason in ("MIT trial completed", "Drag test completed"):
            self.stop_reason = reason
            self.state = "FF_RAMP_OUT"
            self.ff_phase_t = now
            self.ff_exit_ratio = self.mit_ff_ratio
            self.exit_kp = self.mit_kp.copy()
            self.exit_kd = self.mit_kd.copy()
            self.ff_print_t = now
            if float(self.config["mit_ff_scale"]) <= 0:
                self.event("渐变：纯PD（0%），机械臂静止3秒")
            else:
                self.event(
                    f"渐变：前馈 {100 * self.mit_ff_ratio:.0f}→0%，用时{self.config['mit_ff_ramp_s']:g}秒，期间机械臂静止"
                )
            return
        if self.returning_mit:
            return
        if self.state == "ARMING" and not self.motors_ready(s):
            self.mit_ff_ratio = 0.0
            self.begin_disarm(s, now)
            return
        try:
            from piper_dynamics.real_robot.control.impedance import bumpless_reference

            self.backend.check(s, True)
            if not self.motors_ready(s):
                raise ValueError("MIT not active; no forced return")
            old = self.mit_telemetry
            kp = np.array(self.config["mit_return_kp"])
            kd = np.array(self.config["mit_return_kd"])
            if old is None:
                q = s["q"].copy()
                v = np.zeros(6)
            else:
                q, v = bumpless_reference(
                    s["q"],
                    s["qd"],
                    old["q_ref"],
                    old["v_ref"],
                    old["kp_protocol"],
                    old["kd_protocol"],
                    old["wire_t_ref"],
                    kp,
                    kd,
                )
            bounded = np.clip(q, self.lower, self.upper)
            if np.max(abs(q - bounded)) > 1e-8:
                self.event(
                    "RETURN_REFERENCE_WARNING：无扰参考触及规划边界，已限制；切换不保证完全连续。"
                )
            self.mit_ff_ratio = 0.0
            self.mit_kp = kp
            self.mit_kd = kd
            self.stop_velocity = v
            self.command(bounded)
            self.returning_mit = True
            self.event(
                "MIT_ABORT_RETURN：前馈请求立即清零；统一PD增益接管，以无扰参考制动，MIT回位失能。"
            )
            self._request_stop_base(s, now, reason)
        except Exception as e:
            self.fault("MIT controlled return unavailable: " + str(e))
        finally:
            if hasattr(self, "stop_velocity"):
                del self.stop_velocity
