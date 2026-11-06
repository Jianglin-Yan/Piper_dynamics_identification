"""Three position-mode diagnostic trials. No identified-model torque commands.

Timed phases share the existing prepare/stop/park/disable controller. In these
payload trials J6 holds its startup angle; all trials enable all six axes.
"""

import hashlib
import json
from pathlib import Path

import numpy as np

from piper_dynamics.paths import robot_source_path
from piper_dynamics.real_robot.trials.trajectory import Segment, check_path, transit

TRIALS = ("static", "reciprocal", "payload")
LOADS_KG = (0.0, 0.09, 0.245, 0.335)
STATIC_POSES_DEG = np.array(
    [
        [0, 45, -60, 0, 0, 0],
        [0, 75, -125, 0, 35, 0],
        [0, 25, -35, 0, -20, 0],
        [0, 100, -65, 0, -35, 0],
        [-50, 50, -90, -35, 40, -45],
        [50, 50, -90, 35, -40, 45],
        [0, 65, -100, 55, 50, 75],
        [0, 40, -75, -55, -50, -75],
    ],
    float,
)
RECIPROCAL_POSES_DEG = np.array(
    [
        [0, 45, -60, 0, 0, 0],
        [0, 30, -65, 0, 15, 20],
        [0, 60, -80, 0, -15, -25],
        [15, 45, -75, 15, 15, 30],
    ],
    float,
)
PAYLOAD_POSES_DEG = np.array(
    [[0, 45, -60, 0, 0, 0], [0, 30, -65, 0, 15, 0], [0, 60, -80, 0, -15, 0]], float
)


def configure(cfg):
    payload = cfg["trajectory"] == "payload"
    cfg.update(
        passive_wrist=False,
        hold_start_wrist=payload,
        release_gripper=payload,
        feedback_stop_policy="warn",
        validation_move_velocity_rad_s=0.12 if cfg["trajectory"] == "static" else 0.08,
        validation_move_acceleration_rad_s2=0.12 if cfg["trajectory"] == "static" else 0.08,
        validation_settle_s=2.0,
        validation_capture_s=6.0,
        validation_settle_timeout_s=30.0,
        reciprocal_speeds_rad_s=[0.04, 0.12],
        payload_masses_kg=list(LOADS_KG),
        trajectory_scale=1.0,
        trajectory_speed_scale=1.0,
    )
    cfg.setdefault("payload_point_link6_m", None)


class Waypoints:
    def check_limits(self, lower, upper, vmax, amax):
        for segment in self.parts:
            check_path(segment, lower, upper, vmax, amax)

    def __init__(self, poses, vmax, amax, minimum=6.0):
        self.parts = []
        self.ends = []
        self.duration = 0.0
        for a, b in zip(poses[:-1], poses[1:]):
            p = transit(a, b, vmax, amax, minimum=minimum)
            self.parts.append(p)
            self.duration += p.duration
            self.ends.append(self.duration)

    def at(self, t):
        t = float(np.clip(t, 0, self.duration))
        i = min(int(np.searchsorted(self.ends, t, side="right")), len(self.parts) - 1)
        begin = 0.0 if i == 0 else self.ends[i - 1]
        return self.parts[i].at(t - begin)


class Reverse:
    def check_limits(self, lower, upper, vmax, amax):
        check_path(self.path, lower, upper, vmax, amax)

    def __init__(self, path):
        self.path = path
        self.duration = path.duration

    def at(self, t):
        q, v, a = self.path.at(self.duration - float(np.clip(t, 0, self.duration)))
        return q, -v, a


class RuntimeReference:
    """Executed reference timeline, including operator-dependent hold durations."""

    def __init__(self, start, duration):
        self.initial = np.asarray(start).copy()
        self.entries = []
        self.duration = float(duration)

    def append(self, t, path):
        self.entries.append((float(t), path))

    def at(self, t):
        if not self.entries:
            return self.initial.copy(), np.zeros(6), np.zeros(6)
        ix = max(0, int(np.searchsorted([x[0] for x in self.entries], float(t), side="right")) - 1)
        begin, path = self.entries[ix]
        return path.at(max(0.0, float(t) - begin))


class ValidationTrial:
    def __init__(self, start, cfg, lower, upper):
        self.c = cfg
        self.kind = cfg["trajectory"]
        self.vmax = cfg["validation_move_velocity_rad_s"]
        self.amax = cfg["validation_move_acceleration_rad_s2"]
        source = {
            "static": STATIC_POSES_DEG,
            "reciprocal": RECIPROCAL_POSES_DEG,
            "payload": PAYLOAD_POSES_DEG,
        }[self.kind]
        self.poses = np.deg2rad(source.copy())
        self.poses += np.asarray(start) - self.poses[0]
        if self.kind == "payload":
            self.poses[:, 5] = start[5]
        self.order = list(range(len(self.poses))) + ([0] if self.kind == "static" else [])
        speeds = cfg["reciprocal_speeds_rad_s"] if self.kind == "reciprocal" else [self.vmax]
        self.forwards = [Waypoints(self.poses, v, self.amax) for v in speeds]
        self.speed_round = 0
        self.forward = self.forwards[0]
        self.reverse = Reverse(self.forward)
        self.active_joints = [1, 2, 3, 4, 5] if cfg.get("passive_wrist") else [1, 2, 3, 4, 5, 6]
        self.passive_joints = [6] if cfg.get("passive_wrist") else []
        for f, v in zip(self.forwards, speeds):
            for segment in f.parts:
                check_path(segment, lower, upper, v, self.amax)
        self.hold_s = cfg["validation_capture_s"]
        self.settle_s = cfg["validation_settle_s"]
        if self.kind == "static":
            nominal = sum(
                transit(self.poses[a], self.poses[b], self.vmax, self.amax, minimum=6.0).duration
                for a, b in zip(self.order[:-1], self.order[1:])
            ) + len(self.order) * (self.hold_s + self.settle_s + 0.5)
        else:
            nominal = 2 * sum(f.duration for f in self.forwards) + 2 * len(self.forwards) * (
                self.hold_s + self.settle_s + 0.5
            )
        self.reference = RuntimeReference(start, nominal)
        self.plan = {
            "schema_version": 2,
            "trajectory_type": self.kind,
            "poses_deg": np.rad2deg(self.poses).tolist(),
            "pose_order": self.order,
            "controlled_joints": self.active_joints,
            "passive_joints": self.passive_joints,
            "fixed_at_startup_joints": [6] if cfg.get("hold_start_wrist") else [],
            "gripper_policy": "disable only; manually adjustable"
            if self.kind == "payload"
            else "unchanged; no gripper commands",
            "reciprocal_speeds_rad_s": speeds if self.kind == "reciprocal" else None,
            "move_velocity_bound_rad_s": max(speeds),
            "move_acceleration_bound_rad_s2": self.amax,
            "stable_settle_s": self.settle_s,
            "capture_s": self.hold_s,
            "payload_masses_kg": list(LOADS_KG) if self.kind == "payload" else None,
            "payload_point_link6_m": cfg.get("payload_point_link6_m"),
            "absolute_load_torque_calibrated": False,
            "payload_mass_label_policy": "unknown until manually annotated; no automatic assignment by time or current",
            "time_policy": "payload records continuously; LB+Y directly advances pose, not load stages",
            "reverse_policy": "q_reverse(t)=q_forward(T-t), v_reverse=-v_forward(T-t), a_reverse=a_forward(T-t)",
            "simulation_scope": "motion / interaction preview; current is zero, no synthetic torque calibration evidence",
        }
        self.phase = "not_started"
        self.pose_id = 0
        self.visit = 0
        self.mass_index = 0
        self.confirmed_mass = None
        self.direction = "none"
        self.phase_t = 0.0
        self.test_t = None
        self.stable_since = None
        self.capture_id = 0
        self.completed_windows = 0
        self.last_printed = None
        self.steady = False
        self.latest_gripper = None
        self.gripper_history = []

    def _hold(self, k, now):
        p = Segment(k.last_target, k.last_target, 1.0)
        self.reference.append(now - self.test_t, p)

    def _phase(self, k, phase, now, message):
        self.phase = phase
        self.phase_t = now
        self.stable_since = None
        self.steady = False
        if phase in ("moving", "settling"):
            k.pose_history = []
            self.gripper_history = []
        if message:
            k.event(message)

    def begin(self, k, now):
        self.test_t = now
        self._hold(k, now)
        if self.kind == "reciprocal":
            self._moving(k, now, self.forward, "forward", len(self.poses) - 1)
        else:
            self.pose_id = 0
            self.confirmed_mass = None
            self._phase(
                k,
                "settling",
                now,
                "POSE 1：开始持续记录，可自行挂载／更换；卸载后LB+Y直接切下一姿态，LB+RB结束。"
                if self.kind == "payload"
                else "POSE 1：等待实测静止，再采集6秒。",
            )

    def _moving(self, k, now, path, direction, goal_id):
        k.check(
            path,
            max(self.c["reciprocal_speeds_rad_s"]) if self.kind == "reciprocal" else self.vmax,
            self.amax,
        )
        self.path = path
        self.goal = path.at(path.duration)[0]
        self.pose_id = goal_id
        self.direction = direction
        self.reference.append(now - self.test_t, path)
        self._phase(
            k,
            "moving",
            now,
            f"{direction.upper()}：平滑移动到姿态{goal_id + 1}；"
            + ("J6保持启动角度，夹爪可手调。" if self.kind == "payload" else "六轴受控。")
            + (
                f" 第{self.speed_round + 1}档，上限{self.c['reciprocal_speeds_rad_s'][self.speed_round]:.2f}rad/s。"
                if self.kind == "reciprocal"
                else ""
            ),
        )

    def _settled(self, k, s, now):
        # All protocols require actual rest; payload gripper can be adjusted manually.
        stable = k.position_stable(
            s, now, self.goal if self.phase == "moving" else k.last_target, k.c["settle_error_deg"]
        )
        grip = s.get("gripper", {}).get("gripper_position_raw")
        self.gripper_history.append((now, float(s["q"][5]), grip))
        self.gripper_history = [x for x in self.gripper_history if x[0] >= now - 0.55]
        if len(self.gripper_history) < 2 or now - self.gripper_history[0][0] < 0.45:
            return False
        wrist = np.ptp([x[1] for x in self.gripper_history]) <= np.deg2rad(0.2)
        g = [x[2] for x in self.gripper_history if x[2] is not None]
        grip_ok = (
            not g or max(g) - min(g) <= 500
        )  # raw 0.001 mm, allows 0.5 mm quantization/motion.
        return bool(stable and wrist and grip_ok)

    def tick(self, k, now, s, action):
        if self.phase == "not_started":
            self.begin(k, now)
        if self.kind == "payload" and action == "START" and now - self.test_t > 0.02:
            if self.phase == "moving":
                k.event("正在切换姿态，本次LB+Y忽略；到位后再按。")
            else:
                self.visit += 1
                n = self.visit % len(self.poses)
                self.confirmed_mass = None
                self._moving(
                    k,
                    now,
                    transit(k.last_target, self.poses[n], self.vmax, self.amax, minimum=6.0),
                    "pose_change",
                    n,
                )
                return
        if self.phase == "moving":
            elapsed = now - self.phase_t
            k.command(self.path.at(elapsed)[0])
            if elapsed >= self.path.duration:
                self._hold(k, now)
                self._phase(
                    k,
                    "settling",
                    now,
                    f"姿态{self.pose_id + 1}到位：继续记录，可自行挂载／换载；LB+Y直接切姿态。"
                    if self.kind == "payload"
                    else "到达目标：等待静止后自动采集。",
                )
            return
        k.command(k.last_target)
        if self.phase == "settling":
            stable = self._settled(k, s, now)
            if stable:
                if self.stable_since is None:
                    self.stable_since = now
                if now - self.stable_since >= self.settle_s:
                    self.capture_anchor = s["q"].copy()
                    self.capture_gripper = s.get("gripper", {}).get("gripper_position_raw")
                    self.capture_id += 1
                    self._phase(
                        k,
                        "capture",
                        now,
                        f"CAPTURE：姿态{self.pose_id + 1}，"
                        + (
                            f"{self.confirmed_mass:.3f} kg，"
                            if self.confirmed_mass is not None
                            else ""
                        )
                        + f"稳态窗口{self.hold_s:g}秒；"
                        + (
                            "持续记录，挂载重量未标注。"
                            if self.kind == "payload"
                            else "采完自动继续。"
                        ),
                    )
            else:
                self.stable_since = None
            if now - self.phase_t > self.c["validation_settle_timeout_s"]:
                # Manual adjustment may take longer; keep holding, do not declare a driver fault.
                self.phase_t = now
                k.event("WAIT_STILL：尚未确认静止，继续等待；LB+RB可随时结束。")
            return
        if self.phase == "capture":
            self.steady = self._settled(k, s, now) and np.max(
                abs(s["q"] - self.capture_anchor)
            ) <= np.deg2rad(0.5)
            grip = s.get("gripper", {}).get("gripper_position_raw")
            if (
                grip is not None
                and self.capture_gripper is not None
                and abs(grip - self.capture_gripper) > 500
            ):
                self.steady = False
            if not self.steady:
                self._phase(
                    k,
                    "settling",
                    now,
                    "CAPTURE_RESTART：检测到调整／运动，本段不完整，重新等静止后采集。",
                )
                return
            if now - self.phase_t < self.hold_s:
                return
            k.event(f"CAPTURE_DONE：姿态{self.pose_id + 1}，记录段{self.capture_id}完成。")
            if self.kind == "payload":
                self.completed_windows += 1
                self._phase(
                    k,
                    "settling",
                    now,
                    f"姿态{self.pose_id + 1}继续记录；t={now - self.test_t:.1f}s。可自行换载，卸载后LB+Y切姿态。",
                )
            elif self.kind == "reciprocal":
                if self.direction == "forward":
                    self._moving(k, now, self.reverse, "reverse", 0)
                elif self.speed_round + 1 < len(self.forwards):
                    self.speed_round += 1
                    self.forward = self.forwards[self.speed_round]
                    self.reverse = Reverse(self.forward)
                    self._moving(k, now, self.forward, "forward", len(self.poses) - 1)
                else:
                    self.finish(k, now)
            else:
                self.visit += 1
                if self.visit >= len(self.order):
                    self.finish(k, now)
                else:
                    n = self.order[self.visit]
                    self._moving(
                        k,
                        now,
                        transit(k.last_target, self.poses[n], self.vmax, self.amax, minimum=6.0),
                        "static_move",
                        n,
                    )

    def finish(self, k, now):
        self.phase = "complete"
        self.steady = False
        p = transit(k.last_target, k.return_target(), k.vmax, k.amax)
        k.check(p)
        k.set_path("RETURNING", p, now)

    def metadata(self, s, now):
        paired = None
        if self.kind == "reciprocal" and self.phase == "moving":
            elapsed = float(np.clip(now - self.phase_t, 0, self.path.duration))
            paired = elapsed if self.direction == "forward" else self.forward.duration - elapsed
        return dict(
            protocol=self.kind,
            phase=self.phase,
            pose_id=self.pose_id + 1,
            visit=self.visit,
            direction=self.direction,
            pair_time_s=paired,
            mass_kg=self.confirmed_mass,
            mass_label_source="not annotated; operator controls loading independently"
            if self.kind == "payload"
            else "no external load prescribed",
            speed_round=self.speed_round + 1 if self.kind == "reciprocal" else None,
            speed_bound_rad_s=self.c["reciprocal_speeds_rad_s"][self.speed_round]
            if self.kind == "reciprocal"
            else self.vmax,
            capture_id=self.capture_id,
            steady_candidate=bool(self.phase == "capture" and self.steady),
            active_joints=self.active_joints,
            passive_joints=self.passive_joints,
            J6_measured_rad=float(s["q"][5]),
            gripper=s.get("gripper", {}),
        )

    def scripted_action(self):
        if (
            self.kind == "payload"
            and self.phase == "settling"
            and self.completed_windows >= 2 * (self.visit + 1)
        ):
            return "ABORT" if self.visit >= len(self.poses) - 1 else "START"
        return None

    def save(self, path):
        path = Path(path)
        self.plan["source_sha256"] = {
            name: hashlib.sha256(robot_source_path(name).read_bytes()).hexdigest()
            for name in [
                "validation_trials.py",
                "controller.py",
                "hardware.py",
                "standing.py",
                "run.py",
                "test_data.py",
                "simulation.py",
                "validation_simulation.py",
                "shutdown.py",
                "gamepad.py",
            ]
        }
        (path / "reference_plan.json").write_text(
            json.dumps(self.plan, ensure_ascii=False, indent=2) + "\n"
        )
        # Frozen forward path provides a reproducible pairing coordinate; actual
        # hold times and passive J6 coordinates are in TESTING rows.
        t = np.arange(0, self.forward.duration + 0.005, 0.01)
        t = np.clip(t, 0, self.forward.duration)
        values = [self.forward.at(x) for x in t]
        np.savez_compressed(
            path / "reference_trajectory.npz",
            t=t,
            q_ref=np.array([x[0] for x in values]),
            qd_ref=np.array([x[1] for x in values]),
            qdd_ref=np.array([x[2] for x in values]),
            coordinate="first-speed forward-only nominal path; full executed references and speed rounds are in test_samples.jsonl",
        )


def summarize(path):
    """Compact descriptive results, without asserting physical torque calibration."""
    path = Path(path)
    captures = {}
    with (path / "test_samples.jsonl").open() as stream:
        for line in stream:
            r = json.loads(line)
            tag = r.get("trial")
            if not tag:
                continue
            key = tag["capture_id"]
            if (
                tag["phase"] == "capture"
                and tag.get("steady_candidate")
                and r["quality"]["valid_measurement"]
            ):
                captures.setdefault(key, []).append(r)
    records = []
    for key, rows in captures.items():
        tag = rows[0]["trial"]
        duration = rows[-1]["t_monotonic_s"] - rows[0]["t_monotonic_s"]
        current = np.array([x["feedback"]["current_A"] for x in rows])
        q = np.array([x["feedback"]["q"] for x in rows])
        temp = np.array([x["feedback"].get("temp_C", [np.nan] * 6) for x in rows], float)
        records.append(
            dict(
                capture_id=key,
                pose_id=tag["pose_id"],
                visit=tag["visit"],
                direction=tag["direction"],
                mass_kg=tag["mass_kg"],
                speed_round=tag.get("speed_round"),
                speed_bound_rad_s=tag.get("speed_bound_rad_s"),
                duration_s=duration,
                complete=duration
                >= float(json.loads((path / "config.json").read_text())["validation_capture_s"])
                - 0.05,
                sample_count=len(rows),
                temperature_mean_C=temp.mean(0).tolist(),
                temperature_peak_C=temp.max(0).tolist(),
                current_mean_A=current.mean(0).tolist(),
                current_std_A=current.std(0).tolist(),
                q_mean_deg=np.rad2deg(q.mean(0)).tolist(),
                q_span_deg=np.rad2deg(np.ptp(q, axis=0)).tolist(),
                gripper_position_raw_mean=float(
                    np.mean(
                        [
                            x["feedback"].get("gripper", {}).get("gripper_position_raw", np.nan)
                            for x in rows
                        ]
                    )
                ),
            )
        )
    differences = []
    for r in records:
        if r["mass_kg"] is None or r["mass_kg"] == 0 or not r["complete"]:
            continue
        empty = next(
            (
                e
                for e in records
                if e["complete"] and e["visit"] == r["visit"] and e["mass_kg"] == 0
            ),
            None,
        )
        if empty is None:
            continue
        delta = np.array(r["q_mean_deg"]) - empty["q_mean_deg"]
        grip_delta = r["gripper_position_raw_mean"] - empty["gripper_position_raw_mean"]
        comparable = bool(
            np.max(abs(delta)) <= 0.5 and np.isfinite(grip_delta) and abs(grip_delta) <= 500
        )
        differences.append(
            dict(
                pose_id=r["pose_id"],
                visit=r["visit"],
                mass_kg=r["mass_kg"],
                delta_current_A=(np.array(r["current_mean_A"]) - empty["current_mean_A"]).tolist(),
                pose_delta_deg=delta.tolist(),
                gripper_delta_raw=grip_delta,
                same_geometry_for_direct_pairing=comparable,
                note="J6/gripper can change; mismatched geometry is not a same-pose torque calibration pair.",
            )
        )
    report = dict(
        payload_mass_annotations_required=json.loads((path / "config.json").read_text())[
            "trajectory"
        ]
        == "payload",
        protocol=json.loads((path / "config.json").read_text())["trajectory"],
        captures=records,
        payload_differences=differences,
        torque_calibrated=False,
        data_mode=json.loads((path / "test_manifest.json").read_text())["mode"],
        passive_joints=[6]
        if json.loads((path / "config.json").read_text()).get("passive_wrist")
        else [],
        fixed_at_startup_joints=[6]
        if json.loads((path / "config.json").read_text()).get("hold_start_wrist")
        else [],
        limitations=[
            "all six joints enabled; payload fixes J6 at startup angle and releases only the gripper",
            "payload masses are unlabelled in this free-loading workflow; annotate actual intervals before comparing known weights",
            "hanging point coordinates and current/command scaling must be verified before claiming Nm accuracy",
            "reciprocal matching must use measured q, |v|, temperature and actual J6/gripper geometry; reference pair time alone is insufficient",
        ],
    )
    (path / "validation_summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=True) + "\n"
    )
    return report
