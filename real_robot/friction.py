"""六轴低速摩擦采集：单轴小行程、多档速度、正反往复、换向驻留。

复用现有 MOVE J 位置控制与阶段机；本模块只负责生成整段参考时间线并提供
逐样本的标签（关节、速度档、方向、重复序号），供离线摩擦拟合使用。
"""
import numpy as np
from .trajectory import Segment, transit, check_path


class Timeline:
    """由若干 Segment 顺序拼成的长参考；at(t) 与普通 path 接口一致。"""

    def __init__(self, parts, labels):
        self.parts = parts
        self.labels = labels
        self.ends = []
        self.duration = 0.
        for part in parts:
            self.duration += part.duration
            self.ends.append(self.duration)

    def index(self, t):
        t = float(np.clip(t, 0., self.duration))
        return min(int(np.searchsorted(self.ends, t, side='right')), len(self.parts) - 1)

    def at(self, t):
        i = self.index(t)
        begin = 0. if i == 0 else self.ends[i - 1]
        return self.parts[i].at(t - begin)


def configure(c):
    c.update(
        friction_joints=[1, 6, 4, 5, 2, 3],
        friction_stroke_deg=[20., 12., 12., 12., 12., 20.],
        friction_speeds_rad_s=[.005, .01, .02, .05, .1, .2],
        friction_dwell_s=3.,
        friction_repeats=2,
        friction_move_acceleration_rad_s2=.08,
        feedback_stop_policy='stop',
        loop_watchdog_s=.5,raw_can_enabled=False,export_csv=False,
        passive_wrist=False, hold_start_wrist=False, release_gripper=False,
        trajectory_scale=1., trajectory_speed_scale=1.,
    )


class FrictionTrial:
    kind = 'friction'

    def __init__(self, start, c, lower, upper):
        self.c = c
        self.active_joints = [1, 2, 3, 4, 5, 6]
        self.passive_joints = []
        self.amax = float(c['friction_move_acceleration_rad_s2'])
        self.dwell = float(c['friction_dwell_s'])
        self.speeds = [float(v) for v in c['friction_speeds_rad_s']]
        self.joints = [int(j) for j in c['friction_joints']]
        self.strokes = [float(v) for v in c['friction_stroke_deg']]
        self.repeats = int(c['friction_repeats'])
        start = np.asarray(start, float)
        parts = []
        labels = []
        pose = 0

        def add(part, joint, speed, direction, repeat, phase):
            nonlocal pose
            check_path(part, lower, upper, max(speed, 1e-6), self.amax)
            parts.append(part)
            labels.append(dict(joint=joint, speed=speed, direction=direction,
                               repeat=repeat + 1, phase=phase, pose_id=pose))
            pose += 1

        for joint in self.joints:
            if not 1 <= joint <= 6:
                raise ValueError('friction joint must be 1..6')
            axis = joint - 1
            stroke = np.deg2rad(self.strokes[axis]) / 2.
            center = start.copy()
            plus = start.copy(); plus[axis] += stroke
            minus = start.copy(); minus[axis] -= stroke
            for repeat in range(self.repeats):
                for round_index, speed in enumerate(self.speeds):
                    hold = Segment(center, center, self.dwell)
                    add(hold, joint, speed, 'hold', repeat, f'hold_{round_index}')
                    add(transit(center, plus, speed, self.amax, minimum=1.), joint, speed, 'forward', repeat, 'forward')
                    add(Segment(plus, plus, self.dwell), joint, speed, 'hold', repeat, 'hold_plus')
                    add(transit(plus, minus, speed, self.amax, minimum=1.), joint, speed, 'reverse', repeat, 'reverse')
                    add(Segment(minus, minus, self.dwell), joint, speed, 'hold', repeat, 'hold_minus')
                    add(transit(minus, center, speed, self.amax, minimum=1.), joint, speed, 'return', repeat, 'return')
        self.reference = Timeline(parts, labels)
        self.plan = {
            'schema_version': 1,
            'trajectory_type': 'friction',
            'joint_order': self.joints,
            'stroke_deg': self.strokes,
            'speeds_rad_s': self.speeds,
            'dwell_s': self.dwell,
            'repeats': self.repeats,
            'move_acceleration_bound_rad_s2': self.amax,
            'duration_s': self.reference.duration,
            'segments': len(parts),
            'controlled_joints': self.active_joints,
            'gripper_policy': 'unchanged; no gripper commands',
            'time_policy': 'one joint at a time; other joints hold the standing pose',
            'direction_policy': 'forward/reverse traverse the same path; same-speed pairs cancel gravity',
        }
        self.test_t = None
        self.phase = 'not_started'
        self.completed = False
        self.last_index = -1

    def begin(self, k, now):
        self.test_t = now
        self.phase = 'running'
        k.event(f'FRICTION：六轴低速摩擦采集，{len(self.joints)} 轴、{len(self.speeds)} 档速度、'
                f'每档 {self.repeats} 次，预计 {self.reference.duration/60:.1f} 分钟')

    def tick(self, k, now, s, action):
        elapsed = now - self.test_t
        if elapsed >= self.reference.duration:
            self.completed = True
            k.request_stop(s, now, 'friction sequence completed')
            return
        q, _, _ = self.reference.at(elapsed)
        k.command(q)
        self.last_index = self.reference.index(elapsed)
        self.phase = self.reference.labels[self.last_index]['phase']

    def metadata(self, s, now):
        label = self.reference.labels[max(self.last_index, 0)] if self.labels_ready(now) else self.reference.labels[0]
        return dict(protocol='friction', phase=label['phase'], pose_id=label['pose_id'] + 1,
                    joint=label['joint'], repeat=label['repeat'], direction=label['direction'],
                    speed_round=self.speeds.index(label['speed']) + 1,
                    speed_bound_rad_s=label['speed'], pair_time_s=None, mass_kg=None,
                    capture_id=0, steady_candidate=label['phase'].startswith('hold'))

    def labels_ready(self, now):
        return self.test_t is not None and now >= self.test_t

    def save(self, path):
        import hashlib, json
        from pathlib import Path
        path = Path(path)
        self.plan['source_sha256'] = {name: hashlib.sha256(
            (Path(__file__).parent / name).read_bytes()).hexdigest()
            for name in ['friction.py', 'controller.py', 'hardware.py', 'standing.py',
                         'run.py', 'test_data.py', 'simulation.py', 'shutdown.py']}
        (path / 'reference_plan.json').write_text(
            json.dumps(self.plan, ensure_ascii=False, indent=2) + '\n')
