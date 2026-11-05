"""Trial construction; task modules do not import the controller class."""
import numpy as np
from piper_dynamics.core.torque import K,B,C,WIRE_TO_JOINT
from piper_dynamics.real_robot.trials.trajectory import Excitation,ScaledTrajectory,Segment
def _configure_multisine(controller, cfg):
    """Build the existing multisine trial and provenance."""
    from piper_dynamics.real_robot.trials.multisine import RepositoryMultisine, reference_report, TrajectoryRejected
    controller.nominal_test = RepositoryMultisine(controller.start, cfg)
    controller.test = ScaledTrajectory(controller.nominal_test, controller.start, cfg['trajectory_scale'], cfg.get('trajectory_speed_scale', 1.0))
    low = np.deg2rad(cfg['lower_deg']) + np.deg2rad(cfg['margin_deg'])
    high = np.deg2rad(cfg['upper_deg']) - np.deg2rad(cfg['margin_deg'])
    controller.reference_plan = reference_report(controller.test, controller.nominal_test, low, high, controller.test_vmax, controller.test_amax)
    controller.emit(f'MULTISINE：六关节多正弦，幅度scale={controller.test.scale:g}，速度scale={controller.test.speed_scale:g}，时长{controller.test.duration:g}秒，' + f"峰值速度 {max(controller.reference_plan['max_velocity_per_joint_rad_s']):.3f} rad/s，" + f"峰值加速度 {max(controller.reference_plan['max_acceleration_per_joint_rad_s2']):.3f} rad/s²。")
    if controller.reference_plan['hard_violations']:
        raise TrajectoryRejected(controller.reference_plan)
    if controller.reference_plan['warnings']:
        controller.emit('REFERENCE_WARNING：' + str(controller.reference_plan['warnings']) + '；规划参考告警，不作急停。')

def _configure_gravity(controller, cfg):
    """Build the existing gravity trial and provenance."""
    controller.test = Segment(controller.start, controller.start, cfg['duration_s'])
    controller.nominal_test = controller.test

def _configure_mit(controller, cfg):
    """Build the existing mit trial and provenance."""
    from piper_dynamics.real_robot.trials.validation import Waypoints
    offsets = np.deg2rad(cfg['mit_test_offsets_deg'])
    poses = np.array([controller.start] + [controller.start + o for o in offsets] + [controller.start])
    controller.test = Waypoints(poses, controller.test_vmax, controller.test_amax, minimum=cfg['mit_test_minimum_segment_s'])
    controller.nominal_test = controller.test
    controller.check(controller.test, controller.test_vmax, controller.test_amax)
    from piper_dynamics.core.torque import K, B, C, WIRE_TO_JOINT
    controller.reference_plan = {'trajectory_type': 'mit', 'all_stages': 'MIT only, including prepare/return', 'poses_deg': np.rad2deg(poses).tolist(), 'offset_deg': [list(o) for o in cfg['mit_test_offsets_deg']], 'minimum_segment_s': cfg['mit_test_minimum_segment_s'], 'duration_s': controller.test.duration, 'test_max_velocity_rad_s': controller.test_vmax, 'test_max_acceleration_rad_s2': controller.test_amax, 'ff_scale': cfg['mit_ff_scale'], 'ff_ramp_s': cfg['mit_ff_ramp_s'], 'prepare_kp_protocol': cfg['mit_prepare_kp'], 'prepare_kd_protocol': cfg['mit_prepare_kd'], 'kp_protocol': cfg['mit_kp'], 'kd_protocol': cfg['mit_kd'], 'return_kp_protocol': cfg['mit_return_kp'], 'return_kd_protocol': cfg['mit_return_kd'], 'gravity_model': None if controller.gravity is None else controller.gravity.provenance, 'simulation_gain_assumption': 'PD protocol torque scaled to joint by b*c; not hardware gain calibration', 'zero_ff_note': 'requested FF exactly 0; old 8-bit encoding has finite zero residue'}
    controller.reference_plan['wire_conversion'] = {'unit': 'joint Nm / (b*c) on the wire; firmware multiplies back', 'k': K.tolist(), 'b': B.tolist(), 'c': C.tolist(), 'b_c': WIRE_TO_JOINT.tolist(), 't_ff_limit_Nm': (8 * WIRE_TO_JOINT).tolist(), 'k_b_c_Nm_per_A': (K * B * C).tolist()}
    controller.reference_plan['torque_estimate_caps_Nm'] = list(cfg['mit_joint_torque_estimate_caps_Nm'])
    controller.reference_plan['estimate_velocity_limit_rad_s'] = float(cfg['mit_estimate_velocity_limit_rad_s'])
    controller.reference_plan['budget_exceed_duration_s'] = float(cfg['mit_budget_exceed_duration_s'])

def _configure_friction(controller, cfg):
    """Build the existing friction trial and provenance."""
    from piper_dynamics.real_robot.trials.friction import FrictionTrial
    controller.trial = FrictionTrial(controller.start, cfg, controller.lower, controller.upper)
    controller.test = controller.trial.reference
    controller.nominal_test = controller.test
    controller.reference_plan = controller.trial.plan

def _configure_diagnostic(controller, cfg):
    """Build the existing diagnostic trial and provenance."""
    from piper_dynamics.real_robot.trials.validation import ValidationTrial
    controller.trial = ValidationTrial(controller.start, cfg, controller.lower, controller.upper)
    controller.test = controller.trial.reference
    controller.nominal_test = controller.test
    controller.reference_plan = controller.trial.plan
    controller.emit('验证模式：' + ('J1–J5 受控，J6 被动' if cfg.get('passive_wrist') else '六轴受控') + ('，正反档 ' + str(cfg['reciprocal_speeds_rad_s']) + ' rad/s' if cfg['trajectory'] == 'reciprocal' else '，移动上限 ' + str(cfg['validation_move_velocity_rad_s']) + ' rad/s'))
    if cfg['trajectory'] == 'payload':
        controller.emit('PAYLOAD：自行挂载／更换并等待；全程持续记录，不自动标重量。卸载后LB+Y直接切下一姿态，LB+RB结束。')

def _configure_pilot(controller, cfg):
    """Build the original scaled pilot trajectory."""
    controller.nominal_test = Excitation(controller.start, cfg['joint'] - 1, np.deg2rad(cfg['amplitude_deg']), cfg['frequency_hz'], cfg['duration_s'], cfg['ramp_s'])
    controller.test = ScaledTrajectory(controller.nominal_test, controller.start, cfg.get('trajectory_scale', 1.0), cfg.get('trajectory_speed_scale', 1.0))
    controller.check(controller.test)
    controller.emit(f"TEST_SCALE：{controller.test.scale:.0%} 幅度；J{cfg['joint']} ±{cfg['amplitude_deg'] * controller.test.scale:.3f}°，速度比例 {controller.test.speed_scale:g}，时长 {controller.test.duration:g} 秒。")

def configure_trial(controller,cfg):
    """Construct exactly one trial using the current execution configuration."""
    if controller.multisine:
        _configure_multisine(controller,cfg)
    elif cfg.get('trajectory') == 'gravity':
        _configure_gravity(controller,cfg)
    elif cfg.get('trajectory') == 'mit':
        _configure_mit(controller,cfg)
    elif cfg.get('trajectory') == 'friction':
        _configure_friction(controller,cfg)
    elif cfg.get('trajectory') in ('static', 'reciprocal', 'payload'):
        _configure_diagnostic(controller,cfg)
    else:
        _configure_pilot(controller,cfg)
