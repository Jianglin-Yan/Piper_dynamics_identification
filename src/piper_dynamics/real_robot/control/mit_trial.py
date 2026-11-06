"""MIT trial configuration and compatibility exports."""

import numpy as np

from piper_dynamics.core.gravity_model import GravityModel, sha
from piper_dynamics.core.torque import NM_PER_A, WIRE_TO_JOINT, B, C, K
from piper_dynamics.paths import LEGACY_ROBOT_ROOT as BASE
from piper_dynamics.real_robot.backends.mit import MitHardware, MitSimulation
from piper_dynamics.real_robot.control.impedance import bumpless_reference
from piper_dynamics.real_robot.controller import Controller
from piper_dynamics.real_robot.protocol.mit import decode_command, encode_mit, uint

MIT_KP = [10.0] * 6
MIT_KD = [0.8] * 6


def configure(c, ff_scale=0.0, gain_scale=1.0, model="nominal", fit="sg21_LS"):
    if not np.isfinite(ff_scale) or not 0 <= ff_scale <= 1:
        raise ValueError("ff-scale must be in [0,1]")
    if not np.isfinite(gain_scale) or not 0.5 <= gain_scale <= 1:
        raise ValueError("mit-gain-scale must be in [0.5,1]")
    if not c.get("standing_start") or c.get("current_pose_only"):
        raise ValueError("MIT trial requires --standing")
    if model not in ("nominal", "identified"):
        raise ValueError("Unknown gravity model")
    c.update(
        mit_ff_scale=float(ff_scale),
        mit_gain_scale=float(gain_scale),
        gravity_model=model,
        gravity_fit=fit,
        # Every PD-controlled stage uses the same vector; no stage may fall back to a lower gain.
        mit_kp=[float(x * gain_scale) for x in MIT_KP],
        mit_kd=[float(x * gain_scale) for x in MIT_KD],
        mit_prepare_kp=[float(x * gain_scale) for x in MIT_KP],
        mit_prepare_kd=[float(x * gain_scale) for x in MIT_KD],
        mit_return_kp=[float(x * gain_scale) for x in MIT_KP],
        mit_return_kd=[float(x * gain_scale) for x in MIT_KD],
        mit_ff_ramp_s=3.0,
        mit_joint_torque_estimate_caps_Nm=[16.0, 16.0, 14.0, 3.0, 3.0, 2.0],
        # A one-cycle velocity spike must not abort the run: the estimate only faults
        # after a sustained exceedance, and its D term uses a clamped velocity.
        mit_budget_exceed_duration_s=0.10,
        mit_estimate_velocity_limit_rad_s=0.6,
        # Prepare/return keep the slow 0.08 rad/s; only the TESTING path uses 0.15 so
        # the arm visibly swings while gravity still dominates the torque.
        max_velocity_rad_s=0.08,
        max_acceleration_rad_s2=0.08,
        test_max_velocity_rad_s=0.15,
        test_max_acceleration_rad_s2=0.15,
        # Test path: start -> +shoulder/elbow -> add wrist -> shoulder/elbow back ->
        # return. Four segments of roughly 15-30 deg keep every joint moving while the
        # whole TESTING phase stays near 19 s.
        mit_test_offsets_deg=[
            [10.0, 15.0, 15.0, 0.0, 0.0, 0.0],
            [10.0, 15.0, 15.0, 12.0, 15.0, 20.0],
            [-10.0, -15.0, -15.0, 12.0, 15.0, 20.0],
        ],
        mit_test_minimum_segment_s=3.5,
        settle_error_deg=2.8,
        mit_prepare_tolerance_deg=8.0,
        # Pure PD leaves ~4.2-4.3 deg static sag at the park pose, so the MOVE J 3 deg
        # release window would always time out. This MIT-only window is used instead.
        mit_park_tolerance_deg=8.0,
        park_release_tolerance_deg=3.0,
        settle_timeout_s=20.0,
        mit_release_s=2.0,
        feedback_stop_policy="stop",
        passive_wrist=False,
        hold_start_wrist=False,
        release_gripper=False,
        torque_candidate_coefficients_Nm_per_A=NM_PER_A.tolist(),
        torque_candidate_profile="official standard PiPER k*b*c; not individually calibrated",
    )


DRAG_KP = 2.0
DRAG_KD = 0.15


def configure_drag(c, model="nominal", fit="sg21_LS"):
    """配置拖动测试Kp=2、Kd=0.15、前馈100%，准备/回位/急停保留基础10/0.8。

    沿用三秒渐变，使测试增益与前馈一起过渡；手推速度、电流监视值保持现有配置。"""
    configure(c, 1.0, 1.0, model, fit)
    c.update(
        mit_kp=[DRAG_KP] * 6,
        mit_kd=[DRAG_KD] * 6,
        mit_test_offsets_deg=[],
        mit_test_minimum_segment_s=1.0,
        # A hand push is much faster than a scripted trajectory; keep a stop line
        # below the firmware 3 rad/s but well above normal hand motion.
        feedback_speed_warning_rad_s=1.0,
        feedback_speed_stop_rad_s=2.0,
        feedback_speed_stop_duration_s=0.2,
        feedback_speed_emergency_rad_s=2.8,
        feedback_speed_emergency_duration_s=0.1,
        max_current_feedback_A=[4.0, 4.0, 4.0, 3.0, 3.0, 3.0],
        feedback_current_warning_multiplier=0.75,
    )


MitController = Controller

__all__ = [
    "GravityModel",
    "MitController",
    "MitHardware",
    "MitSimulation",
    "configure",
    "MIT_KP",
    "MIT_KD",
    "K",
    "B",
    "C",
    "NM_PER_A",
    "WIRE_TO_JOINT",
    "BASE",
    "sha",
    "uint",
    "encode_mit",
    "decode_command",
    "bumpless_reference",
]
