"""MIT trial configuration and compatibility exports."""
import numpy as np
from piper_dynamics.core.torque import K,B,C,NM_PER_A,WIRE_TO_JOINT
from piper_dynamics.core.gravity_model import GravityModel,sha
from piper_dynamics.real_robot.protocol.mit import uint,encode_mit,decode_command
from piper_dynamics.real_robot.controller import Controller
from piper_dynamics.paths import LEGACY_ROBOT_ROOT as BASE
from piper_dynamics.real_robot.backends.mit import MitHardware,MitSimulation
from piper_dynamics.real_robot.control.impedance import bumpless_reference
MIT_KP=[10.]*6
MIT_KD=[.8]*6
def configure(c,ff_scale=0.,gain_scale=1.,model='nominal',fit='sg21_LS'):
    if not np.isfinite(ff_scale) or not 0<=ff_scale<=1:raise ValueError('ff-scale must be in [0,1]')
    if not np.isfinite(gain_scale) or not .5<=gain_scale<=1:raise ValueError('mit-gain-scale must be in [0.5,1]')
    if not c.get('standing_start') or c.get('current_pose_only'):raise ValueError('MIT trial requires --standing')
    if model not in ('nominal','identified'):raise ValueError('Unknown gravity model')
    c.update(mit_ff_scale=float(ff_scale),mit_gain_scale=float(gain_scale),gravity_model=model,gravity_fit=fit,
        # Every PD-controlled stage uses the same vector; no stage may fall back to a lower gain.
        mit_kp=[float(x*gain_scale) for x in MIT_KP],mit_kd=[float(x*gain_scale) for x in MIT_KD],
        mit_prepare_kp=[float(x*gain_scale) for x in MIT_KP],mit_prepare_kd=[float(x*gain_scale) for x in MIT_KD],
        mit_return_kp=[float(x*gain_scale) for x in MIT_KP],mit_return_kd=[float(x*gain_scale) for x in MIT_KD],
        mit_ff_ramp_s=3.,mit_joint_torque_estimate_caps_Nm=[16.,16.,14.,3.,3.,2.],
        # A one-cycle velocity spike must not abort the run: the estimate only faults
        # after a sustained exceedance, and its D term uses a clamped velocity.
        mit_budget_exceed_duration_s=.10,mit_estimate_velocity_limit_rad_s=.6,
        # Prepare/return keep the slow 0.08 rad/s; only the TESTING path uses 0.15 so
        # the arm visibly swings while gravity still dominates the torque.
        max_velocity_rad_s=.08,max_acceleration_rad_s2=.08,
        test_max_velocity_rad_s=.15,test_max_acceleration_rad_s2=.15,
        # Test path: start -> +shoulder/elbow -> add wrist -> shoulder/elbow back ->
        # return. Four segments of roughly 15-30 deg keep every joint moving while the
        # whole TESTING phase stays near 19 s.
        mit_test_offsets_deg=[[10.,15.,15.,0.,0.,0.],
                              [10.,15.,15.,12.,15.,20.],
                              [-10.,-15.,-15.,12.,15.,20.]],
        mit_test_minimum_segment_s=3.5,
        settle_error_deg=2.8,mit_prepare_tolerance_deg=8.0,
        # Pure PD leaves ~4.2-4.3 deg static sag at the park pose, so the MOVE J 3 deg
        # release window would always time out. This MIT-only window is used instead.
        mit_park_tolerance_deg=8.,park_release_tolerance_deg=3.,settle_timeout_s=20.,mit_release_s=2.0,
        feedback_stop_policy='stop',passive_wrist=False,hold_start_wrist=False,release_gripper=False,
        torque_candidate_coefficients_Nm_per_A=NM_PER_A.tolist(),
        torque_candidate_profile='official standard PiPER k*b*c; not individually calibrated')

MitController=Controller
