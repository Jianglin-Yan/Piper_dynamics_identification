"""Configuration construction and validation for existing PiPER trials."""

import json

import numpy as np

from piper_dynamics.paths import LEGACY_ROBOT_ROOT as BASE
from piper_dynamics.real_robot.monitoring import validate_speed_limits


def validate_common_options(c):
    "Validate common options."
    validate_speed_limits(c)
    c.setdefault("trajectory", "pilot")
    if c["trajectory"] not in (
        "pilot",
        "multisine",
        "static",
        "reciprocal",
        "payload",
        "friction",
        "gravity",
        "mit",
        "drag",
    ):
        raise ValueError("Unknown trajectory")
    c.setdefault("trajectory_scale", 1.0)
    c.setdefault("trajectory_speed_scale", 1.0)
    if not np.isfinite(c["trajectory_speed_scale"]) or not 0 < c["trajectory_speed_scale"] <= 1:
        raise ValueError("trajectory_speed_scale must be in (0, 1]")
    if not np.isfinite(c["trajectory_scale"]) or not 0 < c["trajectory_scale"] <= 1:
        raise ValueError("trajectory_scale must be in (0, 1]")
    c.setdefault("startup_limit_tolerance_deg", 10.0)
    c.setdefault("tracking_warning_deg", 10.0)
    c.setdefault("measurement_angle_tolerance_deg", 6.0)
    if (
        not np.isfinite(c["startup_limit_tolerance_deg"])
        or not 0 < c["startup_limit_tolerance_deg"] <= 15
    ):
        raise ValueError("Invalid startup allowance")
    if (
        not np.isfinite(c["measurement_angle_tolerance_deg"])
        or not 0 <= c["measurement_angle_tolerance_deg"] <= 15
    ):
        raise ValueError("Invalid measurement angle tolerance")
    if (
        not np.isfinite(c["tracking_warning_deg"])
        or not 0 < c["tracking_warning_deg"] < c["tracking_error_deg"]
    ):
        raise ValueError("Tracking warning must be below hard threshold")
    c.setdefault("settle_position_window_deg", 0.2)
    c.setdefault("arming_velocity_limit_rad_s", 0.15)
    c.setdefault("park_release_tolerance_deg", 3.0)
    c.setdefault("disable_timeout_s", 2.0)
    c.setdefault("disable_confirm_s", 0.15)
    for key in (
        "settle_position_window_deg",
        "arming_velocity_limit_rad_s",
        "park_release_tolerance_deg",
        "disable_timeout_s",
        "disable_confirm_s",
    ):
        if not np.isfinite(c[key]) or c[key] <= 0:
            raise ValueError("Invalid " + key)


def validate_joint_limits(c):
    "Validate joint limits."
    if not c["settle_error_deg"] <= c["park_release_tolerance_deg"] <= 3.0:
        raise ValueError("Park release tolerance must be between settle tolerance and 3 degrees")
    if c["disable_confirm_s"] >= c["disable_timeout_s"]:
        raise ValueError("Disable confirmation must fit the timeout")
    for key in ("lower_deg", "upper_deg", "simulation_initial_deg", "max_current_feedback_A"):
        if len(c[key]) != 6 or not np.all(np.isfinite(c[key])):
            raise ValueError("Invalid " + key)
    if np.any(np.array(c["max_current_feedback_A"]) <= 0):
        raise ValueError("Current thresholds must be positive")
    if np.any(np.array(c["lower_deg"]) + 2 * c["margin_deg"] >= np.array(c["upper_deg"])):
        raise ValueError("Empty joint safety interval")
    if c["start_deg"] is not None and (
        len(c["start_deg"]) != 6 or not np.all(np.isfinite(c["start_deg"]))
    ):
        raise ValueError("Invalid start_deg")
    for key in (
        "amplitude_deg",
        "frequency_hz",
        "duration_s",
        "ramp_s",
        "margin_deg",
        "max_velocity_rad_s",
        "max_acceleration_rad_s2",
        "tracking_error_deg",
        "settle_error_deg",
        "settle_velocity_rad_s",
        "settle_timeout_s",
        "feedback_timeout_s",
        "status_timeout_s",
        "loop_watchdog_s",
        "max_temperature_C",
        "command_hz",
    ):
        if not np.isfinite(c[key]) or c[key] <= 0:
            raise ValueError("Invalid " + key)


def validate_pilot_profile(c):
    "Validate pilot profile."
    if type(c["joint"]) is not int or c["joint"] not in range(1, 7):
        raise ValueError("joint must be 1..6")
    if c["model"] != "piper":
        raise ValueError("Only standard PiPER is supported")
    coeff = np.asarray(c.get("torque_candidate_coefficients_Nm_per_A", []), float)
    if coeff.shape != (6,) or not np.all(np.isfinite(coeff)) or np.any(coeff == 0):
        raise ValueError("Six finite nonzero candidate torque coefficients required")
    if c["command_hz"] != 100:
        raise ValueError("First pilot profile uses 100 Hz commands")
    if (
        c["max_velocity_rad_s"] > 0.2
        or c["max_acceleration_rad_s2"] > 0.3
        or c["amplitude_deg"] > 5
        or (c["frequency_hz"] > 0.3)
    ):
        raise ValueError("Outside small single-joint pilot profile")
    if not 1 <= c["move_speed_percent"] <= 20:
        raise ValueError("Pilot move speed percentage must be 1..20")


def validate_multisine(c):
    "Validate multisine."
    if c["trajectory"] == "multisine":
        if c.get("current_pose_only"):
            raise ValueError("Six-joint multisine cannot use --from-current")
        for key in (
            "test_max_velocity_rad_s",
            "test_max_acceleration_rad_s2",
            "braking_max_acceleration_rad_s2",
            "test_tracking_warning_deg",
            "test_tracking_hard_deg",
            "multisine_base_frequency_hz",
            "multisine_sample_dt_s",
        ):
            if not np.isfinite(c[key]) or c[key] <= 0:
                raise ValueError("Invalid " + key)
        if c["test_tracking_warning_deg"] >= c["test_tracking_hard_deg"]:
            raise ValueError("Tracking warning must be below hard threshold")
        if type(c["multisine_harmonics"]) is not int or not 1 <= c["multisine_harmonics"] <= 5:
            raise ValueError("Invalid harmonic count")
        if type(c["multisine_seed"]) is not int or c["multisine_seed"] < 0:
            raise ValueError("Invalid multisine seed")
        if not 0 < c["multisine_amplitude_fraction"] <= 0.6:
            raise ValueError("Invalid multisine amplitude fraction")
        for key in ("multisine_generation_lower_deg", "multisine_generation_upper_deg"):
            if len(c[key]) != 6 or not np.all(np.isfinite(c[key])):
                raise ValueError("Invalid " + key)
        if np.any(
            np.array(c["multisine_generation_lower_deg"])
            >= np.array(c["multisine_generation_upper_deg"])
        ):
            raise ValueError("Invalid multisine generation interval")
        if c["multisine_sample_dt_s"] != 0.002:
            raise ValueError("Original generator uses 0.002 s interpolation knots")
        if c.get("feedback_stop_policy") != "warn":
            raise ValueError("Multisine profile requires warn-only reference policy")


def validate_diagnostic_trials(c):
    "Validate diagnostic trials."
    if c["trajectory"] in ("static", "reciprocal", "payload"):
        if not c.get("standing_start") or c.get("current_pose_only"):
            raise ValueError("Validation trials require --standing")
        payload = c["trajectory"] == "payload"
        if (
            c.get("passive_wrist")
            or bool(c.get("release_gripper")) != payload
            or bool(c.get("hold_start_wrist")) != payload
        ):
            raise ValueError(
                "Validation uses six enabled joints; only payload holds startup J6 and releases gripper"
            )
        speeds = c["reciprocal_speeds_rad_s"]
        if not speeds or any((not np.isfinite(v) or not 0.01 <= v <= 0.12 for v in speeds)):
            raise ValueError("Reciprocal speeds must be within 0.01..0.12 rad/s")
        if c["trajectory_scale"] != 1 or c["trajectory_speed_scale"] != 1:
            raise ValueError(
                "Validation protocols use fixed slow paths; do not apply multisine scales"
            )
        for key in (
            "validation_move_velocity_rad_s",
            "validation_move_acceleration_rad_s2",
            "validation_settle_s",
            "validation_capture_s",
            "validation_settle_timeout_s",
        ):
            if not np.isfinite(c[key]) or c[key] <= 0:
                raise ValueError("Invalid " + key)
        if (
            c["validation_move_velocity_rad_s"] > 0.12
            or c["validation_move_acceleration_rad_s2"] > 0.12
        ):
            raise ValueError("Validation paths exceed slow profile")
        if not 1 <= c["validation_settle_s"] <= 30 or not 2 <= c["validation_capture_s"] <= 60:
            raise ValueError("Invalid diagnostic dwell")


def validate_friction(c):
    "Validate friction."
    if c["trajectory"] == "friction":
        if c.get("friction_start", "current") == "current":
            if c.get("standing_start") or c.get("current_pose_only"):
                raise ValueError(
                    "Current-pose friction collection must not use --standing/--from-current"
                )
        elif not c.get("standing_start") or c.get("current_pose_only"):
            raise ValueError("Standing friction collection requires --standing")
        if c.get("passive_wrist") or c.get("hold_start_wrist") or c.get("release_gripper"):
            raise ValueError("Friction collection uses six enabled joints and no gripper")
        if c["trajectory_scale"] != 1 or c["trajectory_speed_scale"] != 1:
            raise ValueError("Friction collection uses fixed paths")
        joints = c["friction_joints"]
        if (
            not isinstance(joints, (list, tuple))
            or not joints
            or any(type(joint) is not int for joint in joints)
            or len(set(joints)) != len(joints)
            or any(not 1 <= joint <= 6 for joint in joints)
        ):
            raise ValueError(
                "Friction joints must be unique integers in 1..6; execution order is preserved"
            )
        strokes = np.asarray(c["friction_stroke_deg"], float)
        if (
            strokes.shape != (6,)
            or not np.isfinite(strokes).all()
            or np.any(strokes < 2)
            or np.any(strokes > 40)
        ):
            raise ValueError("Invalid friction stroke")
        speeds = np.asarray(c["friction_speeds_rad_s"], float)
        if (
            speeds.size < 2
            or not np.isfinite(speeds).all()
            or np.any(speeds < 0.002)
            or np.any(speeds > 0.25)
            or np.any(np.diff(speeds) <= 0)
        ):
            raise ValueError("Invalid friction speeds")
        if not 0.5 <= c["friction_dwell_s"] <= 10 or not 1 <= c["friction_repeats"] <= 5:
            raise ValueError("Invalid friction dwell/repeats")
        if (
            not np.isfinite(c["friction_move_acceleration_rad_s2"])
            or not 0 < c["friction_move_acceleration_rad_s2"] <= 0.5
        ):
            raise ValueError("Invalid friction acceleration")


def validate_mit(c):
    "Validate mit."
    if c["trajectory"] in ("mit", "drag"):
        if (
            not c.get("standing_start")
            or c.get("current_pose_only")
            or c["trajectory_scale"] != 1
            or (c["trajectory_speed_scale"] != 1)
        ):
            raise ValueError("MIT uses its fixed standing path without multisine scaling")
        if (
            not np.isfinite(c["mit_ff_scale"])
            or not 0 <= c["mit_ff_scale"] <= 1
            or c["mit_ff_ramp_s"] != 3.0
        ):
            raise ValueError("MIT FF fraction/ramp invalid")
        for key in (
            "mit_kp",
            "mit_kd",
            "mit_prepare_kp",
            "mit_prepare_kd",
            "mit_return_kp",
            "mit_return_kd",
        ):
            values = np.asarray(c[key], float)
            if values.shape != (6,) or not np.isfinite(values).all() or np.any(values <= 0):
                raise ValueError("Invalid " + key)
        if np.any(np.array(c["mit_return_kp"]) < c["mit_kp"]) or np.any(
            np.array(c["mit_return_kd"]) < c["mit_kd"]
        ):
            raise ValueError("MIT return gains must not be lower than test gains")


def validate_config(c):
    "Validate the effective configuration in the original check order."
    validate_common_options(c)
    validate_joint_limits(c)
    validate_pilot_profile(c)
    validate_multisine(c)
    validate_diagnostic_trials(c)
    validate_friction(c)
    validate_mit(c)
    if c["duration_s"] > 120 or c["duration_s"] < 2 * c["ramp_s"]:
        raise ValueError("Invalid trial duration")


def configure_mit(cfg, a, ap, selected):
    "Apply mit/drag trial overrides."
    if selected == "drag":
        if not (a.sim or a.hardware):
            ap.error("Drag trial is sim/hardware only")
        if a.ff_scale is not None or a.mit_gain_scale is not None:
            ap.error("Drag uses a fixed Kp 5 and 100% feedforward")
        from piper_dynamics.real_robot.control.mit_trial import configure_drag

        configure_drag(cfg, a.gravity_model or "identified", a.gravity_fit or "sg21_LS")
        return
    if selected == "mit":
        if not (a.sim or a.hardware):
            ap.error("MIT trial is sim/hardware only")
        if a.gravity_seconds is not None or a.gravity_stiffness is not None:
            ap.error(
                "MIT fixed-path trial uses --ff-scale and --mit-gain-scale, not gravity hold options"
            )
        from piper_dynamics.real_robot.control.mit_trial import configure

        configure(
            cfg,
            0.0 if a.ff_scale is None else a.ff_scale,
            1.0 if a.mit_gain_scale is None else a.mit_gain_scale,
            a.gravity_model or "nominal",
            a.gravity_fit or "sg21_LS",
        )
    elif a.ff_scale is not None or a.mit_gain_scale is not None:
        ap.error("--ff-scale/--mit-gain-scale require --trajectory mit")


def configure_gravity(cfg, a, ap, selected):
    "Apply gravity trial overrides."
    if selected == "gravity":
        if a.hardware:
            ap.error(
                "Legacy MOVE J-to-MIT hardware trial retired. Use bash real_robot/test_mit.sh hardware --ff-scale 0 or 0.25"
            )
        from piper_dynamics.real_robot.control.gravity_hold import configure

        configure(
            cfg,
            a.gravity_model or "identified",
            a.gravity_fit or "sg21_LS",
            30.0 if a.gravity_seconds is None else a.gravity_seconds,
            1.0 if a.gravity_stiffness is None else a.gravity_stiffness,
        )
    elif selected != "mit" and any(
        (
            x is not None
            for x in (a.gravity_model, a.gravity_fit, a.gravity_seconds, a.gravity_stiffness)
        )
    ):
        ap.error("Gravity options require --trajectory gravity/mit")


def configure_friction(cfg, a, ap, selected):
    "Apply friction trial overrides."
    if selected == "friction":
        from piper_dynamics.real_robot.trials.friction import configure as friction_configure

        friction_configure(cfg)
        if a.friction_joints is not None:
            cfg["friction_joints"] = a.friction_joints
        if a.friction_speeds is not None:
            cfg["friction_speeds_rad_s"] = a.friction_speeds
        if a.friction_repeats is not None:
            cfg["friction_repeats"] = a.friction_repeats
        cfg["friction_start"] = a.friction_start
    elif any((x is not None for x in (a.friction_joints, a.friction_speeds, a.friction_repeats))):
        ap.error("--friction-* options require --trajectory friction")


def configure_diagnostics(cfg, a, ap, selected):
    "Apply diagnostics trial overrides."
    if selected in ("static", "reciprocal", "payload"):
        from piper_dynamics.real_robot.trials.validation import configure

        configure(cfg)
        if a.reciprocal_speeds is not None:
            if selected != "reciprocal":
                ap.error("--reciprocal-speeds requires the reciprocal test")
            cfg["reciprocal_speeds_rad_s"] = a.reciprocal_speeds
        if a.capture_seconds is not None:
            cfg["validation_capture_s"] = a.capture_seconds
        if a.settle_seconds is not None:
            cfg["validation_settle_s"] = a.settle_seconds
        if a.payload_point_m is not None:
            if not np.all(np.isfinite(a.payload_point_m)):
                ap.error("Invalid hanging point")
            cfg["payload_point_link6_m"] = a.payload_point_m
    elif any(
        (
            x is not None
            for x in (a.capture_seconds, a.settle_seconds, a.payload_point_m, a.reciprocal_speeds)
        )
    ):
        ap.error("Diagnostic options require static, reciprocal or payload")


def apply_execution_options(cfg, a, ap, selected):
    "Apply explicit CLI overrides and frozen candidate settings."
    if a.scale is not None:
        cfg["trajectory_scale"] = a.scale
    if a.speed_scale is not None:
        cfg["trajectory_speed_scale"] = a.speed_scale
    if a.seed is not None:
        if selected != "multisine":
            ap.error("--seed requires --trajectory multisine")
        cfg["multisine_seed"] = a.seed
    if a.standing:
        from piper_dynamics.real_robot.standing import STANDING_DEG

        cfg["start_deg"] = cfg.get("standing_start_deg", STANDING_DEG)
    if a.raw_can:
        cfg["raw_can_enabled"] = True
    if a.csv:
        cfg["export_csv"] = True
    cfg.setdefault("raw_can_enabled", True)
    cfg.setdefault("export_csv", True)
    if selected == "friction" and cfg.get("friction_start") == "current":
        cfg["standing_start"] = False
        cfg["start_deg"] = None
    if a.candidate:
        if selected != "multisine" or a.seed is not None:
            ap.error("--candidate requires --trajectory multisine; cannot combine with --seed")
        from piper_dynamics.real_robot.trials.designed_trajectory import load_candidate

        candidate = load_candidate(cfg, a.candidate, a.candidate_bank)
        print(
            "CANDIDATE："
            + a.candidate
            + f"；冻结幅度/速度={cfg['trajectory_scale']:g}/{cfg['trajectory_speed_scale']:g}；"
            + candidate["description_zh"],
            flush=True,
        )


def build_config(a, ap):
    "Load a profile, apply ordered overrides and validate before transport startup."
    cfg = json.loads(a.config.read_text())
    selected = a.trajectory or cfg.get("trajectory", "pilot")
    if selected == "multisine":
        if not a.standing or a.from_current:
            ap.error("Six-joint multisine uses --standing; cannot use --from-current")
        if cfg.get("trajectory") != "multisine":
            cfg.update(json.loads((BASE / "multisine_profile.json").read_text()))
    cfg["trajectory"] = selected
    cfg["current_pose_only"] = a.from_current
    cfg["standing_start"] = a.standing
    configure_mit(cfg, a, ap, selected)
    configure_gravity(cfg, a, ap, selected)
    configure_friction(cfg, a, ap, selected)
    configure_diagnostics(cfg, a, ap, selected)
    apply_execution_options(cfg, a, ap, selected)
    validate_config(cfg)
    if a.from_current:
        if cfg["joint"] != 6 or cfg["start_deg"] is not None or cfg["frequency_hz"] > 0.1:
            raise ValueError("--from-current requires joint=6, start_deg=null, frequency<=0.1 Hz")
    return cfg
