"""Command-line parsing and input-only utilities; no transport is opened here."""

import argparse
import time
from pathlib import Path
import numpy as np
from piper_dynamics.paths import LEGACY_ROBOT_ROOT as BASE
from piper_dynamics.real_robot.gamepad import Gamepad, calibrate


def build_parser():
    "Build the existing PiPER command-line interface."
    ap = argparse.ArgumentParser(description=__doc__)
    mode = ap.add_mutually_exclusive_group(required=True)
    for flag in ("sim", "hardware", "read-only", "calibrate-pad", "monitor-pad", "recover"):
        mode.add_argument("--" + flag, action="store_true")
    add_control_options(ap)
    add_trial_options(ap)
    add_runtime_options(ap)
    return ap


def add_control_options(ap):
    "Add mode, model and MIT command options."
    ap.add_argument(
        "--supported",
        action="store_true",
        help="explicit recovery only: confirm the folded arm is physically supported for power loss",
    )
    ap.add_argument("--config", type=Path, default=BASE / "config.json")
    ap.add_argument("--pad", type=Path, default=BASE / "g8.json")
    ap.add_argument(
        "--standing",
        action="store_true",
        help="LB+X performs boundary entry then moves to a fixed standing test pose",
    )
    ap.add_argument(
        "--from-current",
        action="store_true",
        help="J6-only trial anchored at actual startup pose; limited parked J2/J3 exceptions",
    )
    ap.add_argument(
        "--trajectory",
        choices=[
            "pilot",
            "multisine",
            "static",
            "reciprocal",
            "payload",
            "friction",
            "gravity",
            "mit",
        ],
        help="pilot/multisine diagnostics or unified MIT PD/FF trial",
    )
    ap.add_argument(
        "--ff-scale", type=float, help="MIT-only trial: gravity feedforward fraction 0..1, default0"
    )
    ap.add_argument(
        "--mit-gain-scale",
        type=float,
        help="MIT-only trial: protocol PD gain fraction 0.5..1, default1",
    )
    ap.add_argument(
        "--gravity-model",
        choices=["nominal", "identified"],
        help="gravity/MIT: official nominal or corrected balanced-v2 identification",
    )
    ap.add_argument(
        "--gravity-fit",
        choices=[
            "raw_LS",
            "sg21_LS",
            "sg41_LS",
            "sg21_NNLS",
            "sg21_Stribeck_NLS",
            "sg21_Stribeck_GA",
        ],
        help="gravity only: frozen balanced-v2 fit; default sg21_LS",
    )
    ap.add_argument(
        "--gravity-seconds", type=float, help="gravity only: test duration 10..120s, default30"
    )
    ap.add_argument(
        "--gravity-stiffness",
        type=float,
        help="gravity only: final weak position gain fraction 0..1; 0 gives gravity+damping",
    )


def add_trial_options(ap):
    "Add fixed experiment and trajectory options."
    ap.add_argument(
        "--reciprocal-speeds",
        type=float,
        nargs="+",
        help="reciprocal only: paired speed caps in rad/s; default 0.04 0.12",
    )
    ap.add_argument(
        "--friction-joints",
        type=int,
        nargs="+",
        help="friction only: joint order, default 1 6 4 5 2 3",
    )
    ap.add_argument(
        "--friction-speeds", type=float, nargs="+", help="friction only: speed levels in rad/s"
    )
    ap.add_argument("--friction-repeats", type=int, help="friction only: repeats per speed")
    ap.add_argument(
        "--friction-start",
        choices=["current", "standing"],
        default="standing",
        help="friction only: start and return at the measured pose (default) or at the standing pose",
    )
    ap.add_argument(
        "--capture-seconds",
        type=float,
        help="validation only: stationary recording duration, default 6",
    )
    ap.add_argument(
        "--settle-seconds",
        type=float,
        help="validation only: extra stable time before recording, default 2",
    )
    ap.add_argument(
        "--payload-point-m",
        type=float,
        nargs=3,
        help="optional hanging point xyz in link6 coordinates, metres; do not guess",
    )
    ap.add_argument(
        "--seed",
        type=int,
        help="reproducible multisine trajectory seed; original experiment used 0",
    )
    ap.add_argument("--candidate", help="frozen optimized multisine candidate, e.g. smooth-v1")
    ap.add_argument(
        "--candidate-bank",
        type=Path,
        default=BASE.parent.parent / "identification/trajectories/candidates-v1.json",
    )
    ap.add_argument(
        "--speed-scale",
        type=float,
        help="test playback-speed fraction (0,1]; 0.5 doubles test/ramp duration, preserving motion range",
    )


def add_runtime_options(ap):
    "Add interaction, output and timeout options."
    ap.add_argument(
        "--scale",
        type=float,
        help="test displacement fraction (0,1]; preparation, return, time and frequency unchanged",
    )
    ap.add_argument(
        "--preflight-only",
        action="store_true",
        help="validate the hardware session without reading button actions or enabling motors",
    )
    ap.add_argument(
        "--headless",
        action="store_true",
        help="simulation only: skip the preview window for automated checks",
    )
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument(
        "--auto", action="store_true", help="自动 PREPARE→START，跑完自行回位失能；不需要手柄"
    )
    ap.add_argument(
        "--raw-can",
        action="store_true",
        help="merge: also log every CAN frame to raw_can.jsonl (large)",
    )
    ap.add_argument(
        "--csv",
        action="store_true",
        help="merge: also export test_signals.csv and test_acceleration.csv",
    )
    ap.add_argument(
        "--scripted",
        choices=["normal", "abort", "speed-stop"],
        help="simulation only; speed-stop injects speed feedback to preview a controlled stop",
    )
    ap.add_argument("--output", type=Path, default=BASE.parent / "runs")
    ap.add_argument("--max-seconds", type=float, default=120)


def validate_arguments(a, ap):
    "Reject invalid mode combinations before reading config or opening CAN."
    if a.headless:
        if not a.sim:
            ap.error("--headless is simulation-only")
        a.viewer = False
    if a.scripted and (not a.sim):
        ap.error("--scripted is simulation-only")
    if not np.isfinite(a.max_seconds) or a.max_seconds <= 0:
        ap.error("--max-seconds must be positive and finite")
    if a.viewer and (not a.sim):
        ap.error("--viewer is simulation-only")
    if a.standing and a.from_current:
        ap.error("--standing and --from-current cannot be combined")
    if a.standing and (not (a.hardware or a.sim)):
        ap.error("--standing requires --hardware or --sim")
    if a.from_current and (not (a.hardware or a.sim)):
        ap.error("--from-current requires --hardware or --sim")
    if a.preflight_only and (not a.hardware):
        ap.error("--preflight-only requires --hardware")
    if a.recover and (not a.supported):
        ap.error(
            "恢复会立即失电：先支撑好蜷缩机械臂，再使用 recover --supported 明确确认；不会自动使能。"
        )
    if a.supported and (not a.recover):
        ap.error("--supported is only for --recover")


def handle_input_only(a):
    "Handle pad calibration/monitoring without constructing a robot session."
    if a.calibrate_pad:
        calibrate(a.pad)
        return True
    if a.monitor_pad:
        pad = Gamepad(a.pad)
        print("只读手柄验证：LB+X / LB+Y / LB+RB；不连接 CAN。", flush=True)
        end = time.monotonic() + a.max_seconds
        try:
            while time.monotonic() < end:
                action = pad.poll(time.monotonic())
                if action:
                    print(action, flush=True)
                time.sleep(0.005)
        except KeyboardInterrupt:
            print("手柄监视已结束。", flush=True)
        finally:
            pad.close()
        return True
    return False
