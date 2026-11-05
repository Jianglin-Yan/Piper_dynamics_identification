"""Inspect or run identified-model drag control. Hardware is opt-in."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from .controller import DragController, DragSettings
from .model import IdentifiedTorqueModel


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fit", type=Path, help="final fit npz; defaults to current split")
    parser.add_argument("--q", nargs=6, type=float, metavar="RAD", help="offline gravity estimate at six joint angles")
    parser.add_argument("--live", action="store_true", help="send MIT commands to a real arm")
    parser.add_argument("--config", type=Path, help="validated per-joint settings JSON for live mode")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--protocol", choices=("v2", "v3"), default="v3")
    parser.add_argument("--rate", type=float, default=50, help="requested command rate in Hz")
    parser.add_argument("--acknowledge-uncalibrated-torque", action="store_true",
                        help="explicitly accept that the identification torque scale is uncalibrated")
    args = parser.parse_args()
    if args.live == (args.q is not None):
        parser.error("choose exactly one of --q (offline) or --live")
    model = IdentifiedTorqueModel(args.fit)
    if args.q is not None:
        print(json.dumps({"fit": str(model.path), "q_rad": args.q,
                          "gravity_candidate_Nm": model.gravity(np.array(args.q)).tolist(),
                          "torque_calibrated": model.torque_calibrated}, indent=2))
        return
    if args.config is None:
        parser.error("--live requires --config")
    if not np.isfinite(args.rate) or not 0 < args.rate <= 100:
        parser.error("--rate must be in (0, 100] Hz")
    if not model.torque_calibrated and not args.acknowledge_uncalibrated_torque:
        parser.error("fit torque is uncalibrated; live mode requires explicit acknowledgement")
    settings = DragSettings(**json.loads(args.config.read_text()))
    controller = DragController(model, settings)
    from .piper_io import PiperMITIO

    io = PiperMITIO(args.can, args.protocol)
    period = 1.0 / args.rate
    entered_mit = False
    try:
        # Obtain fresh feedback and compute the first bounded command before
        # switching control mode; no old position target is ever sent.
        startup_deadline = time.monotonic() + 2.0
        while True:
            try:
                first = io.read(settings.max_feedback_age_s)
                break
            except RuntimeError as exc:
                if str(exc) != "waiting for joint and motor feedback" or time.monotonic() >= startup_deadline:
                    raise
                time.sleep(min(period, 0.02))
        command = controller.command(first.q, first.qd)
        io.enter_mit()
        entered_mit = True
        while True:
            start = time.monotonic()
            feedback = io.read(settings.max_feedback_age_s)
            command = controller.command(feedback.q, feedback.qd)
            if time.monotonic() - feedback.received_at > settings.max_feedback_age_s:
                raise RuntimeError("control computation exceeded feedback age limit")
            io.send(command)
            remaining = period - (time.monotonic() - start)
            if remaining > 0:
                time.sleep(remaining)
    finally:
        if entered_mit:
            io.stop()


if __name__ == "__main__":
    main()
