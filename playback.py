"""Play the identification excitation in an interactive MuJoCo viewer.

Run from ``agx_piper_grasp_sim`` with:
    .venv/bin/python -m dynamics_identification.playback
"""

from __future__ import annotations

import argparse
import time
from typing import Sequence

import numpy as np

from agx_piper_sim.sim.simulation import Simulation
from .data_collector import _park_objects
from .excitation import swevers_trajectory


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Play the identification trajectory")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--seed", type=int, default=0,
                        help="identification trajectory seed (default: 0)")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="playback speed relative to real time")
    parser.add_argument("--amplitude-frac", type=float, default=0.6)
    parser.add_argument("--camera", choices=("free", "scene_cam", "top_cam", "wrist_cam"),
                        default="free")
    args = parser.parse_args(argv)
    if args.speed <= 0:
        parser.error("--speed must be positive")

    # GLFW is required for the interactive MuJoCo viewer.
    import os
    os.environ["MUJOCO_GL"] = "glfw"
    import mujoco
    import mujoco.viewer

    sim = Simulation(render=False, gravity_compensation=False)
    viewer = None
    try:
        sim.reset(randomize=False)
        sim._trajectory = None
        _park_objects(sim)
        traj = swevers_trajectory(
            sim.joint_lo, sim.joint_hi, duration=args.duration,
            dt=float(sim.model.opt.timestep), seed=args.seed,
            amplitude_frac=args.amplitude_frac,
        )

        # Bring the robot to the initial reference pose, then let the same
        # position servos used by collection follow each sampled setpoint.
        sim.set_arm_target(traj.q[0])
        for _ in range(round(1.0 / sim.model.opt.timestep)):
            sim.step()

        viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
        if args.camera != "free":
            cam_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_CAMERA, args.camera)
            viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            viewer.cam.fixedcamid = cam_id

        print(f"[playback] seed={args.seed}, duration={args.duration:g}s, "
              f"speed={args.speed:g}x; close the viewer to stop")
        wall_start = time.monotonic()
        for i, target in enumerate(traj.q):
            if not viewer.is_running():
                break
            sim.set_arm_target(target)
            sim.step()
            with viewer.lock():
                viewer.sync()
            # Pace by simulation time, independent of Python/render overhead.
            deadline = wall_start + (i + 1) * (traj.t[1] - traj.t[0]) / args.speed
            delay = deadline - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        print("[playback] finished")
    finally:
        if viewer is not None:
            viewer.close()
        sim.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
