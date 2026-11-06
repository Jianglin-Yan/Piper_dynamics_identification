"""零力拖动入口。

LB+X：准备到站立位，Kp/前馈渐变到 5/100%；LB+Y：开始记录；再次 LB+X：
反向渐变并沿 MIT 路径回到停靠位。命令形如
`bash scripts/test_drag.sh [sim|hardware]`。
"""

import sys

FORBIDDEN = ("--trajectory", "--from-current", "--recover", "--read-only")


def build_argv(args):
    for item in args:
        if item == "--trajectory" or item.startswith("--trajectory="):
            raise SystemExit("run_drag 固定使用 drag 轨迹，不接受 --trajectory")
        if item in FORBIDDEN[1:]:
            raise SystemExit("run_drag 不接受 " + item)
        if item in ("--ff-scale", "--mit-gain-scale") or item.startswith(("--ff-scale=", "--mit-gain-scale=")):
            raise SystemExit("零力拖动固定 Kp 5、前馈 100%，不接受 --ff-scale/--mit-gain-scale")
    return ["--standing", "--trajectory", "drag", *args]


def main():
    sys.argv = [sys.argv[0], *build_argv(sys.argv[1:])]
    from piper_dynamics.real_robot.app import main as run_main

    run_main()


if __name__ == "__main__":
    main()
