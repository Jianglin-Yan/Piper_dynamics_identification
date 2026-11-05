"""MOVE J 入口（静态／正反／负载／多正弦／pilot）。

由 `--trial` 选择任务；不接受 `--trajectory`，避免绕过固定任务参数。
"""

import sys

TRIALS = ("static", "reciprocal", "payload", "friction", "pilot", "multisine")


def _pop(args, name):
    out = []
    value = None
    i = 0
    while i < len(args):
        item = args[i]
        if item == name:
            if i + 1 >= len(args):
                raise SystemExit(name + " 缺少参数")
            value = args[i + 1]
            i += 2
            continue
        if item.startswith(name + "="):
            value = item.split("=", 1)[1]
            i += 1
            continue
        out.append(item)
        i += 1
    return out, value


def build_argv(args):
    for item in args:
        if item == "--trajectory" or item.startswith("--trajectory="):
            raise SystemExit("run_movej 用 --trial 选择任务，不接受 --trajectory")
    args, trial = _pop(list(args), "--trial")
    trial = trial or "static"
    if trial not in TRIALS:
        raise SystemExit("--trial 只能是 " + "/".join(TRIALS))
    return ["--standing", "--trajectory", trial, *args]


def main():
    sys.argv = [sys.argv[0], *build_argv(sys.argv[1:])]
    from piper_dynamics.real_robot.app import main as run_main

    run_main()


if __name__ == "__main__":
    main()
