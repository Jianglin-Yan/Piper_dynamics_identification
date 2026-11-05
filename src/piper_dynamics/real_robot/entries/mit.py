"""MIT 固定轨迹入口。

命令仍是你熟悉的 `bash real_robot/test_mit.sh [sim|hardware] --ff-scale 0..1`；
参数拼装集中在这里，shell 只负责转发。
"""
import sys

FORBIDDEN=('--trajectory','--from-current','--recover','--read-only')


def build_argv(args):
    for item in args:
        if item=='--trajectory' or item.startswith('--trajectory='):
            raise SystemExit('run_mit 固定使用 MIT 轨迹，不接受 --trajectory')
        if item in FORBIDDEN[1:]:
            raise SystemExit('run_mit 不接受 '+item+'；请用 pilot.sh 的对应入口')
    return ['--standing','--trajectory','mit',*args]


def main():
    sys.argv=[sys.argv[0],*build_argv(sys.argv[1:])]
    from piper_dynamics.real_robot.app import main as run_main
    run_main()


if __name__=='__main__':main()
