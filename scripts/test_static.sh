#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python="${PIPER_PYTHON:-/home/shin/anaconda3/envs/lerobot/bin/python}"
case "${1:-sim}" in
  sim) shift; exec env -u LD_PRELOAD "$python" -m real_robot.run_movej --sim --trial static --max-seconds 1200 "$@" ;;
  hardware) shift; exec env -u LD_PRELOAD "$python" -m real_robot.run_movej --hardware --trial static --max-seconds 1200 "$@" ;;
  *) printf '%s\n' 'Usage: bash real_robot/test_static.sh [sim|hardware]'; exit 2 ;;
esac
