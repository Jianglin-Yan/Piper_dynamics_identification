#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python="${PIPER_PYTHON:-/home/shin/anaconda3/envs/lerobot/bin/python}"
case "${1:-sim}" in
  sim) shift; exec env -u LD_PRELOAD "$python" -m real_robot.run_movej --sim --trial reciprocal --max-seconds 600 "$@" ;;
  hardware) shift; exec env -u LD_PRELOAD "$python" -m real_robot.run_movej --hardware --trial reciprocal --max-seconds 600 "$@" ;;
  *) printf '%s\n' 'Usage: bash real_robot/test_reciprocal.sh [sim|hardware]'; exit 2 ;;
esac
