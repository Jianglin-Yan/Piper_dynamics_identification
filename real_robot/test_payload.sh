#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
python="${PIPER_PYTHON:-/home/shin/anaconda3/envs/lerobot/bin/python}"
case "${1:-sim}" in
  sim) shift; exec env -u LD_PRELOAD "$python" -m real_robot.run_movej --sim --trial payload --max-seconds 3600 "$@" ;;
  hardware) shift; exec env -u LD_PRELOAD "$python" -m real_robot.run_movej --hardware --trial payload --max-seconds 3600 "$@" ;;
  *) printf '%s\n' 'Usage: bash real_robot/test_payload.sh [sim|hardware]'; exit 2 ;;
esac
