#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
piper_python="${PIPER_PYTHON:-/home/shin/anaconda3/envs/lerobot/bin/python}"
case "${1:-sim}" in
  pad) shift; exec env -u LD_PRELOAD "$piper_python" -m real_robot.run --monitor-pad --max-seconds 300 "$@" ;;
  sim) if [ "$#" -gt 0 ]; then shift; fi; exec env -u LD_PRELOAD "$piper_python" -m real_robot.run --sim --viewer --max-seconds 300 "$@" ;;
  read) shift; exec env -u LD_PRELOAD "$piper_python" -m real_robot.run --read-only "$@" ;;
  hardware) shift; exec env -u LD_PRELOAD "$piper_python" -m real_robot.run --hardware --max-seconds 300 "$@" ;;
  *) printf '%s\n' 'Usage: bash real_robot/pilot.sh [pad|sim|read|hardware]'; exit 2 ;;
esac
