#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source "$(dirname "${BASH_SOURCE[0]}")/python_env.sh"
case "${1:-sim}" in
  pad) shift; exec env -u LD_PRELOAD "$piper_python" -m piper_dynamics.real_robot.app --monitor-pad --max-seconds 300 "$@" ;;
  sim) if [ "$#" -gt 0 ]; then shift; fi; exec env -u LD_PRELOAD "$piper_python" -m piper_dynamics.real_robot.app --sim --viewer --max-seconds 300 "$@" ;;
  read) shift; exec env -u LD_PRELOAD "$piper_python" -m piper_dynamics.real_robot.app --read-only "$@" ;;
  recover) shift; exec env -u LD_PRELOAD "$piper_python" -m piper_dynamics.real_robot.app --recover "$@" ;;
  hardware) shift; exec env -u LD_PRELOAD "$piper_python" -m piper_dynamics.real_robot.app --hardware --max-seconds 300 "$@" ;;
  *) printf '%s\n' 'Usage: bash scripts/pilot.sh [pad|sim|read|recover --supported|hardware]'; exit 2 ;;
esac
