#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source "$(dirname "${BASH_SOURCE[0]}")/python_env.sh"
case "${1:-sim}" in
  sim) shift; exec env -u LD_PRELOAD "$piper_python" -m piper_dynamics.real_robot.entries.drag --sim --max-seconds 3600 "$@" ;;
  hardware) shift; exec env -u LD_PRELOAD "$piper_python" -m piper_dynamics.real_robot.entries.drag --hardware --max-seconds 3600 "$@" ;;
  *) printf '%s\n' 'Usage: bash scripts/test_drag.sh [sim|hardware]'; exit 2 ;;
esac
