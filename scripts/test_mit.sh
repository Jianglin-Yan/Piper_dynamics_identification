#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source "$(dirname "${BASH_SOURCE[0]}")/python_env.sh"
case "${1:-sim}" in
  sim) shift; exec env -u LD_PRELOAD "$piper_python" -m piper_dynamics.real_robot.entries.mit --sim --max-seconds 600 "$@" ;;
  hardware) shift; exec env -u LD_PRELOAD "$piper_python" -m piper_dynamics.real_robot.entries.mit --hardware --max-seconds 600 "$@" ;;
  *) printf '%s\n' 'Usage: bash scripts/test_mit.sh [sim|hardware] --ff-scale 0..1'; exit 2 ;;
esac
