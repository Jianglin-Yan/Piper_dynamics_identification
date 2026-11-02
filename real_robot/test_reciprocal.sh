#!/usr/bin/env bash
set -euo pipefail
trial_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
trial_mode="${1:-sim}"
if [ "$#" -gt 0 ]; then shift; fi
case "$trial_mode" in sim|hardware) ;; *) printf '%s\n' 'Usage: bash real_robot/test_reciprocal.sh [sim|hardware] [options]'; exit 2 ;; esac
for trial_arg in "$@"; do
  case "$trial_arg" in
    --trajectory|--trajectory=*|--from-current|--recover|--read-only|--config|--config=*)
      printf '%s\n' 'This entry uses the fixed reciprocal diagnostic protocol; use its documented options.' >&2; exit 2 ;;
  esac
done
exec bash "$trial_dir/pilot.sh" "$trial_mode" --standing --trajectory reciprocal --max-seconds 600 "$@"
