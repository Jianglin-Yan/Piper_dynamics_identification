#!/usr/bin/env bash
set -euo pipefail
mit_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
mit_mode="${1:-sim}"
if [ "$#" -gt 0 ]; then shift; fi
case "$mit_mode" in sim|hardware) ;; *) printf '%s\n' 'Usage: bash real_robot/test_mit.sh [sim|hardware] --ff-scale 0..1 [options]'; exit 2 ;; esac
for mit_arg in "$@"; do
 case "$mit_arg" in --trajectory|--trajectory=*|--from-current|--recover|--read-only|--config|--config=*|--scale|--scale=*|--speed-scale|--speed-scale=*) printf '%s\n' 'This entry uses a fixed MIT-only protocol; compare trials with --ff-scale.' >&2; exit 2 ;; esac
done
exec bash "$mit_dir/pilot.sh" "$mit_mode" --standing --trajectory mit --max-seconds 600 "$@"
