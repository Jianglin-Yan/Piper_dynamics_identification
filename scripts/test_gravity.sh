#!/usr/bin/env bash
set -euo pipefail
mit_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
mit_mode="${1:-sim}"
if [ "$#" -gt 0 ]; then shift; fi
printf '%s\n' '原重力入口已改为全程MIT固定轨迹；默认前馈比例0.25。纯PD请用 test_mit.sh --ff-scale 0。'
exec bash "$mit_dir/test_mit.sh" "$mit_mode" --ff-scale 0.25 "$@"
