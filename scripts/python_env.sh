#!/usr/bin/env bash
# Shared setup for the thin project launchers.
piper_repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -n "${PIPER_PYTHON:-}" ]]; then
  piper_python="$PIPER_PYTHON"
elif [[ -x "$piper_repo_root/.venv/bin/python" ]]; then
  piper_python="$piper_repo_root/.venv/bin/python"
elif [[ -x "$HOME/anaconda3/envs/lerobot/bin/python" ]]; then
  piper_python="$HOME/anaconda3/envs/lerobot/bin/python"
else
  piper_python="$(command -v python3)"
fi
export PYTHONPATH="$piper_repo_root/src${PYTHONPATH:+:$PYTHONPATH}"
