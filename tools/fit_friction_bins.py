"""Compatibility launcher for an offline src pipeline."""

import importlib
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
importlib.import_module("real_robot")
main = importlib.import_module("piper_dynamics.pipeline.friction_bins").main
if __name__ == "__main__":
    main()
