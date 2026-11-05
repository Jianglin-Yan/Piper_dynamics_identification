"""Compatibility forwarding to piper_dynamics.pipeline.trajectory_design."""

import importlib
import sys

implementation = importlib.import_module("piper_dynamics.pipeline.trajectory_design")
if __name__ == "__main__":
    implementation.main()
else:
    sys.modules[__name__] = implementation
