"""Compatibility forwarding to piper_dynamics.real_robot.trials.validation."""

import importlib
import sys

implementation = importlib.import_module("piper_dynamics.real_robot.trials.validation")
if __name__ == "__main__":
    implementation.main()
else:
    sys.modules[__name__] = implementation
