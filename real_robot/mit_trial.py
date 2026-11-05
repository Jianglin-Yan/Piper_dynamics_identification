"""Compatibility forwarding to piper_dynamics.real_robot.control.mit_trial."""

import importlib
import sys

implementation = importlib.import_module("piper_dynamics.real_robot.control.mit_trial")
if __name__ == "__main__":
    implementation.main()
else:
    sys.modules[__name__] = implementation
