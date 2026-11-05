"""Compatibility forwarding to piper_dynamics.real_robot.app."""

import importlib
import sys

implementation = importlib.import_module("piper_dynamics.real_robot.app")
if __name__ == "__main__":
    implementation.main()
else:
    sys.modules[__name__] = implementation
