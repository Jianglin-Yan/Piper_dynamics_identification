"""Compatibility forwarding to piper_dynamics.real_robot.entries.movej."""
import importlib
import sys

implementation = importlib.import_module('piper_dynamics.real_robot.entries.movej')
if __name__ == '__main__':
    implementation.main()
else:
    sys.modules[__name__] = implementation
