"""Compatibility forwarding to piper_dynamics.real_robot.current_pose."""
import importlib
import sys

implementation = importlib.import_module('piper_dynamics.real_robot.current_pose')
if __name__ == '__main__':
    implementation.main()
else:
    sys.modules[__name__] = implementation
