"""Compatibility forwarding to piper_dynamics.pipeline.run_summary."""
import importlib
import sys

implementation = importlib.import_module('piper_dynamics.pipeline.run_summary')
if __name__ == '__main__':
    implementation.main()
else:
    sys.modules[__name__] = implementation
