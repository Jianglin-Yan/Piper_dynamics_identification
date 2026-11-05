"""Compatibility forwarding to piper_dynamics.pipeline.export_signals."""

import importlib
import sys

implementation = importlib.import_module("piper_dynamics.pipeline.export_signals")
if __name__ == "__main__":
    implementation.main()
else:
    sys.modules[__name__] = implementation
