"""Compatibility alias for the existing excitation generator."""

import importlib
import sys

importlib.import_module("real_robot")
sys.modules[__name__] = importlib.import_module("piper_dynamics.core.excitation")
