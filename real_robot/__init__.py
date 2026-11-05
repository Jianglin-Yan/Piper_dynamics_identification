"""Compatibility entry point for existing real_robot callers.

The canonical source package lives under src; editable installs and PYTHONPATH=src
load it normally. This fallback keeps existing project-root commands usable.
"""
import importlib.util
import sys
from pathlib import Path

if 'piper_dynamics' not in sys.modules:
    package_root = Path(__file__).resolve().parent.parent / 'src/piper_dynamics'
    spec = importlib.util.spec_from_file_location(
        'piper_dynamics', package_root / '__init__.py',
        submodule_search_locations=[str(package_root)])
    package = importlib.util.module_from_spec(spec)
    sys.modules['piper_dynamics'] = package
    spec.loader.exec_module(package)
