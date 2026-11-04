"""Filesystem layout of the active standard PiPER identification pipeline.

Paths used only by the archived PiPER X scripts live in
``piper_dynamics.legacy.legacy_paths``.
"""

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
INPUTS = PROJECT_ROOT / "inputs"

RAW_INPUTS = INPUTS / "raw" / "standard_piper"
TRAIN_INPUTS = INPUTS / "train"
VALIDATION_INPUTS = INPUTS / "validation"

MODELS = PROJECT_ROOT / "models"
OUTPUTS = PROJECT_ROOT / "outputs"
STANDARD_MODELS = MODELS / "standard_piper"
STANDARD_OUTPUTS = OUTPUTS / "standard_piper"

#: Backwards-compatible alias for the archived raw data directory.
STANDARD_INPUTS = RAW_INPUTS
