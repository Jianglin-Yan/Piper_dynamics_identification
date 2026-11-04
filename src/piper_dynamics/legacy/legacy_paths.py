"""Paths used only by the archived PiPER X recomputation scripts."""

from pathlib import Path

from ..paths import INPUTS, MODELS, OUTPUTS, PROJECT_ROOT

MESH_ROOT = MODELS / "collision_meshes"
#: Frozen source snapshot imported by the archived entry points.
REFERENCE = Path(__file__).resolve().parent / "snapshot"

#: The raw data these scripts expected was removed during the input cleanup.
LEGACY_INPUTS = INPUTS / "removed_legacy_piper_x"
INPUT_RUN = LEGACY_INPUTS / "experiment_20261029"
SPLIT = LEGACY_INPUTS / "splits" / "sg41_selected_train_test_samples.npz"

__all__ = [
    "INPUTS",
    "INPUT_RUN",
    "MESH_ROOT",
    "MODELS",
    "OUTPUTS",
    "PROJECT_ROOT",
    "REFERENCE",
    "SPLIT",
]
