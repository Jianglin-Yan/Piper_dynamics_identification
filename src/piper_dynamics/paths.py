"""Project paths shared by the offline identification entry points."""

from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
INPUT_RUN = PROJECT_ROOT / "inputs" / "experiment_20261029"
SPLIT = PROJECT_ROOT / "inputs" / "splits" / "sg41_selected_train_test_samples.npz"
MODELS = PROJECT_ROOT / "models"
MESH_ROOT = MODELS / "collision_meshes"
REFERENCE = Path(__file__).resolve().parent / "reference"
OUTPUTS = PROJECT_ROOT / "outputs"
STANDARD_INPUTS = PROJECT_ROOT / "inputs" / "standard_piper"
STANDARD_MODELS = MODELS / "standard_piper"
STANDARD_OUTPUTS = OUTPUTS / "standard_piper"
