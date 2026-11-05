"""Filesystem layout of the active standard PiPER identification pipeline.

Control configuration, models, acquisitions and analysis outputs share this layout.
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

# Public compatibility paths are logical names used in historical fingerprints.
LEGACY_ROBOT_ROOT = PROJECT_ROOT / "real_robot"
ROBOT_PROFILES = Path(__file__).parent / "real_robot" / "profiles"
RUNS = PROJECT_ROOT / "runs"
IDENTIFICATION_ARCHIVE = PROJECT_ROOT.parent / "identification"
SOURCE_MODULES = {
    "run": "real_robot.app",
    "run_mit": "real_robot.entries.mit",
    "run_movej": "real_robot.entries.movej",
    "controller": "real_robot.controller",
    "hardware": "real_robot.backends.hardware",
    "simulation": "real_robot.backends.simulation",
    "validation_simulation": "real_robot.backends.validation_simulation",
    "gravity": "real_robot.control.gravity_hold",
    "mit_trial": "real_robot.control.mit_trial",
    "trajectory": "real_robot.trials.trajectory",
    "multisine": "real_robot.trials.multisine",
    "designed_trajectory": "real_robot.trials.designed_trajectory",
    "validation_trials": "real_robot.trials.validation",
    "friction": "real_robot.trials.friction",
    "design_trajectories": "pipeline.trajectory_design",
    "identify_offline": "pipeline.measured_run",
    "analyze": "pipeline.run_summary",
    "export_signals": "pipeline.export_signals",
    "test_data": "real_robot.recording",
    "protocol_read": "real_robot.protocol_read",
    "shutdown": "real_robot.shutdown",
    "standing": "real_robot.standing",
    "monitoring": "real_robot.monitoring",
    "current_pose": "real_robot.current_pose",
    "gamepad": "real_robot.gamepad",
    "ui": "real_robot.ui",
    "operator_exit": "real_robot.operator_exit",
    "session_history": "real_robot.session_history",
}


def robot_source_path(name: str) -> Path:
    """Resolve an old logical source name to its single current implementation."""
    module = SOURCE_MODULES[Path(name).stem]
    return Path(__file__).parent / (module.replace(".", "/") + ".py")


def model_reference_path(relative: str) -> Path:
    """Resolve historical model labels without rewriting their recorded hashes."""
    known = {
        "assets/piper.urdf": STANDARD_MODELS / "piper.urdf",
        "assets/nominal_piper.json": STANDARD_MODELS / "nominal_piper.json",
    }
    return known.get(relative, LEGACY_ROBOT_ROOT / relative)
