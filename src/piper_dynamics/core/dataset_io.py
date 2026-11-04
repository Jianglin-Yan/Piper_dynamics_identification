"""Loading and validation of archived measurement ``.npz`` files."""

from __future__ import annotations

from pathlib import Path

import numpy as np

REQUIRED_KEYS = ("t", "q", "qd", "qdd", "cycle", "target_Nm")


def load_dataset(path: Path) -> dict[str, np.ndarray]:
    """Load the measurement keys required by the identification pipeline.

    Raises:
        ValueError: if a required key is missing or an array has the wrong shape.
    """
    with np.load(path) as archive:
        missing = set(REQUIRED_KEYS) - set(archive.files)
        if missing:
            raise ValueError(f"{path}: missing keys {sorted(missing)}")
        data = {name: archive[name] for name in REQUIRED_KEYS}
    n = len(data["q"])
    if any(data[name].shape != (n, 6) for name in ("q", "qd", "qdd", "target_Nm")):
        raise ValueError(f"{path}: q, qd, qdd and target_Nm must have shape (N, 6)")
    if any(data[name].shape != (n,) for name in ("t", "cycle")):
        raise ValueError(f"{path}: t and cycle must have shape (N,)")
    return data


def load_archive(path: Path) -> dict[str, np.ndarray]:
    """Load every array in the file, after validating the required keys.

    The extra keys (for example ``current_A``) are preserved when the raw
    acquisition is copied into ``inputs/train`` and ``inputs/validation``.
    """
    load_dataset(path)
    with np.load(path) as archive:
        return {name: archive[name] for name in archive.files}
