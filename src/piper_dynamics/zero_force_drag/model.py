"""Evaluate the final identification result without treating fitted inertias as physical."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from ..core.parameter_schema import PARAMETER_NAMES
from ..core.rigid_body_regressor import RegressorBuilder
from ..paths import STANDARD_MODELS
from ..pipeline.standard_piper_pipeline import LINK_BODIES, default_fit_path, load_model
from ..utils.hashing import sha256


class IdentifiedTorqueModel:
    """Gravity torque from the fitted 60 rigid parameters at zero velocity.

    The identified inertias need not be individually physical. Evaluating the
    existing linear regressor preserves the fitted base parameter combination.
    """

    def __init__(self, fit_path: Path | None = None):
        self.path = Path(fit_path or default_fit_path()).resolve()
        report_path = self.path.with_suffix(".json")
        report = json.loads(report_path.read_text())
        with np.load(self.path, allow_pickle=False) as archive:
            self.theta = np.asarray(archive["theta"], dtype=float).copy()
            names = tuple(str(x) for x in archive["parameter_names"])
        if self.theta.shape != (78,) or names != PARAMETER_NAMES or not np.all(np.isfinite(self.theta)):
            raise ValueError("fit has an invalid or incompatible 78-parameter vector")
        if report.get("robot") != "standard PiPER":
            raise ValueError("fit is not for the standard PiPER")
        model_path = STANDARD_MODELS / "piper.xml"
        if report.get("model_sha256") != sha256(model_path):
            raise ValueError("fit model hash differs from current standard PiPER XML")
        self.torque_calibrated = bool(report.get("torque_calibrated", False))
        self.builder = RegressorBuilder(load_model(), link_bodies=LINK_BODIES)
        self.joint_range = np.asarray(
            self.builder.model.jnt_range[self.builder.joint_ids], dtype=float
        ).copy()

    def gravity(self, q: np.ndarray) -> np.ndarray:
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.all(np.isfinite(q)):
            raise ValueError("q must contain six finite joint angles in radians")
        zeros = np.zeros(6)
        rigid = self.builder.build_one(q, zeros, zeros)
        result = rigid @ self.theta[:60]
        if not np.all(np.isfinite(result)):
            raise ValueError("nonfinite gravity prediction")
        return result
