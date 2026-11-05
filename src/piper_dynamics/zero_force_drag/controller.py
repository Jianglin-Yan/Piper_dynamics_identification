"""Pure drag-control logic; independent of CAN transport and the SDK."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..utils.validation import finite_vector
from .model import IdentifiedTorqueModel


@dataclass(frozen=True)
class DragSettings:
    """Explicit per-joint bounds; no implicit live-control tuning."""

    kp: tuple[float, ...]
    kd: tuple[float, ...]
    torque_limit_nm: tuple[float, ...]
    compensation_scale: tuple[float, ...]
    max_speed_rad_s: tuple[float, ...]
    max_feedback_age_s: float = 0.1

    def __post_init__(self) -> None:
        finite_vector(self.kp, "kp", 6, 0, 500)
        finite_vector(self.kd, "kd", 6, 0, 5)
        finite_vector(self.torque_limit_nm, "torque_limit_nm", 6, 0, 8)
        finite_vector(self.compensation_scale, "compensation_scale", 6, 0, 2)
        finite_vector(self.max_speed_rad_s, "max_speed_rad_s", 6, 0, 45)
        if not 0 < self.max_feedback_age_s <= 1:
            raise ValueError("max_feedback_age_s must be in (0, 1]")


@dataclass(frozen=True)
class DragCommand:
    position_rad: np.ndarray
    velocity_rad_s: np.ndarray
    kp: np.ndarray
    kd: np.ndarray
    torque_nm: np.ndarray
    raw_gravity_nm: np.ndarray


class DragController:
    """Low stiffness drag: current measured position is the MIT position target.

    Friction fit is deliberately excluded: its offset and smooth Coulomb terms
    can cause self motion near zero speed. Gravity uses the final fit exactly.
    """

    def __init__(self, model: IdentifiedTorqueModel, settings: DragSettings):
        self.model = model
        self.settings = settings

    def command(self, q: np.ndarray, qd: np.ndarray) -> DragCommand:
        q = finite_vector(q, "position feedback", 6, -12.5, 12.5)
        qd = finite_vector(qd, "speed feedback", 6, -45, 45)
        if np.any(q < self.model.joint_range[:, 0]) or np.any(q > self.model.joint_range[:, 1]):
            raise ValueError("joint position is outside the model limits")
        if np.any(np.abs(qd) > self.settings.max_speed_rad_s):
            raise ValueError("joint speed exceeds configured drag limit")
        gravity = self.model.gravity(q)
        limit = np.asarray(self.settings.torque_limit_nm)
        desired = gravity * np.asarray(self.settings.compensation_scale)
        if np.any(np.abs(desired) > limit):
            raise ValueError("gravity compensation exceeds configured torque limit")
        # q_ref tracks measured q, eliminating the fixed-position restoring force.
        return DragCommand(q.copy(), np.zeros(6), np.asarray(self.settings.kp),
                           np.asarray(self.settings.kd), desired, gravity)
