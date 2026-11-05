"""Names, groups and units of the 78-dimensional identification vector.

Layout: 6 links x 10 inertial parameters (``[m, h, I_origin]``), followed by
6 joints x 3 friction parameters.
"""

from __future__ import annotations

from .regression_model import FRICTION_NAMES
from .rigid_body_regressor import PARAM_NAMES

LINK_NAMES = tuple(f"link{i}" for i in range(1, 7))

PARAMETER_NAMES = tuple(f"link{i}.{name}" for i in range(1, 7) for name in PARAM_NAMES) + tuple(
    f"joint{i}.{name}" for i in range(1, 7) for name in FRICTION_NAMES
)

LINK_UNITS = {
    "mass": "kg",
    "com_x": "m",
    "com_y": "m",
    "com_z": "m",
    "Ixx": "kg*m^2",
    "Iyy": "kg*m^2",
    "Izz": "kg*m^2",
    "Ixy": "kg*m^2",
    "Ixz": "kg*m^2",
    "Iyz": "kg*m^2",
}

FRICTION_UNITS = {
    "viscous_friction": "N*m/(rad/s)",
    "smooth_coulomb_friction": "N*m",
    "offset": "N*m",
}
