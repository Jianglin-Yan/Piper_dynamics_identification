"""Archived module name for the rigid-body regressor.

The implementation now lives in
:mod:`piper_dynamics.core.rigid_body_regressor`. This module re-exports the
public names so that the frozen PiPER X entry points can keep importing
``.regressor`` unchanged.
"""

from piper_dynamics.core.rigid_body_regressor import (  # noqa: F401
    ARM_JOINT_NAMES,
    CARRIER,
    LINK_BODIES,
    N_LINK_PARAMS,
    PARAM_NAMES,
    RegressorBuilder,
    com_from_params,
    describe_theta,
    from_com_params,
    ground_truth_theta,
    inertia_6vec,
    inertia_mat,
    mat_to_quat,
    quat_to_mat,
    to_com_params,
)

__all__ = [
    "ARM_JOINT_NAMES",
    "CARRIER",
    "LINK_BODIES",
    "N_LINK_PARAMS",
    "PARAM_NAMES",
    "RegressorBuilder",
    "com_from_params",
    "describe_theta",
    "from_com_params",
    "ground_truth_theta",
    "inertia_6vec",
    "inertia_mat",
    "mat_to_quat",
    "quat_to_mat",
    "to_com_params",
]
