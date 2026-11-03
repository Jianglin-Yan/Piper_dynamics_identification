"""Dynamics parameter identification for the 6-DOF Piper arm in MuJoCo.

Pipeline (see ``README.md``):

    excitation  ->  data_collector  ->  regressor  ->  identify  ->  validate

Only simulated data is used; the URDF parameters embedded in the MuJoCo model
serve as ground truth.
"""

__all__ = ["excitation", "data_collector"]
