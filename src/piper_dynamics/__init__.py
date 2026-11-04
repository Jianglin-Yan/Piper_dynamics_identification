"""Offline dynamics parameter identification for the standard PiPER arm.

The package is organised by responsibility:

``paths``
    Filesystem layout of the repository.
``core``
    Reusable building blocks: archive loading, the rigid-body regressor,
    regression features/scaling, the truncated-SVD solver and reporting.
``pipeline``
    The standard PiPER workflow: trajectory-level split and end-to-end fit.
``legacy``
    Archived PiPER X recomputation scripts. They are kept for reference only
    and are not runnable with the current inputs.

Run the workflow with ``python -m piper_dynamics <command>``; see
``python -m piper_dynamics --help``.
"""

__version__ = "1.0.0"
