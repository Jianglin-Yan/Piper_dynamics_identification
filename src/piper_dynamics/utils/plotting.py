"""Matplotlib plots comparing measured and predicted joint torques."""

from __future__ import annotations

from pathlib import Path
from typing import Sequence

import numpy as np

JOINT_LABELS = tuple(f"J{i}" for i in range(1, 7))


def plot_torque_comparison(
    time_s: np.ndarray,
    measured_Nm: np.ndarray,
    predicted_Nm: np.ndarray,
    output_path: Path,
    *,
    title: str = "",
    joint_labels: Sequence[str] = JOINT_LABELS,
    figure_size: tuple[float, float] = (26.0, 4.6),
    dpi: int = 150,
    line_width: float = 0.9,
    annotate_rmse: bool = True,
) -> Path:
    """Plot measured vs predicted torque for all six joints on one row.

    Args:
        time_s: Time vector of shape ``(N,)``.
        measured_Nm: Measured (target) torque of shape ``(N, 6)``.
        predicted_Nm: Predicted torque of shape ``(N, 6)``.
        output_path: PNG destination; parent directories are created.
        title: Optional figure title.
        joint_labels: Subplot titles, one per joint.
        figure_size: Figure size in inches; the default is one wide row.
        dpi: Raster resolution.
        line_width: Curve line width.
        annotate_rmse: Append the per-joint RMSE to each subplot title.

    Returns:
        The written PNG path.

    Raises:
        ValueError: on shape mismatches.
        ImportError: if matplotlib is not installed.
    """
    time = np.asarray(time_s, dtype=float).reshape(-1)
    measured = np.asarray(measured_Nm, dtype=float)
    predicted = np.asarray(predicted_Nm, dtype=float)
    if measured.ndim != 2 or measured.shape[1] != 6:
        raise ValueError("measured_Nm must have shape (N, 6)")
    if predicted.shape != measured.shape:
        raise ValueError("predicted_Nm and measured_Nm must have the same shape")
    if time.size != measured.shape[0]:
        raise ValueError("time_s length must match the number of samples")

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(1, 6, figsize=figure_size)
    for joint, axis in enumerate(axes):
        axis.plot(time, measured[:, joint], color="black", lw=line_width,
                  label="measured (target)")
        axis.plot(time, predicted[:, joint], color="tab:red", lw=line_width,
                  alpha=0.85, label="predicted")
        subtitle = joint_labels[joint]
        if annotate_rmse:
            rmse = float(np.sqrt(np.mean((predicted[:, joint] - measured[:, joint]) ** 2)))
            subtitle = f"{subtitle}  RMSE {rmse:.3f} N·m"
        axis.set_title(subtitle, fontsize=9)
        axis.grid(alpha=0.25, lw=0.5)
        axis.tick_params(labelsize=7)
        axis.set_xlabel("time (s)", fontsize=8)
        if joint == 0:
            axis.set_ylabel("torque (N·m)", fontsize=9)
            axis.legend(fontsize=7, loc="upper right")
    if title:
        figure.suptitle(title, fontsize=12)
    figure.tight_layout(rect=(0, 0, 1, 0.94) if title else (0, 0, 1, 1))
    figure.savefig(output_path, dpi=dpi)
    plt.close(figure)
    return output_path
