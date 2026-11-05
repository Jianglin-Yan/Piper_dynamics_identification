"""Small shared numerical input validators."""

from __future__ import annotations

import numpy as np


def finite_vector(value, name: str, size: int, lower: float, upper: float) -> np.ndarray:
    """Validate shape, finiteness and inclusive bounds without clipping."""
    result = np.asarray(value, dtype=float)
    if (result.shape != (size,) or not np.all(np.isfinite(result)) or
            np.any(result < lower) or np.any(result > upper)):
        raise ValueError(f"{name} must contain {size} finite values in [{lower}, {upper}]")
    return result
