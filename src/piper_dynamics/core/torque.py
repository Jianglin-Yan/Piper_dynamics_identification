"""Standard PiPER reference conversion; not individually calibrated torque."""

import numpy as np

K = np.array([1.18125] * 3 + [0.9576] * 3)
B = np.array([4.0] * 3 + [1.0] * 3)
C = np.array([1.0] * 3 + [0.813252] * 3)
NM_PER_A = K * B * C
WIRE_TO_JOINT = B * C
