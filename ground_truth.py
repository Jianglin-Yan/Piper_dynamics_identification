"""Ground-truth inertial parameters, read from the MuJoCo model.

MuJoCo stores ``body_mass``, ``body_ipos`` (COM in the body frame),
``body_inertia`` (diagonal, in the ``body_iquat`` frame) and ``body_iquat``.
The regressor in :mod:`regressor` expects, per link frame,

    theta = [ m, m*c, I_o(6) ] ,  c = COM,  I_o = inertia about the frame origin

Bodies rigidly attached after ``link6`` (flange, gripper base and the two
fingers, which do not move during identification) are lumped into link 6 with
the parallel-axis theorem.
"""

from __future__ import annotations

from typing import Dict, List, Sequence

import numpy as np

from .regressor import quat_to_mat

#: bodies lumped into link6 (all rigid while the gripper is held open)
LINK6_SUBTREE = ("link6", "flange_link", "gripper_base", "left_finger", "right_finger")


def _body_inertia_world(model, data, bid: int, R_b: np.ndarray, p_b: np.ndarray):
    """Return (mass, com_world, inertia_about_com_world) for one body."""
    m = float(model.body_mass[bid])
    R_iq = quat_to_mat(model.body_iquat[bid])
    I_diag = np.diag(np.asarray(model.body_inertia[bid], dtype=float))
    I_com = R_b @ R_iq @ I_diag @ R_iq.T @ R_b.T
    com = p_b + R_b @ np.asarray(model.body_ipos[bid], dtype=float)
    return m, com, I_com


def _lump(model, data, body_ids: Sequence[int]):
    """Combine several rigid bodies into one (mass, com, inertia) in world."""
    M = 0.0
    C = np.zeros(3)
    entries = []
    for bid in body_ids:
        R_b = np.asarray(data.xmat[bid], dtype=float).reshape(3, 3)
        p_b = np.asarray(data.xpos[bid], dtype=float)
        m, com, I_com = _body_inertia_world(model, data, bid, R_b, p_b)
        M += m
        C += m * com
        entries.append((m, com, I_com))
    C /= M
    I_C = np.zeros((3, 3))
    for m, com, I_com in entries:
        d = com - C
        I_C += I_com + m * (d @ d * np.eye(3) - np.outer(d, d))
    return M, C, I_C


def _theta_from(M: float, C: np.ndarray, I_C: np.ndarray) -> np.ndarray:
    """Build [m, h, I_o] from mass, COM and inertia-about-COM (same frame)."""
    h = M * C
    I_o = I_C + M * (C @ C * np.eye(3) - np.outer(C, C))
    return np.array([
        M, h[0], h[1], h[2],
        I_o[0, 0], I_o[1, 1], I_o[2, 2],
        I_o[0, 1], I_o[0, 2], I_o[1, 2],
    ])


def ground_truth_theta(model, qpos: Sequence[float] | None = None) -> np.ndarray:
    """Return the (6, 10) ground-truth parameter matrix in the link frames."""
    import mujoco

    data = mujoco.MjData(model)
    if qpos is not None:
        for i, jn in enumerate(("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            data.qpos[model.jnt_qposadr[jid]] = float(qpos[i])
    mujoco.mj_forward(model, data)

    theta = np.zeros((6, 10))
    for i in range(5):                       # link1 .. link5
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"link{i + 1}")
        R_b = np.asarray(data.xmat[bid], dtype=float).reshape(3, 3)
        p_b = np.asarray(data.xpos[bid], dtype=float)
        m, com, I_com = _body_inertia_world(model, data, bid, R_b, p_b)
        c_local = R_b.T @ (com - p_b)        # COM in the link frame
        I_local = R_b.T @ I_com @ R_b
        theta[i] = _theta_from(m, c_local, I_local)

    # link6 + rigidly attached gripper parts
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n) for n in LINK6_SUBTREE]
    R6 = np.asarray(data.xmat[ids[0]], dtype=float).reshape(3, 3)
    p6 = np.asarray(data.xpos[ids[0]], dtype=float)
    M, C_world, I_C_world = _lump(model, data, ids)
    theta[5] = _theta_from(M, R6.T @ (C_world - p6), R6.T @ I_C_world @ R6)
    return theta


def describe(model) -> List[Dict[str, float]]:
    """Human-readable table of the ground-truth parameters."""
    theta = ground_truth_theta(model)
    rows = []
    for i, th in enumerate(theta):
        rows.append({
            "link": f"link{i + 1}",
            "mass": th[0],
            "com_x": th[1] / th[0] if th[0] else 0.0,
            "com_y": th[2] / th[0] if th[0] else 0.0,
            "com_z": th[3] / th[0] if th[0] else 0.0,
            "Ixx": th[4], "Iyy": th[5], "Izz": th[6],
            "Ixy": th[7], "Ixz": th[8], "Iyz": th[9],
        })
    return rows


if __name__ == "__main__":  # pragma: no cover
    import mujoco

    from agx_piper_sim.sim.simulation import DEFAULT_SCENE

    mdl = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    for r in describe(mdl):
        print(f"{r['link']}: m={r['mass']:.4f} c=({r['com_x']:+.4f},{r['com_y']:+.4f},{r['com_z']:+.4f}) "
              f"I=({r['Ixx']:.2e},{r['Iyy']:.2e},{r['Izz']:.2e})")
