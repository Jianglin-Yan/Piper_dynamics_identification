"""Exact dynamics regressor for the 6-DOF Piper arm.

The joint torque of a rigid multibody system is *linear* in its inertial
parameters, so the manipulator equation can be written as

    tau(q, qd, qdd) = Y(q, qd, qdd) @ theta

Parameterisation -- per link, expressed in the link frame whose origin sits on
the joint axis::

    theta_i = [ m, h_x, h_y, h_z, Ixx, Iyy, Izz, Ixy, Ixz, Iyz ]

    m : link mass
    h : first moment  h = m * c,  c = COM in the link frame
    I : rotational inertia **about the link-frame origin** (NOT about the COM)

The "about the origin" choice is essential.  The Newton-Euler backward pass

    f = m*a + alpha x h + omega x (omega x h)
    n = I_origin*alpha + omega x (I_origin*omega) + h x a

is linear in ``(m, h, I_origin)``.  The inertia about the COM,
``I_com = I_origin - m*(c.c*1 - c c^T)``, is *non-linear* in ``(m, h)``, so a
COM-based parameter vector does not admit a constant regressor.  Physically
meaningful COM values are recovered afterwards with :func:`to_com_params`.

How ``Y`` is obtained
---------------------
``mj_inverse`` implements the recursive Newton-Euler algorithm and is itself
linear in these parameters, hence

    d tau / d theta_k = ( tau(carrier + eps e_k) - tau(carrier) ) / eps

is an exact column of ``Y`` for any ``eps``.  A constant "carrier" inertia is
installed on every arm body while probing so that

* the model never becomes singular (a zero-mass model breaks MuJoCo's
  constraint factorisation), and
* ``carrier + eps*e_k`` always corresponds to a body MuJoCo can represent with
  a positive-definite COM inertia (the parallel-axis shift for a first-moment
  probe subtracts up to ``h.h/m`` from the principal moments, which is why the
  carrier inertia is large).

The carrier is identical for the baseline and for every probe, so its
contribution cancels exactly in the finite difference.  All other links are
held at the carrier too, so only the probed link contributes.

``_self_test`` verifies ``Y @ theta_gt == mj_inverse`` to machine precision.

Gripper handling
----------------
The five bodies distal to ``link6`` (flange, gripper base and both fingers) are
rigid with respect to ``link6`` while the gripper opening is held constant, so
they are lumped into link 6.  Setting every arm body to the carrier before
probing is what makes a link-6 probe measure the columns of that single
equivalent rigid body.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence

import numpy as np

#: names of the 10 base parameters of one link
PARAM_NAMES: tuple[str, ...] = (
    "m", "hx", "hy", "hz", "Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz",
)
N_LINK_PARAMS = len(PARAM_NAMES)

ARM_JOINT_NAMES: tuple[str, ...] = (
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
)

#: Bodies that form one link of the identified model.  The first entry of each
#: tuple owns the joint and defines the link frame.
LINK_BODIES: tuple[tuple[str, ...], ...] = (
    ("link1",),
    ("link2",),
    ("link3",),
    ("link4",),
    ("link5",),
    ("link6", "flange_link", "gripper_base", "left_finger", "right_finger"),
)

#: Inertia installed on every arm body while probing.  Large, so that the
#: parallel-axis shift of a first-moment probe cannot make the COM inertia
#: indefinite.  It cancels exactly in the finite difference, so its magnitude
#: has no influence on the result.
CARRIER: np.ndarray = np.array(
    [0.05, 0.0, 0.0, 0.0, 100.0, 100.0, 100.0, 0.0, 0.0, 0.0], dtype=float
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def quat_to_mat(q: Sequence[float]) -> np.ndarray:
    """MuJoCo (w, x, y, z) -> 3x3 rotation matrix."""
    w, x, y, z = (float(v) for v in q)
    n = np.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-12:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 rotation matrix -> (w, x, y, z) quaternion."""
    R = np.asarray(R, dtype=float)
    tr = float(np.trace(R))
    if tr > 0.0:
        s = np.sqrt(tr + 1.0) * 2.0
        q = (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
             (R[1, 0] - R[0, 1]) / s)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s,
             (R[0, 2] + R[2, 0]) / s)
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s,
             (R[1, 2] + R[2, 1]) / s)
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
             (R[1, 2] + R[2, 1]) / s, 0.25 * s)
    q = np.asarray(q, dtype=float)
    return q / np.linalg.norm(q)


def inertia_6vec(I: np.ndarray) -> np.ndarray:
    """3x3 symmetric matrix -> [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]."""
    return np.array([I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2]], dtype=float)


def inertia_mat(v: Sequence[float]) -> np.ndarray:
    """[Ixx, Iyy, Izz, Ixy, Ixz, Iyz] -> 3x3 symmetric matrix."""
    v = np.asarray(v, dtype=float)
    return np.array([
        [v[0], v[3], v[4]],
        [v[3], v[1], v[5]],
        [v[4], v[5], v[2]],
    ])


def com_from_params(theta: Sequence[float]) -> np.ndarray:
    """c = h / m (zero when the mass is negligible)."""
    th = np.asarray(theta, dtype=float)
    if abs(th[0]) < 1e-12:
        return np.zeros(3)
    return th[1:4] / th[0]


def to_com_params(theta: Sequence[float]) -> np.ndarray:
    """Convert ``[m, h, I_origin]`` to the physically meaningful ``[m, h, I_com]``."""
    th = np.asarray(theta, dtype=float)
    m = float(th[0])
    h = th[1:4]
    I_o = inertia_mat(th[4:10])
    if abs(m) < 1e-12:
        I_c = I_o
    else:
        I_c = I_o - (float(h @ h) / m * np.eye(3) - np.outer(h, h) / m)
    return np.concatenate([[m], h, inertia_6vec(I_c)])


def from_com_params(theta_com: Sequence[float]) -> np.ndarray:
    """Convert ``[m, h, I_com]`` to ``[m, h, I_origin]``."""
    th = np.asarray(theta_com, dtype=float)
    m = float(th[0])
    h = th[1:4]
    I_c = inertia_mat(th[4:10])
    I_o = I_c + (float(h @ h) / m * np.eye(3) - np.outer(h, h) / m) if abs(m) > 1e-12 else I_c
    return np.concatenate([[m], h, inertia_6vec(I_o)])


# ---------------------------------------------------------------------------
# Builder
# ---------------------------------------------------------------------------
class RegressorBuilder:
    """Builds ``Y`` by probing MuJoCo inverse dynamics.

    The builder owns a private copy of the model, so the live simulation model
    is never touched.
    """

    def __init__(self, model, link_bodies: Sequence[Sequence[str]] = LINK_BODIES,
                 joint_names: Sequence[str] = ARM_JOINT_NAMES, eps: float = 1.0,
                 carrier: Sequence[float] = CARRIER, gripper_open: float = 0.0):
        import mujoco

        self.mujoco = mujoco
        self.model = model
        # Contacts are disabled while probing: identification data is only
        # recorded while the arm touches nothing, and arbitrary probe states
        # must not generate contact impulses.  The finger equality constraint
        # cannot produce torque at the arm DOFs; ``_self_test`` verifies both.
        self.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
        self.data = mujoco.MjData(self.model)

        self.eps = float(eps)
        self.carrier = np.asarray(carrier, dtype=float).copy()
        self.gripper_open = float(gripper_open)

        self.link_bodies = tuple(tuple(names) for names in link_bodies)
        self.link_names = tuple(names[0] for names in self.link_bodies)
        self.n_links = len(self.link_bodies)

        self.joint_ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                          for n in joint_names]
        if any(j < 0 for j in self.joint_ids):
            raise ValueError("missing arm joint in model")
        self.qadr = np.array([self.model.jnt_qposadr[j] for j in self.joint_ids])
        self.dadr = np.array([self.model.jnt_dofadr[j] for j in self.joint_ids])

        self._link_body_ids: List[List[int]] = []
        self._all_body_ids: List[int] = []
        for names in self.link_bodies:
            ids = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, n) for n in names]
            if any(b < 0 for b in ids):
                raise ValueError("missing body in model: %r" % (names,))
            self._link_body_ids.append(ids)
            self._all_body_ids.extend(ids)
        self._all_body_ids = sorted(set(self._all_body_ids))

        self.n_params = self.n_links * N_LINK_PARAMS
        self.param_names = ["%s.%s" % (link, p) for link in self.link_names for p in PARAM_NAMES]

        self._set_fingers()
        self._install_carrier()

    # -- model mutation ---------------------------------------------------
    def _set_fingers(self) -> None:
        for jn, val in (("joint7", self.gripper_open), ("joint8", -self.gripper_open)):
            jid = self.mujoco.mj_name2id(self.model, self.mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                self.data.qpos[self.model.jnt_qposadr[jid]] = float(val)

    def write_params(self, bid: int, theta: Sequence[float]) -> None:
        """Write a ``[m, h, I_origin]`` 10-vector into the MuJoCo body fields."""
        th = np.asarray(theta, dtype=float)
        m = float(th[0])
        h = th[1:4]
        I_o = inertia_mat(th[4:10])
        if abs(m) > 1e-12:
            ipos = h / m
            I_c = I_o - (float(h @ h) / m * np.eye(3) - np.outer(h, h) / m)
        else:
            ipos = np.zeros(3)
            I_c = I_o
        # MuJoCo stores a diagonal inertia in the principal frame; recover it
        # from the eigen-decomposition of the symmetric tensor.  Eigenvalues
        # are kept signed: probe configurations may be non-physical, which is
        # harmless because the model is only used for linear-algebra probing.
        w, V = np.linalg.eigh(I_c)
        if np.linalg.det(V) < 0.0:
            V[:, 0] = -V[:, 0]
        self.model.body_mass[bid] = m
        self.model.body_ipos[bid] = ipos
        self.model.body_inertia[bid] = w
        self.model.body_iquat[bid] = mat_to_quat(V)

    def _install_carrier(self) -> None:
        for bid in self._all_body_ids:
            self.write_params(bid, self.carrier)

    # -- evaluation -------------------------------------------------------
    def set_state(self, q, qd, qdd) -> None:
        self.data.qpos[self.qadr] = np.asarray(q, dtype=float)
        self.data.qvel[:] = 0.0
        self.data.qvel[self.dadr] = np.asarray(qd, dtype=float)
        self.data.qacc[:] = 0.0
        self.data.qacc[self.dadr] = np.asarray(qdd, dtype=float)
        self.data.qfrc_applied[:] = 0.0
        self._set_fingers()

    def inverse_dynamics(self) -> np.ndarray:
        """Arm torques (6,) for the current model parameters and state."""
        self.mujoco.mj_inverse(self.model, self.data)
        return np.array(self.data.qfrc_inverse[self.dadr], dtype=float)

    def build_one(self, q, qd, qdd) -> np.ndarray:
        """Regressor for a single sample: (6, n_params)."""
        self.set_state(q, qd, qdd)
        self._install_carrier()
        tau_base = self.inverse_dynamics()
        Y = np.zeros((6, self.n_params))
        for li, bids in enumerate(self._link_body_ids):
            owner = bids[0]
            for k in range(N_LINK_PARAMS):
                probe = self.carrier.copy()
                probe[k] += self.eps
                self.write_params(owner, probe)
                Y[:, li * N_LINK_PARAMS + k] = (self.inverse_dynamics() - tau_base) / self.eps
            self.write_params(owner, self.carrier)
        return Y

    def build(self, q, qd, qdd, progress: bool = False) -> np.ndarray:
        """Regressor for a batch of samples: (N, 6, n_params)."""
        q = np.atleast_2d(np.asarray(q, dtype=float))
        qd = np.atleast_2d(np.asarray(qd, dtype=float))
        qdd = np.atleast_2d(np.asarray(qdd, dtype=float))
        n = q.shape[0]
        Y = np.zeros((n, 6, self.n_params))
        step = max(1, n // 10)
        for s in range(n):
            Y[s] = self.build_one(q[s], qd[s], qdd[s])
            if progress and (s + 1) % step == 0:
                print("  regressor %d/%d" % (s + 1, n), flush=True)
        return Y


# ---------------------------------------------------------------------------
# Ground truth
# ---------------------------------------------------------------------------
def ground_truth_theta(model, gripper_open: float = 0.0) -> np.ndarray:
    """Ground-truth parameters, shape (n_links, 10), ``[m, h, I_origin]``.

    Bodies distal to ``link6`` are combined into link 6 with the parallel axis
    theorem.  ``gripper_open`` must match the finger opening used during data
    collection, because the finger slide displacement changes the lumped
    inertia of the distal assembly.
    """
    import mujoco

    data = mujoco.MjData(model)
    for jn, val in (("joint7", gripper_open), ("joint8", -gripper_open)):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = float(val)
    mujoco.mj_forward(model, data)

    theta = np.zeros((len(LINK_BODIES), N_LINK_PARAMS))
    for li, names in enumerate(LINK_BODIES):
        bid_owner = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, names[0])
        R_o = np.asarray(data.xmat[bid_owner], dtype=float).reshape(3, 3)
        p_o = np.asarray(data.xpos[bid_owner], dtype=float)

        total_m = 0.0
        com_world = np.zeros(3)
        parts = []
        for name in names:
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            if bid < 0:
                continue
            m = float(model.body_mass[bid])
            R_b = np.asarray(data.xmat[bid], dtype=float).reshape(3, 3)
            p_b = np.asarray(data.xpos[bid], dtype=float)
            R_iq = quat_to_mat(model.body_iquat[bid])
            I_com_world = (R_b @ R_iq
                           @ np.diag(np.asarray(model.body_inertia[bid], dtype=float))
                           @ R_iq.T @ R_b.T)
            com = p_b + R_b @ np.asarray(model.body_ipos[bid], dtype=float)
            total_m += m
            com_world += m * com
            parts.append((m, com, I_com_world))
        if total_m <= 0.0:
            continue
        com_world /= total_m
        I_C_world = np.zeros((3, 3))
        for m, com, I_com in parts:
            d = com - com_world
            I_C_world += I_com + m * (d @ d * np.eye(3) - np.outer(d, d))
        c_local = R_o.T @ (com_world - p_o)
        I_com_local = R_o.T @ I_C_world @ R_o
        # shift from the COM to the link-frame origin
        I_o_local = I_com_local + total_m * (
            float(c_local @ c_local) * np.eye(3) - np.outer(c_local, c_local)
        )
        theta[li] = np.concatenate([[total_m], total_m * c_local, inertia_6vec(I_o_local)])
    return theta


def describe_theta(theta: np.ndarray, link_names: Sequence[str]) -> List[Dict[str, float]]:
    """Human readable rows (mass, COM, inertia about the COM) of a parameter matrix."""
    rows = []
    for name, th in zip(link_names, theta):
        th_c = to_com_params(th)
        c = com_from_params(th_c)
        rows.append({
            "link": name, "mass": float(th_c[0]),
            "com_x": float(c[0]), "com_y": float(c[1]), "com_z": float(c[2]),
            "Ixx": float(th_c[4]), "Iyy": float(th_c[5]), "Izz": float(th_c[6]),
            "Ixy": float(th_c[7]), "Ixz": float(th_c[8]), "Iyz": float(th_c[9]),
        })
    return rows


# ---------------------------------------------------------------------------
# Self test
# ---------------------------------------------------------------------------
def _self_test(n_samples: int = 25, gripper_open: float = 0.0) -> int:
    import mujoco

    from agx_piper_sim.sim.simulation import DEFAULT_SCENE

    model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    theta = ground_truth_theta(model, gripper_open=gripper_open)
    builder = RegressorBuilder(model, gripper_open=gripper_open)

    rng = np.random.default_rng(0)
    lo = np.array([model.jnt_range[j, 0] for j in builder.joint_ids])
    hi = np.array([model.jnt_range[j, 1] for j in builder.joint_ids])
    q = lo + (hi - lo) * rng.uniform(0.1, 0.9, size=(n_samples, 6))
    qd = rng.uniform(-1.5, 1.5, size=(n_samples, 6))
    qdd = rng.uniform(-6.0, 6.0, size=(n_samples, 6))

    Y = builder.build(q, qd, qdd)

    # Reference: the untouched model, contacts disabled (random samples may
    # poke through the table) but the finger equality constraint left ENABLED.
    ref_model = mujoco.MjModel.from_xml_path(str(DEFAULT_SCENE))
    ref_model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    ref_data = mujoco.MjData(ref_model)
    for jn, val in (("joint7", gripper_open), ("joint8", -gripper_open)):
        jid = mujoco.mj_name2id(ref_model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        ref_data.qpos[ref_model.jnt_qposadr[jid]] = float(val)
    tau_ref = np.zeros((n_samples, 6))
    for s in range(n_samples):
        ref_data.qpos[builder.qadr] = q[s]
        ref_data.qvel[:] = 0.0
        ref_data.qvel[builder.dadr] = qd[s]
        ref_data.qacc[:] = 0.0
        ref_data.qacc[builder.dadr] = qdd[s]
        ref_data.qfrc_applied[:] = 0.0
        mujoco.mj_inverse(ref_model, ref_data)
        tau_ref[s] = ref_data.qfrc_inverse[builder.dadr]

    tau_reg = np.einsum("nij,j->ni", Y, theta.reshape(-1))
    err = np.abs(tau_reg - tau_ref)
    scale = max(1e-9, float(np.abs(tau_ref).max()))
    ok = bool(np.all(np.isfinite(Y))) and (err.max() / scale) < 1e-7

    print("self-test: max |Y*theta_gt - mj_inverse| = %.3e N*m (rel %.3e)"
          % (err.max(), err.max() / scale))
    print("           max |tau| = %.3f N*m" % scale)
    print("           per-joint max err:", np.array2string(err.max(axis=0), precision=2))
    print("           result:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(_self_test())
