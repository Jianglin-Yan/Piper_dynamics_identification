"""Check that the archived inverse-dynamics model matches the PiPER X inputs."""

import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from .paths import MODELS, REFERENCE


EXPECTED_HASHES = {
    "piper_x.urdf": "0477a703b8f7d8bf2b6638c2d34eff5597051c26d171c9b9172ba8f1183788e5",
    "nominal_x.json": "bc54e8c13a7cf65209f9907bd348893d09eb816ed3828972cdf0ebb472e49d02",
    "real_robot/simulation.py": "89b4b527d55a287b52311273fb941c59b50d651d0298b074071a97e11b204aa6",
    "real_robot/identify_offline.py": "4f27fcf7f14c96a54934692bfc900896938b468bd4d48d5f2967428689e8f548",
    "real_robot/protocol_read.py": "22d112575a6bd5458dffb9c49ca5d9e5f4fec93f95bbb15d8bc05dbde469ed85",
    "regressor.py": "cfe9ba16343adc2008df4aed936657ebcdd064795575b04a591dc53544bcec59",
    "identify.py": "f54f8a79b6d629a6dc206a47b6f9bbc95682f9c491c8b095339385dfc6e0169d",
}


def _source_path(name: str) -> Path:
    return (MODELS if name in {"piper_x.urdf", "nominal_x.json"} else REFERENCE) / name


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _quat_to_matrix(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quat, dtype=float) / np.linalg.norm(quat)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def model_alignment_audit() -> dict:
    hashes = {name: _sha256(_source_path(name)) for name in EXPECTED_HASHES}
    for name, expected in EXPECTED_HASHES.items():
        if hashes[name] != expected:
            raise RuntimeError(f"Archived model dependency changed: {name}")

    sys.path.insert(0, str(REFERENCE))
    import real_robot
    if Path(real_robot.__file__).resolve().parent != (REFERENCE / "real_robot").resolve():
        raise RuntimeError("real_robot resolved outside the archived reference package")
    from real_robot.simulation import model_xml

    nominal = json.loads((MODELS / "nominal_x.json").read_text())
    urdf = ET.parse(MODELS / "piper_x.urdf").getroot()
    model = mujoco.MjModel.from_xml_string(model_xml())
    checked = []
    for i in range(1, 7):
        joint_name = f"joint{i}"
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"link{i}")
        urdf_joint = urdf.find(f"joint[@name='{joint_name}']")
        if joint_id < 0 or body_id < 0 or urdf_joint is None:
            raise RuntimeError(f"PiPER X model is missing {joint_name}/link{i}")

        origin = urdf_joint.find("origin")
        xyz = np.fromstring(origin.get("xyz"), sep=" ")
        rpy = np.fromstring(origin.get("rpy"), sep=" ")
        xyzw = Rotation.from_euler("xyz", rpy).as_quat()
        expected_quat = np.r_[xyzw[3], xyzw[:3]]
        axis = np.fromstring(urdf_joint.find("axis").get("xyz"), sep=" ")
        if not np.allclose(model.body_pos[body_id], xyz, atol=1e-10):
            raise RuntimeError(f"{joint_name} origin differs from URDF")
        if abs(float(model.body_quat[body_id] @ expected_quat)) < 1 - 1e-9:
            raise RuntimeError(f"{joint_name} orientation differs from URDF")
        if not np.allclose(model.jnt_axis[joint_id], axis, atol=1e-10):
            raise RuntimeError(f"{joint_name} axis differs from URDF")

        if i == 6:
            link = nominal["link6_plus_fixed_gripper"]
            com = link["com_m_in_link6_frame"]
            inertia = np.asarray(link["inertia_com_kg_m2_in_link6_axes"])
        else:
            link = nominal["links"][f"link{i}"]
            com = link["com_m_in_link_frame"]
            inertia = np.asarray(link["inertia_com_kg_m2_in_link_axes"])
        if not np.isclose(model.body_mass[body_id], link["mass_kg"], rtol=0, atol=1e-12):
            raise RuntimeError(f"link{i} mass differs from nominal_x.json")
        if not np.allclose(model.body_ipos[body_id], com, atol=1e-10):
            raise RuntimeError(f"link{i} COM differs from nominal_x.json")
        rotation = _quat_to_matrix(model.body_iquat[body_id])
        loaded_inertia = rotation @ np.diag(model.body_inertia[body_id]) @ rotation.T
        if not np.allclose(loaded_inertia, inertia, rtol=1e-8, atol=1e-12):
            raise RuntimeError(f"link{i} inertia differs from nominal_x.json")
        checked.append({"joint": joint_name, "urdf_and_nominal_model_match": True})

    return {
        "status": "passed",
        "source_hashes_sha256": hashes,
        "joints": checked,
        "measured_position_mapping": {
            "conversion": "CAN degrees to radians, direct joint index",
            "absolute_zero_and_sign_independently_verified": False,
        },
    }
