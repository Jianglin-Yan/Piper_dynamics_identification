#!/usr/bin/env python3
"""Fail-closed runner for the EXP-20261029-001 PiPER X offline fit.

All model data are taken from the immutable code snapshot next to this script.
The old project simulation model is never imported. The archived identification
implementation is reused only after the PiPER X model and source paths pass the
checks below.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
EXPERIMENT = HERE.parent
SNAPSHOT = EXPERIMENT / "provenance" / "code_snapshot"
REAL_ROBOT = SNAPSHOT / "real_robot"
ASSETS = REAL_ROBOT / "assets"

EXPECTED_HASHES = {
    "real_robot/assets/piper_x.urdf": "0477a703b8f7d8bf2b6638c2d34eff5597051c26d171c9b9172ba8f1183788e5",
    "real_robot/assets/nominal_x.json": "bc54e8c13a7cf65209f9907bd348893d09eb816ed3828972cdf0ebb472e49d02",
    "real_robot/simulation.py": "89b4b527d55a287b52311273fb941c59b50d651d0298b074071a97e11b204aa6",
    "real_robot/identify_offline.py": "4f27fcf7f14c96a54934692bfc900896938b468bd4d48d5f2967428689e8f548",
    "real_robot/protocol_read.py": "22d112575a6bd5458dffb9c49ca5d9e5f4fec93f95bbb15d8bc05dbde469ed85",
    "regressor.py": "cfe9ba16343adc2008df4aed936657ebcdd064795575b04a591dc53544bcec59",
    "identify.py": "f54f8a79b6d629a6dc206a47b6f9bbc95682f9c491c8b095339385dfc6e0169d",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_parameter_csv(out_dir: Path):
    """Export the training-only vectors in the same readable schema as the split fit."""
    path = out_dir / "exploratory_parameters_NOT_FOR_CONTROL.npz"
    with np.load(path) as data:
        inertial_names = [str(x) for x in data["param_names"]]
        friction_names = [
            f"joint{j}.{term}"
            for j in range(1, 7)
            for term in ("viscous_friction", "smooth_coulomb_friction", "offset")
        ]
        names = {
            "rigid_only_original_LS": inertial_names,
            "rigid_plus_friction_offset_LS": inertial_names + friction_names,
        }
        with (out_dir / "training_parameters.csv").open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["fit_id", "index", "parameter", "unit", "theta_train", "selected_by_qr"])
            for fit_id, parameter_names in names.items():
                theta = data[f"{fit_id}_theta_train"]
                selected = set(int(i) for i in data[f"{fit_id}_base_idx"])
                for i, (parameter, value) in enumerate(zip(parameter_names, theta)):
                    if parameter.endswith(".m"):
                        unit = "kg"
                    elif any(parameter.endswith(f".{x}") for x in ("hx", "hy", "hz")):
                        unit = "kg*m"
                    elif parameter.endswith("viscous_friction"):
                        unit = "N*m*s/rad"
                    elif ("friction" in parameter or parameter.endswith("offset")):
                        unit = "N*m"
                    else:
                        unit = "kg*m^2"
                    writer.writerow([fit_id, i, parameter, unit, format(float(value), ".17g"), i in selected])


def model_alignment_audit():
    """Check loaded MJCF model against this snapshot's URDF and nominal X data."""
    for rel, expected in EXPECTED_HASHES.items():
        actual = sha256(SNAPSHOT / rel)
        if actual != expected:
            raise RuntimeError(f"Unexpected model source hash for {rel}: {actual}")

    # Resolve and pin real_robot before importing any model or fit modules.
    sys.path.insert(0, str(SNAPSHOT))
    import real_robot
    resolved_package = Path(real_robot.__file__).resolve().parent
    if resolved_package != REAL_ROBOT.resolve():
        raise RuntimeError(f"real_robot resolved outside the snapshot: {resolved_package}")

    import mujoco
    from real_robot.simulation import model_xml

    def quat_to_mat(q):
        w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
        return np.array([
            [1 - 2 * (y*y + z*z), 2 * (x*y - w*z), 2 * (x*z + w*y)],
            [2 * (x*y + w*z), 1 - 2 * (x*x + z*z), 2 * (y*z - w*x)],
            [2 * (x*z - w*y), 2 * (y*z + w*x), 1 - 2 * (x*x + y*y)],
        ])

    nominal = json.loads((ASSETS / "nominal_x.json").read_text())
    urdf = ET.parse(ASSETS / "piper_x.urdf").getroot()
    model = mujoco.MjModel.from_xml_string(model_xml())
    checked = []
    for i in range(1, 7):
        name = f"joint{i}"
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"link{i}")
        if jid < 0 or bid < 0:
            raise RuntimeError(f"PiPER X model is missing {name}/link{i}")

        uj = urdf.find(f"joint[@name='{name}']")
        if uj is None:
            raise RuntimeError(f"PiPER X URDF is missing {name}")
        origin = uj.find("origin")
        xyz = np.fromstring(origin.get("xyz"), sep=" ")
        rpy = np.fromstring(origin.get("rpy"), sep=" ")
        expected_quat_xyzw = Rotation.from_euler("xyz", rpy).as_quat()
        expected_quat_wxyz = np.r_[expected_quat_xyzw[3], expected_quat_xyzw[:3]]
        axis = np.fromstring(uj.find("axis").get("xyz"), sep=" ")
        if not np.allclose(model.body_pos[bid], xyz, atol=1e-10):
            raise RuntimeError(f"{name} link origin differs from PiPER X URDF")
        if abs(float(model.body_quat[bid] @ expected_quat_wxyz)) < 1 - 1e-9:
            raise RuntimeError(f"{name} link orientation differs from PiPER X URDF")
        if not np.allclose(model.jnt_axis[jid], axis, atol=1e-10):
            raise RuntimeError(f"{name} joint axis differs from PiPER X URDF")

        if i == 6:
            nominal_link = nominal["link6_plus_fixed_gripper"]
            com = nominal_link["com_m_in_link6_frame"]
            inertia = np.asarray(nominal_link["inertia_com_kg_m2_in_link6_axes"])
            condition = nominal_link["condition"]
        else:
            nominal_link = nominal["links"][f"link{i}"]
            com = nominal_link["com_m_in_link_frame"]
            inertia = np.asarray(nominal_link["inertia_com_kg_m2_in_link_axes"])
            condition = "single PiPER X URDF arm link"
        if not np.isclose(model.body_mass[bid], nominal_link["mass_kg"], rtol=0, atol=1e-12):
            raise RuntimeError(f"link{i} mass differs from nominal_x.json")
        if not np.allclose(model.body_ipos[bid], com, atol=1e-10):
            raise RuntimeError(f"link{i} COM differs from nominal_x.json")
        R_i = quat_to_mat(model.body_iquat[bid])
        inertia_loaded = R_i @ np.diag(model.body_inertia[bid]) @ R_i.T
        if not np.allclose(inertia_loaded, inertia, rtol=1e-8, atol=1e-12):
            raise RuntimeError(f"link{i} inertia differs from nominal_x.json")

        checked.append({
            "joint": name,
            "urdf_transform_and_axis_match": True,
            "nominal_mass_com_inertia_match": True,
            "model_mass_kg": float(model.body_mass[bid]),
            "nominal_parameter_source": "nominal_x.json",
            "link6_gripper_condition": condition if i == 6 else None,
        })

    # Remove preview-only terms exactly as in the archived offline fit.
    model.body_gravcomp[:] = 0
    model.dof_damping[:] = 0
    model.dof_armature[:] = 0
    model.dof_frictionloss[:] = 0
    model.jnt_limited[:] = 0
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    return {
        "status": "passed",
        "model_source": "provenance/code_snapshot/real_robot/simulation.py::model_xml",
        "kinematic_source": "provenance/code_snapshot/real_robot/assets/piper_x.urdf",
        "nominal_inertial_source": "provenance/code_snapshot/real_robot/assets/nominal_x.json",
        "source_hashes_sha256": {rel: sha256(SNAPSHOT / rel) for rel in EXPECTED_HASHES},
        "legacy_project_simulation_model_imported": False,
        "preview_gravity_compensation_damping_armature_friction_limits_contacts": "disabled",
        "joints": checked,
        "measured_position_mapping": {
            "source": "hardware CAN position_deg, joint IDs 1..6",
            "conversion": "degrees to radians, direct joint index; no learned sign or zero offset",
            "absolute_zero_and_sign_independently_verified": False,
            "assumption": "CAN joint convention equals PiPER X URDF joint convention",
        },
    }


def run(run_dir: Path, out_dir: Path, sg_window: int, fit_hz: float,
        torque_coefficients=None, torque_coefficients_source=None):
    audit = model_alignment_audit()
    # The fit module itself must resolve to the same immutable snapshot.
    from real_robot.identify_offline import run_analysis
    try:
        report = run_analysis(
            run_dir, out_dir, window=sg_window, fit_hz=fit_hz,
            torque_coefficients=torque_coefficients,
        )
    except ModuleNotFoundError as exc:
        # The archived script writes all numerical fit artifacts before its
        # optional plotting imports. Keep that valid result if only Matplotlib
        # is absent, and record the missing figures in the run audit.
        if exc.name != "matplotlib" or not (out_dir / "offline_report.json").is_file():
            raise
        report = json.loads((out_dir / "offline_report.json").read_text())
        report["plot_generation"] = "not run: Matplotlib unavailable; numerical fit artifacts were saved"
    report["explicit_model_alignment_audit"] = audit
    report["adapter_runner_sha256"] = sha256(Path(__file__).resolve())
    if torque_coefficients is not None:
        report["torque_coefficient_override"] = {
            "values_Nm_per_A": [float(v) for v in torque_coefficients],
            "source": torque_coefficients_source or "explicit override; source unspecified",
            "calibrated_for_this_unit": False,
        }
    (out_dir / "offline_report.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    )
    (out_dir / "model_alignment_audit.json").write_text(
        json.dumps(audit, ensure_ascii=False, indent=2) + "\n"
    )
    write_parameter_csv(out_dir)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path, help="hardware run directory containing test_manifest.json")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--sg-window", type=int, default=21)
    parser.add_argument("--fit-hz", type=float, default=20.0)
    parser.add_argument(
        "--torque-coefficients", type=float, nargs=6, default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="Optional explicit Nm/A factors; use for documented-reference sensitivity fits.",
    )
    parser.add_argument(
        "--torque-coefficients-source", default=None,
        help="Provenance label recorded in offline_report.json for the explicit factors.",
    )
    args = parser.parse_args()
    if not 0 < args.fit_hz <= 100:
        parser.error("fit-hz must be in (0,100]")
    run(args.run.resolve(), args.out.resolve(), args.sg_window, args.fit_hz,
        args.torque_coefficients, args.torque_coefficients_source)


if __name__ == "__main__":
    main()
