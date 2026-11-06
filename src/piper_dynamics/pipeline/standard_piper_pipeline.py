"""End-to-end identification on the standard PiPER arm.

The workflow is:

1. load one whole real acquisition and build the rigid-body regressor,
2. append the 18 friction columns to obtain the 78-column design matrix,
3. solve a CAD-scaled truncated-SVD least-squares problem,
4. predict the held-out acquisition and write the reports.

Only the training acquisition enters the solve; the validation acquisition is
used exclusively for evaluation.
"""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from ..core.dataset_io import load_dataset
from ..core.parameter_report import (
    parameter_table,
    write_markdown_report,
    write_parameters_csv,
)
from ..core.parameter_schema import PARAMETER_NAMES
from ..core.regression_model import make_friction, metrics, nominal_parameter_scales
from ..core.rigid_body_regressor import RegressorBuilder, ground_truth_theta
from ..core.svd_fit import predict_torque, solve_truncated_svd
from ..paths import STANDARD_MODELS, STANDARD_OUTPUTS
from ..utils.hashing import sha256
from ..utils.plotting import plot_torque_comparison
from .trajectory_split import default_dataset_paths, load_split_rule, output_stem

MODEL_PATH = STANDARD_MODELS / "piper.xml"
LINK_BODIES = tuple((f"link{i}",) for i in range(1, 7))


def load_model() -> mujoco.MjModel:
    """Load the nominal MuJoCo model with all non-rigid terms disabled."""
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    model.body_gravcomp[:] = 0
    model.dof_damping[:] = 0
    model.dof_armature[:] = 0
    model.dof_frictionloss[:] = 0
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    return model


def build_matrix(data: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    """Build the 60-column rigid-body regressor and the nominal parameter vector."""
    model = load_model()
    nominal = ground_truth_theta(model).reshape(-1)
    builder = RegressorBuilder(model, link_bodies=LINK_BODIES)
    rigid = builder.build(data["q"], data["qd"], data["qdd"], progress=True)
    return rigid, nominal


def build_regressor(dataset: Path, out: Path | None = None) -> Path:
    """Write the rigid-body regressor for one dataset to ``outputs/standard_piper``."""
    dataset = dataset.resolve()
    data = load_dataset(dataset)
    rigid, _ = build_matrix(data)
    out = out or STANDARD_OUTPUTS / f"{output_stem(dataset)}_Y.npz"
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, Y=rigid, **data)
    print(f"wrote {out} Y={rigid.shape}")
    return out


def default_fit_path() -> Path:
    """Default output path of the current split's fit."""
    train_dataset, _ = default_dataset_paths()
    return STANDARD_OUTPUTS / f"{output_stem(train_dataset)}_fit.npz"


def fit(
    train_dataset: Path | None = None,
    validation_dataset: Path | None = None,
    out: Path | None = None,
    rank_tolerance: float = 1e-8,
    make_plots: bool = True,
) -> Path:
    """Fit on one whole real acquisition and evaluate on a different acquisition."""
    rule = load_split_rule()
    default_train, default_validation = default_dataset_paths(rule)
    train_dataset = (train_dataset or default_train).resolve()
    validation_dataset = (validation_dataset or default_validation).resolve()
    if train_dataset == validation_dataset:
        raise ValueError("training and validation datasets must be different files")

    train_data = load_dataset(train_dataset)
    validation_data = load_dataset(validation_dataset)

    rigid_train, nominal_rigid = build_matrix(train_data)
    design_train = np.concatenate((rigid_train, make_friction(train_data["qd"])), axis=2)
    nominal = np.concatenate((nominal_rigid, np.zeros(18)))
    scales = nominal_parameter_scales(nominal_rigid)
    theta, singular, keep = solve_truncated_svd(
        design_train, train_data["target_Nm"], nominal, scales, rank_tolerance
    )
    predicted_train = predict_torque(design_train, theta)

    rigid_validation, _ = build_matrix(validation_data)
    design_validation = np.concatenate(
        (rigid_validation, make_friction(validation_data["qd"])), axis=2
    )
    predicted_validation = predict_torque(design_validation, theta)

    out = out or STANDARD_OUTPUTS / f"{output_stem(train_dataset)}_fit.npz"
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out,
        theta=theta,
        nominal_theta=nominal,
        parameter_names=PARAMETER_NAMES,
        parameter_scales=scales,
        singular_values=singular,
        rank=np.count_nonzero(keep),
        predicted_Nm=predicted_train,
        target_Nm=train_data["target_Nm"],
        validation_predicted_Nm=predicted_validation,
        validation_target_Nm=validation_data["target_Nm"],
        cycle=train_data["cycle"],
        validation_cycle=validation_data["cycle"],
    )
    report = {
        "robot": "standard PiPER",
        "method": "CAD-scaled truncated-SVD least squares; unresolved directions set by nominal prior",
        "split": {
            "rule": rule.get("rule"),
            "train": rule["train"],
            "validation": rule["validation"],
            "variant": rule["variant"],
        },
        "train_dataset": str(train_dataset),
        "train_dataset_sha256": sha256(train_dataset),
        "validation_dataset": str(validation_dataset),
        "validation_dataset_sha256": sha256(validation_dataset),
        "model": str(MODEL_PATH),
        "model_sha256": sha256(MODEL_PATH),
        "parameter_file": str(out),
        "rank": int(np.count_nonzero(keep)),
        "rank_tolerance": rank_tolerance,
        "train_samples": int(len(train_data["cycle"])),
        "validation_samples": int(len(validation_data["cycle"])),
        "train": metrics(predicted_train, train_data["target_Nm"]),
        "held_out": metrics(predicted_validation, validation_data["target_Nm"]),
        "torque_calibrated": False,
        "parameter_interpretation": "Torque-prediction candidate, not uniquely identified link inertias",
    }

    rows = parameter_table(theta, nominal)
    csv_path = out.with_name(f"{out.stem}_parameters.csv")
    markdown_path = out.with_suffix(".md")
    write_parameters_csv(csv_path, rows)
    report["parameters"] = rows
    report["parameters_csv"] = str(csv_path)
    report["report_markdown"] = str(markdown_path)

    if make_plots:
        try:
            train_plot = plot_torque_comparison(
                train_data["t"],
                train_data["target_Nm"],
                predicted_train,
                out.with_name(f"{out.stem}_torque_train.png"),
                title=f"Torque comparison: {rule['train']} (training)",
            )
            validation_plot = plot_torque_comparison(
                validation_data["t"],
                validation_data["target_Nm"],
                predicted_validation,
                out.with_name(f"{out.stem}_torque_validation.png"),
                title=f"Torque comparison: {rule['validation']} (held-out)",
            )
            report["torque_plot_train"] = str(train_plot)
            report["torque_plot_validation"] = str(validation_plot)
        except ImportError as error:  # matplotlib is optional at run time
            print(f"warning: skipping torque plots ({error})")

    report_path = out.with_suffix(".json")
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    write_markdown_report(markdown_path, report, rows)
    print(f"wrote {out}, {report_path}, {csv_path} and {markdown_path} (rank {report['rank']}/78)")
    return out
