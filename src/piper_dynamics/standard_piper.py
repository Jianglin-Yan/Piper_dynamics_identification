"""Build and fit the standard PiPER offline dynamics regressor."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import mujoco
import numpy as np

from .paths import STANDARD_INPUTS, STANDARD_MODELS, STANDARD_OUTPUTS
from .reference.regressor import PARAM_NAMES, RegressorBuilder, ground_truth_theta
from .regression import make_friction, metrics, nominal_parameter_scales, sha256


MODEL_PATH = STANDARD_MODELS / "piper.xml"
DEFAULT_DATASET = STANDARD_INPUTS / "old_fast_0" / "sg41.npz"
LINK_BODIES = tuple((f"link{i}",) for i in range(1, 7))
FRICTION_NAMES = ("viscous_friction", "smooth_coulomb_friction", "offset")
PARAMETER_NAMES = tuple(
    f"link{i}.{name}" for i in range(1, 7) for name in PARAM_NAMES
) + tuple(
    f"joint{i}.{name}" for i in range(1, 7) for name in FRICTION_NAMES
)


def load_model() -> mujoco.MjModel:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    model.body_gravcomp[:] = 0
    model.dof_damping[:] = 0
    model.dof_armature[:] = 0
    model.dof_frictionloss[:] = 0
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    return model


def load_dataset(path: Path) -> dict[str, np.ndarray]:
    with np.load(path) as archive:
        required = {"q", "qd", "qdd", "t", "cycle", "target_Nm"}
        missing = required - set(archive.files)
        if missing:
            raise ValueError(f"{path}: missing keys {sorted(missing)}")
        data = {name: archive[name] for name in required}
    n = len(data["q"])
    if any(data[name].shape != (n, 6) for name in ("q", "qd", "qdd", "target_Nm")):
        raise ValueError(f"{path}: q, qd, qdd and target_Nm must have shape (N, 6)")
    if any(data[name].shape != (n,) for name in ("t", "cycle")):
        raise ValueError(f"{path}: t and cycle must have shape (N,)")
    return data


def build_matrix(data: dict[str, np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    model = load_model()
    nominal = ground_truth_theta(model).reshape(-1)
    builder = RegressorBuilder(model, link_bodies=LINK_BODIES)
    rigid = builder.build(data["q"], data["qd"], data["qdd"], progress=True)
    return rigid, nominal


def output_stem(dataset: Path) -> str:
    return f"{dataset.parent.name}_{dataset.stem}"


def build_regressor(dataset: Path, out: Path | None = None) -> Path:
    dataset = dataset.resolve()
    data = load_dataset(dataset)
    rigid, _ = build_matrix(data)
    out = out or STANDARD_OUTPUTS / f"{output_stem(dataset)}_Y.npz"
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, Y=rigid, **data)
    print(f"wrote {out} Y={rigid.shape}")
    return out


def fit(dataset: Path, out: Path | None = None, rank_tolerance: float = 1e-8) -> Path:
    if not np.isfinite(rank_tolerance) or not 0 < rank_tolerance < 1:
        raise ValueError("rank_tolerance must be finite and between 0 and 1")
    dataset = dataset.resolve()
    data = load_dataset(dataset)
    split = json.loads((STANDARD_INPUTS / "splits.json").read_text())["datasets"]
    if dataset.parent.name not in split:
        raise ValueError(f"no cycle split for {dataset.parent.name}")
    selected = split[dataset.parent.name]
    train = np.isin(data["cycle"], selected["train_cycles"])
    test = np.isin(data["cycle"], selected["test_cycles"])
    if not train.any() or not test.any() or np.any(train & test) or not np.all(train | test):
        raise ValueError("training and held-out cycles must partition all samples")

    rigid, nominal_rigid = build_matrix(data)
    full = np.concatenate((rigid, make_friction(data["qd"])), axis=2)
    nominal = np.concatenate((nominal_rigid, np.zeros(18)))
    scales = nominal_parameter_scales(nominal_rigid)
    A_train = full[train].reshape(-1, 78)
    b_train = data["target_Nm"][train].reshape(-1)
    U, singular, Vt = np.linalg.svd(A_train * scales, full_matrices=False)
    keep = singular > rank_tolerance * singular[0]
    if not np.any(keep):
        raise ValueError("training regressor has no identifiable directions")
    delta = (U[:, keep].T @ (b_train - A_train @ nominal)) / singular[keep]
    theta = nominal + scales * (Vt[keep].T @ delta)
    predicted = np.einsum("nij,j->ni", full, theta)

    out = out or STANDARD_OUTPUTS / f"{output_stem(dataset)}_fit.npz"
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        out, theta=theta, nominal_theta=nominal, parameter_names=PARAMETER_NAMES,
        parameter_scales=scales, singular_values=singular, rank=np.count_nonzero(keep),
        predicted_Nm=predicted, target_Nm=data["target_Nm"], train_mask=train,
        test_mask=test, cycle=data["cycle"],
    )
    report = {
        "robot": "standard PiPER",
        "method": "CAD-scaled truncated-SVD least squares; unresolved directions set by nominal prior",
        "dataset": str(dataset),
        "dataset_sha256": sha256(dataset),
        "model": str(MODEL_PATH),
        "model_sha256": sha256(MODEL_PATH),
        "parameter_file": str(out),
        "rank": int(np.count_nonzero(keep)),
        "rank_tolerance": rank_tolerance,
        "train_samples": int(train.sum()),
        "test_samples": int(test.sum()),
        "train": metrics(predicted[train], data["target_Nm"][train]),
        "held_out": metrics(predicted[test], data["target_Nm"][test]),
        "torque_calibrated": False,
        "parameter_interpretation": "Torque-prediction candidate, not uniquely identified link inertias",
    }
    report_path = out.with_suffix(".json")
    report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(f"wrote {out} and {report_path} (rank {report['rank']}/78)")
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    subcommands = parser.add_subparsers(dest="command", required=True)
    for name in ("regressor", "fit"):
        command = subcommands.add_parser(name)
        command.add_argument("dataset", nargs="?", type=Path, default=DEFAULT_DATASET)
        command.add_argument("--out", type=Path)
        if name == "fit":
            command.add_argument("--rank-tolerance", type=float, default=1e-8)
    args = parser.parse_args()
    if args.command == "regressor":
        build_regressor(args.dataset, args.out)
    else:
        fit(args.dataset, args.out, args.rank_tolerance)


if __name__ == "__main__":
    main()
