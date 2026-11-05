"""Joint rigid-body + friction refit using the six low-speed acquisitions.

The dynamic train/validation split is unchanged.  Low-speed friction samples are
added to the training system only; the held-out dynamic acquisition and the
second repeat of every friction segment are used for validation.  A weighted
least-squares fit keeps the two sources comparable: each source contributes
roughly the same total weight.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from piper_dynamics.core.dataset_io import load_dataset
from piper_dynamics.core.regression_model import metrics
from piper_dynamics.paths import IDENTIFICATION_ARCHIVE, OUTPUTS
from piper_dynamics.pipeline.friction_combined import load_arrays, load_reference
from piper_dynamics.pipeline.friction_paths import parse_paths
from piper_dynamics.pipeline.trajectory_split import default_dataset_paths

WIDTH = 0.005


def friction_features(qd: np.ndarray, width: float = WIDTH) -> np.ndarray:
    """Eighteen friction columns with an explicit smoothing width."""
    n = len(qd)
    out = np.zeros((n, 6, 18))
    for joint in range(6):
        out[:, joint, 3 * joint : 3 * joint + 3] = np.column_stack(
            [qd[:, joint], np.tanh(qd[:, joint] / width), np.ones(n)]
        )
    return out


def load_dynamic(path: Path):
    data = load_dataset(path)
    from piper_dynamics.pipeline.standard_piper_pipeline import build_matrix

    rigid, _ = build_matrix(data)
    return dict(rigid=rigid, qd=data["qd"], target=data["target_Nm"])


def load_friction(config):
    """Rigid regressor and torque for the six runs, same selection as the analysis."""
    builder, _, _ = load_reference(config)
    records = []
    for joint in (1, 6, 4, 5, 2, 3):
        data = load_arrays(joint, config)
        mov = data["good"] & (np.abs(data["qd"][:, joint - 1]) > 0.002)
        sel = np.flatnonzero(mov)[::5]
        q, v, a = data["q"][sel], data["qd"][sel], data["qdd"][sel]
        y = builder.build(q, v, a)
        records.append(dict(rigid=y, qd=v, target=data["tau"][sel],
                            repeat=data["repeat"][sel], joint=joint))
    return records


def design(rigid, qd):
    return np.concatenate((rigid, friction_features(qd)), axis=2)


def weighted_lstsq(blocks, target_blocks, weights):
    rows = []
    vals = []
    for design, target, weight in zip(blocks, target_blocks, weights):
        a = design.reshape(-1, design.shape[-1])
        y = target.reshape(-1)
        w = np.sqrt(weight)
        rows.append(a * w)
        vals.append(y * w)
    return np.linalg.lstsq(np.concatenate(rows), np.concatenate(vals), rcond=None)[0]


def rmse_per_joint(pred, target):
    err = pred - target
    return np.sqrt(np.mean(err**2, axis=0))


def main(argv=None):
    config = parse_paths(__doc__, argv)
    archive = Path(IDENTIFICATION_ARCHIVE)
    active = json.loads((archive / "active_revision.json").read_text())
    baseline = np.load(archive / active["revision_directory"] / "training/main_balanced_v2.npz")["sg21_LS"]
    train_path, validation_path = default_dataset_paths()
    train = load_dynamic(train_path)
    validation = load_dynamic(validation_path)
    friction = load_friction(config)

    a_train = design(train["rigid"], train["qd"])
    a_validation = design(validation["rigid"], validation["qd"])
    train_f = [design(r["rigid"], r["qd"]) for r in friction]
    n_dynamic = len(train["qd"])
    n_friction = sum(len(r["qd"]) for r in friction)
    friction_weight = n_dynamic / n_friction
    print(f"dynamic train {n_dynamic}, friction {n_friction}, weight {friction_weight:.3f}", flush=True)

    theta = weighted_lstsq(
        [a_train, *train_f],
        [train["target"], *[r["target"] for r in friction]],
        [1.0, *[friction_weight] * len(friction)],
    )
    baseline = np.asarray(baseline, float)
    report = {"width": WIDTH, "n_dynamic": n_dynamic, "n_friction": n_friction,
              "friction_weight": friction_weight, "methods": {}}
    for name, vector in (("baseline_revision_001", baseline), ("combined", theta)):
        pred_train = np.einsum("nij,j->ni", a_train, vector)
        pred_validation = np.einsum("nij,j->ni", a_validation, vector)
        friction_rmse = {}
        for r, a in zip(friction, train_f):
            pred = np.einsum("nij,j->ni", a, vector)
            per_repeat = {}
            for repeat in (1, 2):
                m = r["repeat"] == repeat
                per_repeat[str(repeat)] = rmse_per_joint(pred[m], r["target"][m]).tolist()
            friction_rmse[r["joint"]] = per_repeat
        report["methods"][name] = dict(
            train=metrics(pred_train, train["target"]),
            validation=metrics(pred_validation, validation["target"]),
            friction=dict(rmse_per_joint=rmse_per_joint(
                np.vstack([np.einsum("nij,j->ni", a, vector) for a in train_f]),
                np.vstack([r["target"] for r in friction])).tolist()),
            friction_per_joint=friction_rmse,
            friction_parameters=vector[60:].reshape(6, 3).tolist(),
        )
        print(name, "train rel", round(report["methods"][name]["train"]["relative_error_norm_ratio"], 4),
              "val rel", round(report["methods"][name]["validation"]["relative_error_norm_ratio"], 4),
              "friction rmse", np.round(report["methods"][name]["friction"]["rmse_per_joint"], 4), flush=True)

    out = config.output
    np.savez(out / "theta_combined.npz", theta=theta, width=WIDTH,
             parameters=["link inertial (60)", "friction fv/fc/fo (18)"])
    (out / "theta_combined.json").write_text(json.dumps({
        "width": WIDTH, "theta": theta.tolist(),
        "inertial_theta_60": theta[:60].tolist(),
        "friction_fv_fc_fo": theta[60:].reshape(6, 3).tolist()}, indent=2))
    (out / "combined_report.json").write_text(json.dumps(report, indent=2))
    print("wrote", out / "theta_combined.npz")


if __name__ == "__main__":
    main()
