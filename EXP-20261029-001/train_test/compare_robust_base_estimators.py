#!/usr/bin/env python3
"""Reproducibly compare OLS, per-joint WLS, and Huber IRLS on the URDF base set."""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
EXPERIMENT = HERE.parent
SNAPSHOT = EXPERIMENT / "provenance" / "code_snapshot"
DEFAULT_BASE = HERE / "full_data_official_reference" / "sg41_all_100hz" / "urdf_base_parameters"
DEFAULT_SPLIT = HERE / "sg41" / "selected_train_test_samples.npz"
TORQUE_COEFFICIENTS = np.array([
    4.725, 4.134375, 4.725, 1.232201376, 0.7787701152, 0.7787701152,
])
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(SNAPSHOT))

from identify_urdf_base_parameters import metrics, sha256  # noqa: E402


def fit_scaled_lstsq(X: np.ndarray, y: np.ndarray, row_weights: np.ndarray) -> np.ndarray:
    Xw = X * row_weights[:, None]
    yw = y * row_weights
    norms = np.maximum(np.linalg.norm(Xw, axis=0), 1e-30)
    coef_scaled, *_ = np.linalg.lstsq(Xw / norms[None, :], yw, rcond=1e-10)
    return coef_scaled / norms


def evaluate(name: str, beta: np.ndarray, X_test: np.ndarray,
             target_test: np.ndarray) -> dict:
    prediction = (X_test @ beta).reshape(-1, 6)
    result = metrics(prediction, target_test)
    result["macro_relative_rmse"] = float(np.mean(result["rmse_over_target_rms"]))
    result["method"] = name
    return result


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", type=Path, default=EXPERIMENT / "data")
    ap.add_argument("--base-out", type=Path, default=DEFAULT_BASE)
    ap.add_argument("--split", type=Path, default=DEFAULT_SPLIT)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--sg-window", type=int, default=41)
    ap.add_argument("--huber-delta", type=float, default=1.345)
    ap.add_argument("--huber-max-iterations", type=int, default=30)
    ap.add_argument("--convergence-tolerance", type=float, default=1e-8)
    args = ap.parse_args()
    if args.huber_delta <= 0 or args.huber_max_iterations < 1 or args.convergence_tolerance <= 0:
        ap.error("Huber delta, iteration count, and convergence tolerance must be positive")

    from real_robot.identify_offline import (  # noqa: E402
        friction_matrix, inertial_model, load_measurements, original_modules,
    )

    rg, _, snapshot_root = original_modules()
    if Path(snapshot_root).resolve() != SNAPSHOT.resolve():
        raise RuntimeError("Regressor did not resolve to the archived PiPER X snapshot")
    model = inertial_model()
    builder = rg.RegressorBuilder(model, link_bodies=tuple((f"link{i}",) for i in range(1, 7)))

    base_npz_path = args.base_out.resolve() / "base_parameter_fit.npz"
    with np.load(base_npz_path) as base_npz:
        V = base_npz["V"]
        parameter_scale = base_npz["parameter_scale"]
    if V.shape != (78, 54) or parameter_scale.shape != (78,):
        raise RuntimeError(f"Unexpected minimum-base dimensions: V={V.shape}, S={parameter_scale.shape}")

    signals, _ = load_measurements(args.run.resolve(), dt=0.01, window=args.sg_window)
    valid_source_idx = np.flatnonzero(np.asarray(signals["valid"], dtype=bool))
    q = signals["q"][valid_source_idx]
    qd = signals["qd"][valid_source_idx]
    qdd = signals["qdd"][valid_source_idx]
    target = signals["current_A"][valid_source_idx] * TORQUE_COEFFICIENTS

    print(f"Building the PiPER X regressor for {len(valid_source_idx)} samples", flush=True)
    Y = builder.build(q, qd, qdd, progress=True)
    A = np.concatenate([Y, friction_matrix(qd)], axis=2).reshape(-1, 78)
    base_matrix = ((A * parameter_scale[None, :]) @ V).reshape(len(q), 6, 54)

    with np.load(args.split.resolve()) as split:
        selected_signal_idx = split["aligned_signal_index"].astype(int)
        is_train = split["split_is_train"].astype(bool)
    source_to_row = {int(source_idx): row for row, source_idx in enumerate(valid_source_idx)}
    train_rows = np.asarray([source_to_row[i] for i in selected_signal_idx[is_train]
                             if i in source_to_row], dtype=int)
    test_rows = np.asarray([source_to_row[i] for i in selected_signal_idx[~is_train]
                            if i in source_to_row], dtype=int)
    if np.intersect1d(train_rows, test_rows).size or len(train_rows) == 0 or len(test_rows) == 0:
        raise RuntimeError("The saved complete-cycle train/test split did not map to disjoint samples")

    X_train = base_matrix[train_rows].reshape(-1, 54)
    y_train = target[train_rows].reshape(-1)
    X_test = base_matrix[test_rows].reshape(-1, 54)
    target_test = target[test_rows]
    n_train = len(train_rows)

    beta_ols = fit_scaled_lstsq(X_train, y_train, np.ones(len(y_train)))
    training_target_rms = np.sqrt(np.mean(target[train_rows] ** 2, axis=0))
    joint_weights = np.tile(1.0 / np.maximum(training_target_rms, 1e-12), n_train)
    beta_wls = fit_scaled_lstsq(X_train, y_train, joint_weights)

    beta_huber = beta_wls.copy()
    converged = False
    final_sigma = np.zeros(6)
    for iteration in range(1, args.huber_max_iterations + 1):
        residual_by_joint = (X_train @ beta_huber - y_train).reshape(n_train, 6)
        normalized_residual = residual_by_joint / np.maximum(training_target_rms[None, :], 1e-12)
        median = np.median(normalized_residual, axis=0)
        final_sigma = 1.4826 * np.median(np.abs(normalized_residual - median[None, :]), axis=0)
        final_sigma = np.maximum(final_sigma, 1e-4)
        absolute = np.abs(normalized_residual)
        robust_sqrt_weight = np.sqrt(np.minimum(
            1.0, args.huber_delta * final_sigma[None, :] / np.maximum(absolute, 1e-12)
        ))
        row_weights = (robust_sqrt_weight / np.maximum(training_target_rms[None, :], 1e-12)).reshape(-1)
        beta_next = fit_scaled_lstsq(X_train, y_train, row_weights)
        relative_change = float(np.linalg.norm(beta_next - beta_huber) /
                                max(np.linalg.norm(beta_huber), 1e-12))
        beta_huber = beta_next
        if relative_change < args.convergence_tolerance:
            converged = True
            break

    results = [
        evaluate("OLS", beta_ols, X_test, target_test),
        evaluate("target_RMS_WLS", beta_wls, X_test, target_test),
        evaluate("target_RMS_Huber_IRLS", beta_huber, X_test, target_test),
    ]
    report = {
        "title": "Robust loss comparison on the PiPER X URDF minimum base set",
        "interpretation": "Exploratory estimator comparison only; it does not change structural identifiability or produce unique link masses.",
        "source": {
            "run": str(args.run.resolve()),
            "test_raw_can_sha256": sha256(args.run.resolve() / "test_raw_can.jsonl"),
            "test_signals_sha256": sha256(args.run.resolve() / "test_signals.npz"),
            "minimum_base_npz_sha256": sha256(base_npz_path),
            "minimum_base_report": str((args.base_out.resolve() / "base_parameter_identification_report.json")),
            "split_file": str(args.split.resolve()),
            "torque_coefficients_Nm_per_A": TORQUE_COEFFICIENTS.tolist(),
            "sg_window": args.sg_window,
            "train_samples": int(len(train_rows)),
            "held_out_samples": int(len(test_rows)),
            "base_dimension": 54,
        },
        "estimators": results,
        "huber_settings": {
            "delta": args.huber_delta,
            "max_iterations": args.huber_max_iterations,
            "iterations_used": iteration,
            "converged": converged,
            "relative_coefficient_change_last_iteration": relative_change,
            "convergence_tolerance": args.convergence_tolerance,
            "mad_scale_of_target_rms_normalized_residual_per_joint": final_sigma.tolist(),
        },
        "selection": "No robust loss is selected: Huber must improve held-out metrics before replacing the ordinary fit.",
        "runtime": {"python": sys.version.split()[0], "numpy": np.__version__, "mujoco": mujoco.__version__},
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    out = args.out.resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({
        "output": str(out),
        "train_samples": len(train_rows),
        "held_out_samples": len(test_rows),
        "metrics": [{
            "method": item["method"],
            "overall_relative_error_norm_ratio": item["relative_error_norm_ratio"],
            "macro_relative_rmse": item["macro_relative_rmse"],
        } for item in results],
        "huber_converged": converged,
        "huber_iterations": iteration,
    }, indent=2), flush=True)


if __name__ == "__main__":
    main()
