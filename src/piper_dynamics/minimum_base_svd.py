#!/usr/bin/env python3
"""Generate a URDF-derived structural base regressor, then identify it.

The URDF is loaded into the archived PiPER X model. Generic synthetic states
are used to derive and independently verify a CAD-scaled SVD minimum parameter
basis. Regressor column-RMS scaling is avoided because finite-difference noise
in structurally zero columns can otherwise be amplified to unit size. Robot
measurements are used only for fitting and validation. This script is offline
and never connects to a robot.
"""
from __future__ import annotations

import argparse
import csv
import json
import platform
import sys
from pathlib import Path

import mujoco
import numpy as np
import scipy

from .model_alignment import model_alignment_audit
from .paths import INPUT_RUN, MODELS, OUTPUTS, REFERENCE, SPLIT
from .regression import make_friction, metrics, nominal_parameter_scales, sha256
from .split import load_validated_split

SNAPSHOT = REFERENCE
sys.path.insert(0, str(SNAPSHOT))

TORQUE_COEFFICIENTS = np.array([
    4.725, 4.134375, 4.725, 1.232201376, 0.7787701152, 0.7787701152,
])
INERTIAL_NAMES = ("m", "hx", "hy", "hz", "Ixx", "Iyy", "Izz", "Ixy", "Ixz", "Iyz")
FRICTION_NAMES = ("viscous_friction", "smooth_coulomb_friction", "offset")


def svd_rank(A_scaled, rel_tol=1e-8):
    """Rank-revealing SVD in physically scaled parameter coordinates."""
    _, singular, vt = np.linalg.svd(A_scaled, full_matrices=False)
    rank = int(np.sum(singular > rel_tol * singular[0])) if singular.size and singular[0] else 0
    Vt = vt.copy()
    # Canonicalize the otherwise arbitrary SVD signs for reproducible files.
    for k in range(Vt.shape[0]):
        pivot = int(np.argmax(np.abs(Vt[k])))
        if Vt[k, pivot] < 0:
            Vt[k] *= -1
    return rank, singular, Vt


def scaled_rank(A, rel_tol=1e-8):
    rank, singular, _ = svd_rank(A, rel_tol)
    return rank, singular


def scaled_lstsq(A, b, rel_tol=1e-10):
    norms = np.maximum(np.linalg.norm(A, axis=0), 1e-30)
    coef_scaled, _, rank, singular = np.linalg.lstsq(A / norms[None, :], b, rcond=rel_tol)
    return coef_scaled / norms, int(rank), singular


def param_names(rg):
    inertial = list(rg.PARAM_NAMES)
    # Link names are the owner body names; the gripper remains lumped into link6.
    names = [f"link{i}.{p}" for i in range(1, 7) for p in inertial]
    names += [f"joint{j}.{p}" for j in range(1, 7) for p in FRICTION_NAMES]
    return names


def generic_states(model, builder, seed, count):
    rng = np.random.default_rng(seed)
    lower = np.array([model.jnt_range[j, 0] for j in builder.joint_ids])
    upper = np.array([model.jnt_range[j, 1] for j in builder.joint_ids])
    q = rng.uniform(lower + 0.05*(upper-lower), upper - 0.05*(upper-lower), (count, 6))
    qd = rng.uniform(-1.5, 1.5, (count, 6))
    qdd = rng.uniform(-4.0, 4.0, (count, 6))
    return q, qd, qdd


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", type=Path, default=INPUT_RUN)
    ap.add_argument("--split", type=Path, default=SPLIT)
    ap.add_argument("--out", type=Path, default=OUTPUTS / "minimum_base")
    ap.add_argument("--sg-window", type=int, default=41)
    ap.add_argument("--generic-samples-per-seed", type=int, default=300)
    ap.add_argument("--rank-tolerance", type=float, default=1e-8)
    ap.add_argument("--fit-rcond", type=float, default=1e-10)
    args = ap.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)

    from real_robot.identify_offline import friction_matrix, inertial_model, load_measurements
    from real_robot.identify_offline import original_modules
    rg, _, snapshot_root = original_modules()
    if Path(snapshot_root).resolve() != SNAPSHOT.resolve():
        raise RuntimeError("Dynamics regressor did not resolve to the archived PiPER X snapshot")
    model_audit = model_alignment_audit()

    model = inertial_model()
    builder = rg.RegressorBuilder(model, link_bodies=tuple((f"link{i}",) for i in range(1, 7)))
    theta_nominal = rg.ground_truth_theta(model).reshape(-1)
    parameter_scale = nominal_parameter_scales(theta_nominal)
    generic_by_seed = {}
    for seed in (100, 101, 102):
        qg, qdg, qddg = generic_states(model, builder, seed, args.generic_samples_per_seed)
        Yi = builder.build(qg, qdg, qddg, progress=False)
        Ai = np.concatenate([Yi, make_friction(qdg)], axis=2).reshape(-1, 78)
        generic_by_seed[seed] = Ai
    A_struct = np.vstack([generic_by_seed[s] for s in (100, 101, 102)])
    A_struct_scaled = A_struct * parameter_scale[None, :]
    rigid_rank, rigid_singular, _ = svd_rank(A_struct_scaled[:, :60], args.rank_tolerance)
    augmented_rank, generic_singular, generic_vt = svd_rank(A_struct_scaled, args.rank_tolerance)
    if rigid_rank != 36 or augmented_rank != 54:
        raise RuntimeError(f"Unexpected URDF-derived model rank: rigid={rigid_rank}, augmented={augmented_rank}")
    V = generic_vt[:augmented_rank].T
    V0 = generic_vt[augmented_rank:].T
    if V.shape != (78, 54) or V0.shape != (78, 24):
        raise RuntimeError(f"Unexpected minimum-set dimensions: V={V.shape}, nullspace={V0.shape}")
    natural_column_rms = np.linalg.norm(A_struct_scaled, axis=0) / np.sqrt(A_struct_scaled.shape[0])
    near_zero_columns = [param_names(rg)[i] for i, value in enumerate(natural_column_rms)
                         if value < 1e-9 * natural_column_rms.max()]
    mass_observability = []
    for i in range(6):
        row = 10*i  # mass coordinate in each [m, h, I_origin] block
        mass_observability.append({
            "link": f"link{i+1}",
            "observable_subspace_projection": float(np.linalg.norm(V[row, :])),
            "nullspace_projection": float(np.linalg.norm(V0[row, :])),
            "interpretation": "projection of a unit CAD-scaled mass coordinate onto the observable/null subspaces; not a statistical confidence",
        })
    retained_to_discarded_singular_gap = float(
        generic_singular[augmented_rank-1] / max(generic_singular[augmented_rank], 1e-300)
    )

    # Verify the selected generic basis on a completely separate random state set.
    qv, qdv, qddv = generic_states(model, builder, 707, 250)
    Yv = builder.build(qv, qdv, qddv, progress=False)
    Av = np.concatenate([Yv, make_friction(qdv)], axis=2).reshape(-1, 78)
    Av_scaled = Av * parameter_scale[None, :]
    verify_rank, verify_singular, _ = svd_rank(Av_scaled @ V, args.rank_tolerance)
    generic_identity_error = float(np.linalg.norm((Av_scaled @ V) @ V.T - Av_scaled) /
                                   max(np.linalg.norm(Av_scaled), 1e-30))
    if generic_identity_error > 1e-8:
        raise RuntimeError(f"URDF minimum-set basis failed independent reconstruction: {generic_identity_error:.3e}")

    # Verify the inertial regressor against inverse dynamics on this archived
    # PiPER X model, independently of the generic SVD subspace identity.
    qi, qdi, qddi = generic_states(model, builder, 883, 40)
    Yi = builder.build(qi, qdi, qddi, progress=False)
    tau_regressor = np.einsum("nij,j->ni", Yi, theta_nominal)
    reference_model = inertial_model()
    reference_data = mujoco.MjData(reference_model)
    reference_builder = rg.RegressorBuilder(
        reference_model, link_bodies=tuple((f"link{i}",) for i in range(1, 7))
    )
    tau_inverse = np.empty_like(tau_regressor)
    for sample in range(len(qi)):
        reference_data.qpos[reference_builder.qadr] = qi[sample]
        reference_data.qvel[:] = 0.0
        reference_data.qvel[reference_builder.dadr] = qdi[sample]
        reference_data.qacc[:] = 0.0
        reference_data.qacc[reference_builder.dadr] = qddi[sample]
        reference_data.qfrc_applied[:] = 0.0
        mujoco.mj_inverse(reference_model, reference_data)
        tau_inverse[sample] = reference_data.qfrc_inverse[reference_builder.dadr]
    regressor_identity_max_abs = float(np.max(np.abs(tau_regressor - tau_inverse)))
    regressor_identity_relative = float(
        regressor_identity_max_abs / max(float(np.max(np.abs(tau_inverse))), 1e-12)
    )
    if regressor_identity_relative > 1e-8:
        raise RuntimeError(
            "URDF inverse-dynamics regressor failed PiPER X nominal identity: "
            f"{regressor_identity_relative:.3e}"
        )

    # Align hardware measurements and use the established full-cycle split.
    signals, manifest = load_measurements(args.run.resolve(), dt=0.01, window=args.sg_window)
    valid_source_idx = np.flatnonzero(np.asarray(signals["valid"], dtype=bool))
    q, qd, qdd = signals["q"][valid_source_idx], signals["qd"][valid_source_idx], signals["qdd"][valid_source_idx]
    current = signals["current_A"][valid_source_idx]
    target = current * TORQUE_COEFFICIENTS
    print(f"Building measured-data regressor for {len(valid_source_idx)} samples", flush=True)
    Y = builder.build(q, qd, qdd, progress=True)
    Y = np.concatenate([Y, friction_matrix(qd)], axis=2)
    A = Y.reshape(-1, 78)
    target_flat = target.reshape(-1)

    train_rows, test_rows = load_validated_split(args.split, signals, valid_source_idx)
    if np.intersect1d(train_rows, test_rows).size or len(train_rows) < 100 or len(test_rows) < 100:
        raise RuntimeError("Train/test cycle indices did not map to disjoint measured samples")

    A_full_scaled = A * parameter_scale[None, :]
    A_base = A_full_scaled @ V
    A_base_train = A_base.reshape(len(q), 6, augmented_rank)[train_rows].reshape(-1, augmented_rank)
    b_train = target[train_rows].reshape(-1)
    beta_train, rank_train, singular_train = scaled_lstsq(A_base_train, b_train, args.fit_rcond)
    A_base_test = A_base.reshape(len(q), 6, augmented_rank)[test_rows].reshape(-1, augmented_rank)
    b_test = target[test_rows].reshape(-1)
    pred_test = (A_base_test @ beta_train).reshape(-1, 6)
    validation_metrics = metrics(pred_test, target[test_rows])

    A_base_all = A_base.reshape(-1, augmented_rank)
    beta_all, rank_all, singular_all = scaled_lstsq(A_base_all, target_flat, args.fit_rcond)
    pred_all = (A_base_all @ beta_all).reshape(-1, 6)
    fit_all_metrics = metrics(pred_all, target)
    pred_train = (A_base_train @ beta_train).reshape(-1, 6)
    train_metrics = metrics(pred_train, target[train_rows])

    # Check that the generic-selected base set remains independent on the
    # measured trajectory and quantify the actual trajectory's excitation.
    base_rank_train, sv_train = scaled_rank(A_base_train, args.rank_tolerance)
    base_rank_test, sv_test = scaled_rank(A_base_test, args.rank_tolerance)
    base_rank_all, sv_all = scaled_rank(A_base_all, args.rank_tolerance)
    if base_rank_train != augmented_rank:
        print(f"WARNING: training trajectory excites only {base_rank_train}/{augmented_rank} generic base parameters", flush=True)

    names = param_names(rg)
    with (out / "urdf_structural_base_parameters.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["base_parameter", "generic_singular_value", "train_coefficient_Nm",
                    "all_data_coefficient_Nm", "coordinate_definition"])
        for order in range(augmented_rank):
            w.writerow([f"beta_{order+1:02d}", generic_singular[order], beta_train[order], beta_all[order],
                        "SVD coordinate of CAD-scaled URDF-derived regressor; not a link parameter"])

    with (out / "urdf_structural_base_directions.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["base_parameter", "full_regressor_parameter", "right_singular_direction",
                    "parameter_scale_S", "minimum_set_theta_map_S_times_V"])
        for k in range(augmented_rank):
            for j, name in enumerate(names):
                w.writerow([f"beta_{k+1:02d}", name, V[j, k], parameter_scale[j], parameter_scale[j] * V[j, k]])

    with (out / "base_parameter_validation_metrics.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["split", "joint", "rmse_Nm", "target_rms_Nm", "rmse_over_target_rms", "r2", "relative_error_norm_ratio"])
        for split_name, m in (("training_cycles", train_metrics), ("held_out_cycles", validation_metrics),
                              ("all_data_in_sample", fit_all_metrics)):
            for j in range(6):
                w.writerow([split_name, j+1, m["rmse_Nm"][j], m["target_rms_Nm"][j],
                            m["rmse_over_target_rms"][j], m["r2_per_joint"][j],
                            m["relative_error_norm_ratio"]])

    seed_ranks = []
    for seed, As in generic_by_seed.items():
        As_scaled = As * parameter_scale[None, :]
        rr, _, _ = svd_rank(As_scaled[:, :60], args.rank_tolerance)
        ra, _, _ = svd_rank(As_scaled, args.rank_tolerance)
        seed_ranks.append({"seed": seed, "samples": args.generic_samples_per_seed,
                           "rigid_inertial_rank": rr, "augmented_rank": ra})
    report = {
        "title": "URDF-derived minimum base parameter identification for PiPER X",
        "status": "base_parameter_set_generated_and_fitted",
        "minimum_base_parameter_fit_successful": bool(
            augmented_rank == 54 and verify_rank == 54 and rank_train == 54 and base_rank_train == 54),
        "physical_link_parameter_identification_successful": False,
        "suitable_for_control": False,
        "urdf_model_source": "models/piper_x.urdf + models/nominal_x.json through archived real_robot.simulation.model_xml",
        "model_alignment_audit": model_audit,
        "structural_rank_method": "SVD on inverse-dynamics regressors multiplied by CAD-derived characteristic parameter scales (m0, m0*ell, m0*ell^2; friction scales [1,0.5,1]) from three independently seeded generic state batches sampled within 5%-95% of URDF joint ranges; qd in [-1.5,1.5] rad/s and qdd in [-4,4] rad/s^2. Column-RMS normalization is deliberately avoided because it amplified finite-difference roundoff in structural-zero columns. The minimum set is the rank-r right-singular subspace, not a subset of original columns.",
        "rank_relative_tolerance": args.rank_tolerance,
        "near_zero_generic_columns_below_scaled_rms_1e-9_of_max": near_zero_columns,
        "retained_to_discarded_singular_value_gap": retained_to_discarded_singular_gap,
        "mass_coordinate_observability_audit": mass_observability,
        "previous_column_rms_rank_artifact": {
            "previous_reported_rank": 65,
            "corrected_rank": 54,
            "cause": "per-column RMS scaling amplified finite-difference roundoff in the 11 listed structural-zero regressor columns",
        },
        "per_seed_generic_ranks": seed_ranks,
        "structural_ranks": {"rigid_body_inertial": rigid_rank, "augmented_inertial_friction_bias": augmented_rank,
                             "full_rigid_parameters": 60, "full_augmented_parameters": 78},
        "independent_generic_basis_validation": {
            "seed": 707, "samples": 250, "selected_basis_rank": verify_rank,
            "full_regressor_reconstruction_relative_error": generic_identity_error,
        },
        "nominal_inverse_dynamics_regressor_identity": {
            "seed": 883, "samples": len(qi),
            "max_absolute_torque_error_Nm": regressor_identity_max_abs,
            "relative_to_max_inverse_torque": regressor_identity_relative,
        },
        "minimum_parameter_set": {
            "coordinate_system": "theta = S @ (V @ beta + V0 @ gamma); beta = V.T @ S^-1 @ theta; Y_base = Y_full @ S @ V",
            "base_dimension": augmented_rank,
            "nullspace_dimension": int(V0.shape[1]),
            "basis_matrix_file": "base_parameter_fit.npz: V, V0, parameter_scale",
            "directions_csv": "urdf_structural_base_directions.csv",
            "inertial_only_rank": rigid_rank,
            "friction_and_bias_columns_added": 18,
            "coefficients_are_link_masses_or_inertias": False,
        },
        "data": {"record_seconds": 120, "valid_samples": len(valid_source_idx), "grid_hz": 100,
                 "sg_window": args.sg_window,
                 "first_valid_s": float(signals["t"][valid_source_idx].min()),
                 "last_valid_s": float(signals["t"][valid_source_idx].max()),
                 "target_torque_coefficients_Nm_per_A": TORQUE_COEFFICIENTS.tolist(),
                 "torque_scale_unit_calibrated": False},
        "train_test_split": {"basis_source": "pre-existing SG41 split by complete 10-second trajectory cycles",
                             "train_cycles_zero_based": [2,4,6,8,10],
                             "test_cycles_zero_based": [1,3,5,7,9],
                             "train_samples": len(train_rows), "test_samples": len(test_rows)},
        "trajectory_base_ranks": {"train": base_rank_train, "test": base_rank_test, "all": base_rank_all,
                                  "n_structural_base_parameters": augmented_rank},
        "training_metrics": train_metrics,
        "held_out_cycle_validation_metrics": validation_metrics,
        "all_data_refit_in_sample_metrics": fit_all_metrics,
        "condition_numbers": {
            "generic_full_basis": float(generic_singular[0] / generic_singular[augmented_rank-1]),
            "training_base_matrix": float(sv_train[0] / max(sv_train[rank_train-1], 1e-30)),
            "test_base_matrix": float(sv_test[0] / max(sv_test[base_rank_test-1], 1e-30)),
            "all_data_base_matrix": float(sv_all[0] / max(sv_all[base_rank_all-1], 1e-30)),
        },
        "source_sha256": {"test_raw_can.jsonl": sha256(args.run / "test_raw_can.jsonl"),
                          "test_signals.npz": sha256(args.run / "test_signals.npz"),
                          "split": sha256(args.split),
                          "urdf": sha256(MODELS / "piper_x.urdf"),
                          "nominal_x": sha256(MODELS / "nominal_x.json"),
                          "script": sha256(Path(__file__).resolve())},
        "runtime": {"python": platform.python_version(), "numpy": np.__version__,
                    "scipy": scipy.__version__, "mujoco": mujoco.__version__},
        "interpretation": [
            "The base coefficients are a minimal torque-prediction parameterization, not six link masses/COMs/inertias.",
            "The SVD right-singular subspace gives a minimum-dimensional coordinate system for the URDF-derived regressor after the declared CAD-scale parameter scaling.",
            "Recovering unique standard link masses/COMs/inertias from base coefficients is impossible without additional assumptions or measurements; any such realization requires a separate constrained inverse map and must preserve the fitted base vector.",
        ],
        "limitations": [
            "The generic state search is a numerical structural-rank procedure, not a symbolic proof of the exact rank.",
            "SDK current-to-torque coefficients have not been calibrated on this particular robot.",
            "Base/table motion was reported but not measured; the model assumes a fixed base.",
            "Joint sign and absolute zero were assumed to match the URDF convention and have not been independently calibrated.",
            "The held-out cycles come from the same 120-second session, not a separate recording.",
        ],
    }
    (out / "base_parameter_identification_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n")
    np.savez_compressed(out / "base_parameter_fit.npz", V=V, V0=V0,
                        parameter_scale=parameter_scale, generic_singular_values=generic_singular,
                        normalized_full_parameter_names=np.asarray(names),
                        beta_train=beta_train, beta_all_data=beta_all,
                        train_prediction=pred_train, test_prediction=pred_test,
                        all_data_prediction=pred_all, valid_source_indices=valid_source_idx)
    print(json.dumps({
        "generic_rigid_rank": f"{rigid_rank}/60",
        "generic_augmented_rank": f"{augmented_rank}/78",
        "train_excitation_rank": f"{base_rank_train}/{augmented_rank}",
        "held_out_cycle_error": validation_metrics["relative_error_norm_ratio"],
        "all_data_in_sample_error": fit_all_metrics["relative_error_norm_ratio"],
        "generic_reconstruction_relative_error": generic_identity_error,
        "nominal_regressor_identity_relative_error": regressor_identity_relative,
        "minimum_base_parameter_fit_successful": report["minimum_base_parameter_fit_successful"],
    }, indent=2), flush=True)


if __name__ == "__main__":
    main()
