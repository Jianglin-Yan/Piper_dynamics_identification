"""Read-only audit of existing PiPER excitation coverage and information.

This evaluates frozen trajectories; it never optimizes or changes a control task.
"""

from __future__ import annotations

import argparse
import json
import platform
import subprocess
from pathlib import Path

import numpy as np
from scipy.ndimage import minimum_filter1d
from scipy.signal import savgol_filter

from piper_dynamics.core.dataset_io import load_archive
from piper_dynamics.core.regression_model import make_friction
from piper_dynamics.core.trajectory_metrics import (
    InformationRegressor,
    calculate_bin_occupancy,
    calculate_information,
    calculate_joint_coverage,
    calculate_speed_coverage,
    calculate_stribeck_information,
)
from piper_dynamics.paths import (
    IDENTIFICATION_ARCHIVE,
    OUTPUTS,
    PROJECT_ROOT,
    RAW_INPUTS,
    RUNS,
    STANDARD_MODELS,
)
from piper_dynamics.utils.hashing import sha256

GRID_DT_S = 0.01
VS_VALUES_RAD_S = [0.003, 0.005, 0.01, 0.02, 0.05, 0.1]


def read_json(path):
    """读取指定JSON记录；缺失、编码或解析错误原样交由调用方处理。"""
    return json.loads(Path(path).read_text())


def source_directory(metadata, archive):
    """按元数据中的实验号和子目录定位历史原始记录，不复制或改写归档。"""
    return (
        archive
        / "experiments"
        / metadata["source_experiment"]
        / metadata["source_subdir"]
    )


def interpolate_columns(t, values, grid):
    """将六轴(N,6)信号按时间线性插值到grid；沿用np.interp的端点语义。"""
    return np.column_stack([np.interp(grid, t, values[:, joint]) for joint in range(6)])


def load_position_derivatives(path):
    """将位置重采样到10ms网格并求SG导数，屏蔽缺口和完整滤波支撑。

    输出q为rad、v为rad/s、a为rad/s²，含SG41和驱动速度对照；不改原始NPZ。"""
    with np.load(path) as data:
        t, q = data["t_test_s"], data["q"]
        drive = data["qd"]
        good = data["valid_measurement"].astype(bool)
    grid = np.arange(t[0], t[-1] + GRID_DT_S * 0.01, GRID_DT_S)
    position = interpolate_columns(t, q, grid)
    right = np.clip(np.searchsorted(t, grid), 1, len(t) - 1)
    good_grid = good[right] & good[right - 1] & (t[right] - t[right - 1] <= 0.03)
    good_grid &= (grid >= t[0]) & (grid <= t[-1])
    # Largest filter window is 51 samples; constant padding excludes both edges.
    valid = minimum_filter1d(good_grid.astype(int), 51, mode="constant", cval=0).astype(
        bool
    )
    velocity = savgol_filter(position, 21, 3, deriv=1, delta=GRID_DT_S, axis=0)
    velocity41 = savgol_filter(position, 41, 3, deriv=1, delta=GRID_DT_S, axis=0)
    acceleration = savgol_filter(position, 51, 3, deriv=2, delta=GRID_DT_S, axis=0)
    return {
        "t": grid,
        "q": position,
        "v": velocity,
        "v41": velocity41,
        "a": acceleration,
        "valid": valid,
        "drive": interpolate_columns(t, drive, grid),
        "original_q": q[good],
    }


def add_stribeck_diagnostics(velocity, valid):
    # Uniform 50 ms spacing limits array size. This is not an independence claim.
    """在有效速度上按50ms抽样，扫描两种平滑宽度的局部敏感度。

    抽样用于限制计算量，不代表时间样本独立，也不估计真实Stribeck参数。"""
    selected = velocity[valid][::5]
    return {
        str(width): [
            calculate_stribeck_information(selected[:, joint], VS_VALUES_RAD_S, width)
            for joint in range(6)
        ]
        for width in (0.02, 0.002)
    }


def audit_dynamic(folder, archive, revision, builder, fixed_scales):
    """审计一条历史动态轨迹，返回指标、原始角度及相空间预览。

    复核源哈希和共同基回归器，再比较实测、规划及位置微分口径；所有来源只读。
    输出的信息量使用共同列尺度，不能由该评分单独推出物理精度或激励最优性。"""
    metadata = read_json(folder / "metadata.json")
    data = load_archive(folder / "sg21.npz")
    expected = metadata["files"]["sg21"]["sha256"]
    if sha256(folder / "sg21.npz") != expected:
        raise ValueError(f"Archived source hash mismatch: {folder}")
    source = source_directory(metadata, archive)
    signals = load_position_derivatives(source / "test_signals.npz")
    plan = metadata["plan"]
    steady = signals["valid"] & (signals["t"] >= plan["ramp_s"])
    steady &= signals["t"] <= plan["duration_s"] - plan["ramp_s"]
    lower = np.deg2rad(metadata["limit_audit"]["reported_lower_deg"])
    upper = np.deg2rad(metadata["limit_audit"]["reported_upper_deg"])
    reference = dict(np.load(source / "reference_trajectory.npz"))
    planned = [
        interpolate_columns(reference["t"], reference[key], data["t"])
        for key in ("q_ref", "qd_ref", "qdd_ref")
    ]
    # 使用同一份历史缓存核验列转换；发现来源或数值差异立即停止，不静默换模型。
    with np.load(revision / "datasets" / folder.name / "sg21.npz") as cached:
        if not np.allclose(data["q"], cached["q"], atol=0, rtol=0):
            raise ValueError(f"Cache positions differ: {folder.name}")
        matrix_drive = np.concatenate(
            [cached["YB"], make_friction(data["qd"])], axis=2
        ).reshape(-1, 54)
        check = builder.build(data["q"][:10], data["qd"][:10], data["qdd"][:10])
        regressor_error = float(abs(check - matrix_drive[:60]).max())
        if regressor_error > 1e-7:
            raise ValueError(f"Regressor conversion failed: {regressor_error}")
    vpos = interpolate_columns(signals["t"], signals["v"], data["t"])
    apos = interpolate_columns(signals["t"], signals["a"], data["t"])
    row = {
        "name": folder.name,
        "source": str(source),
        "metadata": {
            key: metadata[key]
            for key in ("source_experiment", "source_subdir", "source_run")
        },
        "execution": {
            key: plan.get(key)
            for key in ("scale", "speed_scale", "duration_s", "ramp_s", "candidate")
        },
        "firmware_lower_deg": np.rad2deg(lower).tolist(),
        "firmware_upper_deg": np.rad2deg(upper).tolist(),
        "coverage": calculate_joint_coverage(signals["original_q"], lower, upper),
        "planned_coverage": calculate_joint_coverage(reference["q_ref"], lower, upper),
        "planning_interval_coverage": calculate_joint_coverage(
            signals["original_q"],
            np.deg2rad(plan["reference_lower_deg"]),
            np.deg2rad(plan["reference_upper_deg"]),
        ),
        "configuration_grid": [
            calculate_bin_occupancy(data["q"], lower, upper, bins) for bins in (10, 20)
        ],
        "speed_position_sg21": calculate_speed_coverage(
            signals["t"], signals["v"], signals["a"], steady, GRID_DT_S
        ),
        "speed_position_sg41": calculate_speed_coverage(
            signals["t"], signals["v41"], signals["a"], steady, GRID_DT_S
        ),
        "speed_drive": calculate_speed_coverage(
            signals["t"], signals["drive"], signals["a"], steady, GRID_DT_S
        ),
        "speed_planned": calculate_speed_coverage(
            reference["t"],
            reference["qd_ref"],
            reference["qdd_ref"],
            (reference["t"] >= plan["ramp_s"])
            & (reference["t"] <= plan["duration_s"] - plan["ramp_s"]),
            GRID_DT_S,
        ),
        "information": {
            "measured_drive_sg21": calculate_information(matrix_drive, fixed_scales),
            "planned_matched": calculate_information(
                builder.build(*planned), fixed_scales
            ),
            "measured_position_derivatives": calculate_information(
                builder.build(data["q"], vpos, apos), fixed_scales
            ),
        },
        "stribeck_sensitivity": add_stribeck_diagnostics(signals["v"], steady),
        "checks": {
            "regressor_vs_cache_max_error": regressor_error,
            "archived_sg21_hash_match": True,
        },
        "input_sha256": {
            str(folder / "sg21.npz"): expected,
            str(source / "reference_trajectory.npz"): sha256(
                source / "reference_trajectory.npz"
            ),
            str(source / "test_signals.npz"): sha256(source / "test_signals.npz"),
        },
    }
    print("AUDITED", folder.name, flush=True)
    return (
        row,
        signals["original_q"],
        (signals["t"][steady], signals["q"][steady], signals["v"][steady]),
    )


def audit_friction(joint, run, run_root):
    """按采集相位审计指定关节的低速摩擦记录，排除hold与无效区间。

    返回速度档实际分布、低速停留、敏感度和源哈希；没有移动相位时抛出异常。"""
    path = run_root / run / "test_signals.npz"
    signals = load_position_derivatives(path)
    with np.load(path) as data:
        phase = data["trial_phase"].astype(str)
        source_t = data["t_test_s"]
        tags = data["trial_speed_bound_rad_s"]
        direction = data["trial_direction"].astype(str)
    indices = np.clip(
        np.searchsorted(source_t, signals["t"], side="right") - 1, 0, len(source_t) - 1
    )
    active = signals["valid"] & np.isin(
        phase[indices], ["forward", "reverse", "return"]
    )
    if not active.any():
        raise ValueError(f"No recognized moving phases in friction run {run}")
    coverage = calculate_speed_coverage(
        signals["t"], signals["v"], signals["a"], active, GRID_DT_S
    )
    speed = signals["v"][:, joint - 1]
    platform_rows = []
    for target in np.unique(tags):
        if target <= 0:
            continue
        for label in np.unique(direction):
            mask = (
                active
                & np.isclose(tags[indices], target)
                & (direction[indices] == label)
            )
            if mask.sum() < 20:
                continue
            platform_rows.append(
                {
                    "target_rad_s": float(target),
                    "direction": str(label),
                    "median_actual_speed_rad_s": float(np.median(abs(speed[mask]))),
                    "seconds": float(mask.sum() * GRID_DT_S),
                }
            )
    row = {
        "joint": joint,
        "run": run,
        "phase_labels": np.unique(phase).tolist(),
        "speed": coverage["joints"][joint - 1],
        "platforms": platform_rows,
        "stribeck_sensitivity": {
            str(width): calculate_stribeck_information(
                speed[active][::5], VS_VALUES_RAD_S, width
            )
            for width in (0.02, 0.002)
        },
        "input_sha256": sha256(path),
    }
    print("FRICTION", joint, flush=True)
    return row


def audit_candidate_banks(archive):
    """只读冻结候选库，保留优化预算、历史模型及普通PiPER复核来源。"""
    rows = []
    for path in sorted((archive / "trajectories").glob("candidates-v*.json")):
        bank = read_json(path)
        provenance = bank["provenance"]
        rows.append(
            {
                "file": str(path),
                "sha256": sha256(path),
                "model": bank.get("model"),
                "evaluation_revision": bank.get("evaluation_revision"),
                "provenance": provenance,
                "evaluation": bank["evaluation"],
                "optimization_log": bank["optimization_log"],
                "candidates": {
                    name: {
                        "execution": {
                            **bank["execution"],
                            **candidate.get("execution", {}),
                        },
                        "status": candidate["status"],
                        "standard_piper_recheck": candidate.get("piper_recheck"),
                        "legacy_hardware_validation": candidate.get(
                            "legacy_hardware_validation"
                        ),
                    }
                    for name, candidate in bank["candidates"].items()
                },
            }
        )
    return rows


def add_relative_information(rows):
    """就地补充相对old_fast_0的信息指标；同一信号口径分别取基线。

    区分每样本指标和带样本数的总量代理，不将相关样本声称为独立观测。"""
    baseline = next(row for row in rows if row["name"] == "old_fast_0")
    for row in rows:
        for scope, metric in row["information"].items():
            ref = baseline["information"][scope]
            metric["minimum_information_relative"] = (
                metric["sigma_min"] / ref["sigma_min"]
            ) ** 2
            metric["D_information_relative"] = np.exp(
                metric["log_D_per_direction"] - ref["log_D_per_direction"]
            )
            metric["total_D_relative_with_sample_count"] = (
                metric["D_information_relative"]
                * metric["scalar_rows"]
                / ref["scalar_rows"]
            )


def main(argv=None):
    """组织只读轨迹与摩擦审计，将JSON、CSV、图表写入新结果目录。

    已有metrics.json时拒绝覆盖；需要已有采集和修订归档，不优化轨迹或连接CAN。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--archive", type=Path, default=IDENTIFICATION_ARCHIVE)
    parser.add_argument("--revision", default="PIPER-20261102-001")
    parser.add_argument("--run-root", type=Path, default=RUNS)
    parser.add_argument(
        "--output", type=Path, default=OUTPUTS / "trajectory_audit_20261105"
    )
    args = parser.parse_args(argv)
    if (args.output / "metrics.json").exists():
        parser.error("Output already contains results; choose a fresh output directory")
    # 建立固定的历史基与列尺度，再逐条评分；本流程不修改冻结候选或训练划分。
    revision = args.archive / "revisions" / args.revision
    basis = read_json(revision / "basis.json")
    scales = np.asarray(
        read_json(revision / "trajectory_information.json")["fixed_scales"]
    )
    builder = InformationRegressor(STANDARD_MODELS / "piper.urdf", basis["indices"])
    rows, positions, previews = [], [], {}
    for folder in sorted(RAW_INPUTS.iterdir()):
        if not (folder / "metadata.json").exists():
            continue
        row, q, preview = audit_dynamic(folder, args.archive, revision, builder, scales)
        rows.append(row)
        positions.append(q)
        previews[folder.name] = preview
    add_relative_information(rows)
    run_index = read_json(PROJECT_ROOT / "inputs/friction_runs.json")["runs"]
    friction = [
        audit_friction(int(j), run, args.run_root)
        for j, run in sorted(run_index.items())
    ]
    lower, upper = (
        np.deg2rad(rows[0]["firmware_lower_deg"]),
        np.deg2rad(rows[0]["firmware_upper_deg"]),
    )
    result = {
        "schema_version": 1,
        "model": "standard PiPER",
        "revision": args.revision,
        "code_commit": subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=PROJECT_ROOT, text=True
        ).strip(),
        "python": platform.python_version(),
        "dynamic": rows,
        "friction": friction,
        "joint_union_coverage": calculate_joint_coverage(
            np.vstack(positions), lower, upper
        ),
        "candidate_banks": audit_candidate_banks(args.archive),
        "method": {
            "speed_grid_s": GRID_DT_S,
            "low_moving_rad_s": [0.002, 0.02],
            "stable_low_acceleration_rad_s2": 0.05,
            "long_visit_s": 0.25,
            "high_diagnostic_threshold_rad_s": 0.2,
            "vs_scan_rad_s": VS_VALUES_RAD_S,
            "information_basis": "36 common inertial columns + 18 viscous/tanh/offset columns",
            "information_scaling": "fixed archived column RMS, scalar-row normalized, equal-noise proxy",
            "speed_source": "SG21 derivative of resampled measured position; SG41 and drive feedback checks",
            "limits": "recorded per-run hardware joint limits, not measured collision-free workspace",
        },
    }
    # 指标完成后写入JSON；若后续绘图失败，目录仍可能保留部分输出。
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "metrics.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2)
    )
    from piper_dynamics.pipeline.trajectory_audit_report import write_report

    write_report(result, previews, args.output)
    print("REPORT", args.output, flush=True)


if __name__ == "__main__":
    main()
