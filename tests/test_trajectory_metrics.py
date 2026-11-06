"""Scientific metric checks independent of a particular trajectory implementation."""

import numpy as np
import pytest

from piper_dynamics.core.trajectory_metrics import (
    calculate_information,
    calculate_joint_coverage,
    calculate_residence,
    calculate_speed_coverage,
    calculate_stribeck_information,
)


def test_marginal_range_does_not_claim_visited_volume():
    """验证逐轴行程和包络箱比例不会被标为实际访问体积。"""
    result = calculate_joint_coverage([[0, -1], [2, 1]], [0, -2], [4, 2])
    assert result["range_fraction"] == [0.5, 0.5]
    assert result["joint_box_volume_fraction"] == 0.25
    assert "not visited" in result["box_note"]


def test_information_is_invariant_to_repeated_identical_samples():
    """验证重复相同样本不会虚增归一化后的每样本信息。"""
    matrix = np.array([[1.0, 2.0], [2.0, -1.0], [3.0, 1.0]])
    scales = np.array([2.0, 3.0])
    first = calculate_information(matrix, scales)
    repeated = calculate_information(np.tile(matrix, (7, 1)), scales)
    assert repeated["condition"] == pytest.approx(first["condition"])
    assert repeated["sigma_min"] == pytest.approx(first["sigma_min"])
    assert repeated["log_D_per_direction"] == pytest.approx(
        first["log_D_per_direction"]
    )


def test_condition_number_alone_misses_information_amplitude():
    """验证整体幅度改变信息量，却不改变条件数。"""
    matrix = np.array([[1.0, 0.0], [0.0, 2.0]])
    first = calculate_information(matrix, np.ones(2))
    larger = calculate_information(matrix * 3, np.ones(2))
    assert larger["condition"] == pytest.approx(first["condition"])
    assert larger["sigma_min"] ** 2 == pytest.approx(9 * first["sigma_min"] ** 2)


def test_structural_null_direction_is_not_reported_as_full_rank():
    """验证结构零空间保留秩亏状态，不输出伪满秩指标。"""
    result = calculate_information([[1.0, 0.0], [2.0, 0.0]], np.ones(2))
    assert result["rank"] == 1
    assert result["condition"] is None
    assert result["log_D_per_direction"] is None


def test_residence_breaks_at_data_gaps_and_invalid_samples():
    """验证时间缺口和无效样本都会切断连续停留。"""
    t = np.array([0.0, 0.01, 0.5, 0.51])
    result = calculate_residence(t, np.ones(4, bool), np.ones(4, bool), 0.01)
    assert result["visits"] == 2
    assert result["longest_visit_s"] == pytest.approx(0.02)
    result = calculate_residence(
        np.arange(4) * 0.01, np.ones(4, bool), [1, 0, 1, 1], 0.01
    )
    assert result["visits"] == 2


def test_stationary_dwell_is_not_low_speed_friction_motion():
    """验证纯静止驻留不会被统计为非零低速运动。"""
    t = np.arange(50) * 0.01
    result = calculate_speed_coverage(
        t, np.zeros((50, 1)), np.zeros((50, 1)), np.ones(50, bool), 0.01
    )
    assert result["joints"][0]["low_moving_fraction"] == 0
    assert result["joints"][0]["low_residence"]["seconds"] == 0


def test_no_stribeck_amplitude_means_no_velocity_scale_information():
    """验证静摩擦下降幅度为零时，速度尺度敏感度也为零。"""
    v = np.r_[np.linspace(-0.1, -0.001, 100), np.linspace(0.001, 0.1, 100)]
    result = calculate_stribeck_information(v, [0.005, 0.02], delta_fs=0)
    assert all(x["log_vs_sensitivity_rms_Nm"] == 0 for x in result)
    assert all(x["conditional_min_information_per_sample"] == 0 for x in result)


def test_one_speed_magnitude_cannot_separate_stribeck_shape_parameters():
    """验证单一速度幅值不能独立区分Stribeck形状参数。"""
    v = np.tile([-0.01, 0.01], 100)
    result = calculate_stribeck_information(v, [0.01], smoothing_width=0.002)[0]
    assert result["conditional_min_information_per_sample"] < 1e-25
