"""Offline coverage and experiment-information metrics; no device dependencies."""

from __future__ import annotations

import numpy as np

SPEED_EDGES_RAD_S = np.array(
    [0, 0.002, 0.005, 0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 3, np.inf]
)


def calculate_joint_coverage(q, lower, upper):
    """按各轴实测角度范围计算边际覆盖；q为(N,J)、上下界为(J,)，单位均为rad。

    返回角度范围和行程比例；关节包络箱体积不代表实际访问或无碰撞空间体积。"""
    q = np.asarray(q, float)
    lower, upper = np.asarray(lower, float), np.asarray(upper, float)
    if q.ndim != 2 or q.shape[1:] != lower.shape or upper.shape != lower.shape:
        raise ValueError("Position and bound dimensions do not match")
    if len(q) == 0 or not np.isfinite(q).all() or np.any(upper <= lower):
        raise ValueError("Coverage requires finite positions and increasing bounds")
    lo, hi = q.min(axis=0), q.max(axis=0)
    fractions = (hi - lo) / (upper - lower)
    return {
        "min_deg": np.rad2deg(lo).tolist(),
        "max_deg": np.rad2deg(hi).tolist(),
        "range_deg": np.rad2deg(hi - lo).tolist(),
        "range_fraction": fractions.tolist(),
        "mean_marginal_fraction": float(fractions.mean()),
        "joint_box_volume_fraction": float(np.prod(fractions)),
        "box_note": "Bounding box relative to the joint-limit box, not visited or collision-free workspace volume",
    }


def calculate_bin_occupancy(q, lower, upper, bins_per_axis):
    """统计关节构型网格被样本访问的格数；角度与上下界单位一致。

    只计范围内样本；结果依赖分辨率和样本数量，不把总格数解释为无碰撞空间。"""
    q, lo, hi = np.asarray(q), np.asarray(lower), np.asarray(upper)
    normalized = (q - lo) / (hi - lo)
    inside = np.all((normalized >= 0) & (normalized <= 1), axis=1)
    indices = np.floor(normalized[inside] * bins_per_axis).astype(int)
    indices = np.minimum(indices, bins_per_axis - 1)
    count = len(np.unique(indices, axis=0))
    return {
        "bins_per_axis": bins_per_axis,
        "visited_bins": count,
        "total_joint_box_bins": bins_per_axis ** q.shape[1],
        "note": "Sample-count/grid dependent; denominator includes unverified collision states",
    }


def calculate_residence(t, selected, valid, nominal_dt):
    """累计布尔选择区间的停留时间；t与nominal_dt单位为秒。

    无效样本、不相邻样本或超过1.5倍采样间隔的缺口均分段，未选中时返回零停留。"""
    t, selected, valid = (
        np.asarray(t),
        np.asarray(selected, bool),
        np.asarray(valid, bool),
    )
    ids = np.flatnonzero(selected & valid)
    if not len(ids):
        return {
            "seconds": 0.0,
            "visits": 0,
            "longest_visit_s": 0.0,
            "visits_at_least_0p25s": 0,
        }
    breaks = (np.diff(ids) != 1) | (np.diff(t[ids]) > 1.5 * nominal_dt)
    groups = np.split(ids, np.flatnonzero(breaks) + 1)
    durations = np.array([t[g[-1]] - t[g[0]] + nominal_dt for g in groups])
    return {
        "seconds": float(len(ids) * nominal_dt),
        "visits": len(groups),
        "longest_visit_s": float(durations.max()),
        "visits_at_least_0p25s": int(np.sum(durations >= 0.25 - 1e-8)),
    }


def calculate_speed_coverage(t, velocity, acceleration, valid, dt):
    """计算逐轴速度分箱及低速连续停留，排除静止驻留。

    velocity/acceleration为(N,J)，单位rad/s、rad/s²；valid为(N,)，dt单位秒。
    低速与稳定阈值是诊断口径，不是已辨识的摩擦参数；没有有效样本时抛出异常。"""
    velocity, acceleration, valid = (
        np.asarray(velocity),
        np.asarray(acceleration),
        np.asarray(valid, bool),
    )
    if not valid.any():
        raise ValueError("No valid speed samples")
    rows = []
    for joint in range(velocity.shape[1]):
        v, a = velocity[:, joint], acceleration[:, joint]
        speed = abs(v)
        low = (speed >= 0.002) & (speed < 0.02)
        stable_low = low & (abs(a) <= 0.05)
        counts = np.histogram(speed[valid], bins=SPEED_EDGES_RAD_S)[0]
        rows.append(
            {
                "joint": joint + 1,
                "peak_rad_s": float(speed[valid].max()),
                "p95_rad_s": float(np.percentile(speed[valid], 95)),
                "rms_rad_s": float(np.sqrt(np.mean(v[valid] ** 2))),
                "bin_fractions": (counts / valid.sum()).tolist(),
                "low_moving_fraction": float(np.mean(low[valid])),
                "low_positive_fraction": float(np.mean((low & (v > 0))[valid])),
                "low_negative_fraction": float(np.mean((low & (v < 0))[valid])),
                "above_0p2_fraction": float(np.mean((speed >= 0.2)[valid])),
                "low_residence": calculate_residence(t, low, valid, dt),
                "stable_low_residence": calculate_residence(t, stable_low, valid, dt),
                "low_positive_stable": calculate_residence(
                    t, stable_low & (v > 0), valid, dt
                ),
                "low_negative_stable": calculate_residence(
                    t, stable_low & (v < 0), valid, dt
                ),
            }
        )
    return {"valid_duration_s": float(valid.sum() * dt), "joints": rows}


def calculate_information(matrix, fixed_scales, rank_tolerance=1e-8):
    """按固定列尺度和标量行数归一化，计算等噪声假设下的信息代理。

    各轨迹必须使用同一审计基和列尺度；秩亏时条件数与对数D指标返回None。
    这些指标未标定有色测量噪声，不能视为物理Fisher信息或精度保证。"""
    matrix, scales = np.asarray(matrix, float), np.asarray(fixed_scales, float)
    if matrix.ndim != 2 or matrix.shape[1:] != scales.shape or np.any(scales <= 0):
        raise ValueError("Information matrix requires matching positive fixed scales")
    if (
        not np.isfinite(matrix).all()
        or not np.isfinite(scales).all()
        or len(matrix) == 0
    ):
        raise ValueError("Information matrix must be finite and nonempty")
    # 同时固定列尺度并按标量行数归一化，避免仅因重复采样而抬高每样本信息。
    singular = np.linalg.svd(matrix / scales / np.sqrt(len(matrix)), compute_uv=False)
    rank = int(np.sum(singular > singular[0] * rank_tolerance)) if singular[0] else 0
    full_rank = rank == matrix.shape[1]
    return {
        "rank": rank,
        "columns": matrix.shape[1],
        "scalar_rows": len(matrix),
        "condition": float(singular[0] / singular[-1]) if full_rank else None,
        "sigma_min": float(singular[-1]),
        "log_D_per_direction": float(2 * np.log(singular).mean())
        if full_rank
        else None,
        "singular_values": singular.tolist(),
    }


def calculate_stribeck_information(
    velocity, vs_values, smoothing_width=0.02, delta_fs=0.1
):
    """投影掉粘性、库仑和偏置方向后，计算Stribeck局部敏感度。

    velocity/vs_values/smoothing_width单位rad/s，delta_fs单位Nm；返回各速度尺度
    的每样本条件信息。幅度为零时速度尺度不可观测；假设值不是实测拟合参数。"""
    v = np.asarray(velocity, float)
    c = np.tanh(v / smoothing_width)
    nuisance = np.column_stack([v, c, np.ones(len(v))])
    rows = []
    for vs in vs_values:
        ratio = (v / vs) ** 2
        exponential = np.exp(-ratio)
        jacobian = np.column_stack(
            [0.1 * c * exponential, delta_fs * c * exponential * 2 * ratio]
        )
        # 先消去能由既有线性摩擦项解释的方向，再评价非线性参数的独立敏感度。
        coefficients = np.linalg.lstsq(nuisance, jacobian, rcond=1e-10)[0]
        residual = jacobian - nuisance @ coefficients
        singular = np.linalg.svd(residual / np.sqrt(len(v)), compute_uv=False)
        rows.append(
            {
                "vs_rad_s": float(vs),
                "smoothing_width_rad_s": smoothing_width,
                "delta_fs_reference_Nm": delta_fs,
                "conditional_min_information_per_sample": float(singular[-1] ** 2),
                "conditional_information_trace_per_sample": float(np.sum(singular**2)),
                "joint_sensitivity_condition": float(singular[0] / singular[-1])
                if singular[-1] > 1e-15
                else None,
                "log_vs_sensitivity_rms_Nm": float(
                    np.sqrt(np.mean(residual[:, 1] ** 2))
                ),
            }
        )
    return rows


class InformationRegressor:
    """按项目惯性列顺序构造Pinocchio回归器，复用逐样本计算缓存。

    实例持有可变data，不供并发计算共享；列转换由存档MuJoCo回归矩阵交叉复核。"""

    def __init__(self, urdf_path, inertial_indices):
        """从六轴PiPER URDF建立模型并映射惯性列；非六轴模型立即拒绝。"""
        import pinocchio as pin

        self.pin = pin
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        if self.model.nv != 6:
            raise ValueError("Trajectory audit supports the standard six-joint PiPER")
        permutation = np.array([0, 1, 2, 3, 4, 7, 5, 8, 9, 6])
        columns = np.concatenate(
            [10 * joint + np.argsort(permutation) for joint in range(6)]
        )
        self.columns = columns[np.asarray(inertial_indices, int)]

    def build(self, q, velocity, acceleration):
        """构造(N×6,惯性列数+18)矩阵；q、速度、加速度均为(N,6)的SI单位数组。

        按样本复用data并复制回归结果，再追加既有粘性/tanh/偏置特征；不修改输入。"""
        from piper_dynamics.core.regression_model import make_friction

        rigid = np.array(
            [
                self.pin.computeJointTorqueRegressor(self.model, self.data, qi, vi, ai)[
                    :, self.columns
                ].copy()
                for qi, vi, ai in zip(q, velocity, acceleration)
            ]
        )
        return np.concatenate([rigid, make_friction(velocity)], axis=2).reshape(
            -1, len(self.columns) + 18
        )
