# PiPER X 最小基参数上的物理约束辨识

> 本报告把结构最小参数、数据拟合和逐连杆物理实现分开处理。质量/COM/惯量表是受 CAD 先验和明确边界约束的物理可行实现，不是逐参数唯一真值。
> **判定：最小基参数拟合完成；真实逐连杆质量辨识未成功。** 这里的“物理可行”只表示满足本报告采用的网格、伪惯量和 CAD 先验约束。

## 采用的方法

从快照 URDF 的回归矩阵构造 `theta = S(V beta + V0 gamma)`，其中 `beta` 是 54 维最小动力学参数，`gamma` 是 24 维标准参数零空间。力矩预测只依赖 `beta`；`gamma` 只在动力学等价的标准连杆参数实现之间选择。`S` 使用名义 CAD 的质量、长度和惯性尺度，避免有限差分噪声列被放大。
训练目标采用按关节目标力矩 RMS 归一化的加权最小二乘。每个 link 的质量限定在快照 CAD 的 0.5–1.5 倍；COM 必须位于同一 AgileX commit 的 PiPER X 碰撞网格凸包外扩 10 mm 内；伪惯量半正定矩阵的三个二阶矩对角元素还受网格外包 AABB 限制。黏性和库仑摩擦系数约束非负，常值 offset 可正可负。CAD 先验采用伪惯量上的 log-det Bregman divergence；正则强度用完整留出周期选择。

## 为什么之前的零质量解不可信

训练集无约束 OLS 的结构最小基在留出集相对误差为 7.08%，但固定该基向量后，仅施加伪惯量半正定就无法找到标准逐连杆参数实现。这说明无约束 OLS 基本身超出了物理可行集合；后处理随意分配质量不能解决，需要在物理可行集合内重新拟合基参数。
物理约束后，54 维基向量相对无约束 OLS 的相对变化为 71.8%。因此这一步显著改变了数据拟合出的基参数；留出误差接近只说明预测相近，不能证明逐 link 质量是真机参数。

## 留出验证

通过留出周期选择的方法为 `weighted_l2`，CAD 先验权重 λ=0.0001。训练、留出周期和全量重拟合内误差如下：

| 拟合/验证段 | 相对误差（残差范数/目标范数） | 宏平均逐关节相对 RMSE |
|---|---:|---:|
| 训练集 | 7.47% | 18.78% |
| 留出完整周期 | 7.45% | 18.44% |
| 全量 120 秒重新拟合（训练内） | 7.66% | 20.54% |

## 全量数据物理参数实现

| link | 质量 (kg) | 相对 CAD | COM (m, link 坐标) | Icom 主值 (kg·m²) | 网格约束 |
|---|---:|---:|---|---|---|
| link1 | 0.70935 | 0.999× | 0.00050, -0.00036, -0.00315 | 0.0003968, 0.0004225, 0.0004764 | 是 |
| link2 | 1.38464 | 1.194× | -0.04129, -0.00225, 0.00384 | 0.0009266, 0.1462, 0.1463 | 是 |
| link3 | 0.90755 | 1.355× | 0.14201, -0.00571, -0.00556 | 0.0003254, 0.007256, 0.007276 | 是 |
| link4 | 0.58500 | 1.500× | -0.02297, 0.02333, -0.02446 | 0.0001511, 0.000298, 0.0003356 | 是 |
| link5 | 0.52500 | 1.500× | -0.03522, 0.01309, 0.01298 | 0.0001296, 0.0001483, 0.0001667 | 是 |
| link6 | 0.27828 | 0.500× | -0.01014, -0.00902, -0.02159 | 0.0001489, 0.0002987, 0.0003463 | 是 |

约束边界审计：`link2, link4, link5, link6` 至少有一项质量、COM 凸包或二阶矩几何约束接近活跃。伪惯量 PSD 和几何检查通过，只能说明这一组参数与指定外包网格相容；它仍不是由力矩数据唯一测出的逐 link 真值。

以上全量参数是全数据重新拟合的结果；留出误差只对应训练集所选的模型，避免把全量拟合误差冒充验证误差。逐参数、几何边界和 PSD 特征值见 `physically_constrained_link_parameters.csv`、`piper_x_geometry_constraints.json`、`physically_constrained_standard_parameters.csv` 和 JSON 报告。

## 限制

- The 24-dimensional standard-parameter nullspace means per-link mass/COM/inertia are not uniquely determined by this torque dataset. The reported link values are one feasible realization selected by the explicit CAD prior and bounds.
- Physical constraints changed the 54-dimensional training base vector by 71.8% in relative norm from unconstrained OLS; similar validation error does not make the resulting per-link values measured or unique.
- The following links reach a configured mass, COM mesh-hull, or second-moment geometry bound: link2, link4, link5, link6. Bound-active values indicate the result is constraint-limited and should not be treated as independently measured parameters.
- 当前电流到力矩比例来自 SDK 型号参考值，未在这台机械臂上校准。
- 采集时曾报告底座/桌面运动，但没有测量；固定基座动力学模型无法区分这类扰动。
- 质量 0.5–1.5 倍 CAD 是显式先验，不是本台机械臂的实测值；网格约束来自官方碰撞几何的外包络，不描述连杆内部真实密度分布。
- 快照模型的 link6 在闭合夹爪姿态下合并了夹爪惯性。
- 验证数据来自同一段 120 秒记录的完整留出周期，不是另一次独立采集。
- 网格外包络加入了明确的几何膨胀裕量；link6 的几何同时覆盖夹爪开度范围，但惯性先验仍按闭合夹爪聚合值计算。
- 当前使用加权最小二乘，没有对离群残差进行 Huber/IRLS 鲁棒降权；若电流尖峰或同步异常明显，应先确认传感信号并另行做鲁棒性对照。
