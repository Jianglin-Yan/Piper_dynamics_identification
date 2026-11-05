# 联合重拟合结论（2026-11-05）

## 数据与做法

- 动态训练：`inputs/train/balanced_v2_sg41.npz`（3911 样本）
- 动态留出：`inputs/validation/balanced_v1_sg41.npz`（1912 样本）
- 低速摩擦：六轴采集，20 Hz 抽取约 5 万样本，摩擦宽度 0.005
- 加权最小二乘，78 维（60 惯性 + 18 摩擦）

## 结果

| 摩擦权重 | 训练相对误差 | 留出相对误差 | 结论 |
|---:|---:|---:|---|
| 基线（修订 001） | 0.0642 | **0.0835** | 基线 |
| 0.005 | 0.0610 | 0.0854 | 留出 +2.3% |
| 0.010 | 0.0623 | 0.0855 | +2.4% |
| 0.020 | 0.0659 | 0.0875 | +4.8% |
| 0.040 | 0.0738 | 0.0943 | +13% |
| 0.078（等总权重） | 0.0858 | 0.1070 | +28% |

低速重复 2 的逐样本残差几乎不变（J1 0.517→0.518、J2 0.440→0.425、J3 0.543→0.553）；
该逐样本残差主要来自低速位置二阶差分噪声，不能作为模型好坏的判据。

## 结论

- 联合重拟合**没有改善**动态模型：最接近的权重也带来约 2.3% 的留出劣化，正常权重劣化 28%；
- 因此**不替换**当前 `active_revision.json`（修订 001）；零力保持使用的重力参数保持不变；
- 低速摩擦作为**独立模型**保留（`theta_new.npz`，有效范围 ≤0.2 rad/s），供后续摩擦补偿/拖动使用，不并入 78 维动态模型；
- 若后续需要单一模型，可再做 Stribeck + 分段摩擦的形式，而不是简单并入。

## 复现

```bash
PYTHONPATH=src:. python -m piper_dynamics.pipeline.friction_combined --out outputs/friction_fit
PYTHONPATH=src:. python -m piper_dynamics.pipeline.friction_final --out outputs/friction_fit
PYTHONPATH=src:. python -m piper_dynamics.pipeline.friction_joint_refit --out outputs/friction_fit
```
