# PiPER X 离线辨识算法

本目录发布 EXP-20261029-001 当前使用的核心算法、固定模型资源和示例结果。脚本只做离线分析；原始采集数据、硬件控制入口和过期的 78 维直接拟合产物未发布。

## 文件与依赖

| 文件 | 用途 |
|---|---|
| `identify_urdf_base_parameters.py` | 从 URDF 模型构造回归器，以 CAD 尺度 SVD 建立 54 维最小基，拟合并验证 |
| `identify_physically_constrained_base.py` | 最小基坐标下联合拟合可辨识量与零空间，施加伪惯量 PSD、质量、COM、二阶矩及摩擦约束 |
| `piper_x_geometry.py` | 二进制 STL 读取、坐标变换、凸包与 AABB 几何边界 |
| `compare_robust_base_estimators.py` | OLS、逐关节 WLS 与 Huber IRLS 对比 |
| `run_piper_x_identification.py` | 来源哈希与 URDF/模型一致性审计；历史直接拟合入口供追溯 |
| `../provenance/code_snapshot/` | 被哈希固定的回归器、求解器、CAN 解码和模型适配依赖 |
| `../provenance/piper_x_collision_meshes/` | 官方 PiPER X 与夹爪碰撞网格及来源说明 |
| `sg41/selected_train_test_samples.npz` | 本次实验的固定周期划分索引，不能直接用于其他记录 |

Python 3.10+。从仓库根目录安装：

```bash
python3 -m pip install -r requirements-offline.txt
```

根目录的仿真采集流程还需要外部 `agx_piper_sim`；这里的当前离线流程通过快照模型构建 MuJoCo 模型，不需要该仿真器工程。

## 复算输入与命令

需自行提供本次实验原始数据目录，至少包含 `test_manifest.json`、`test_raw_can.jsonl` 和 `test_signals.npz`。数据加载逻辑见快照的 `real_robot/identify_offline.py:load_measurements`：按逐轴 CAN 接收时间对齐、屏蔽缺帧及对称滤波边缘，以实测速度求导得到加速度。参考轨迹不作为回归输入。

以下命令从仓库根目录运行，输出到新的 `outputs/`，保留随仓库发布的示例结果：

```bash
python3 EXP-20261029-001/train_test/identify_urdf_base_parameters.py \
  --run /path/to/EXP-20261029-001/data \
  --out outputs/urdf_base_parameters --sg-window 41

python3 EXP-20261029-001/train_test/identify_physically_constrained_base.py \
  --run /path/to/EXP-20261029-001/data \
  --base-out outputs/urdf_base_parameters \
  --out outputs/physical_base_fit --sg-window 41

python3 EXP-20261029-001/train_test/compare_robust_base_estimators.py \
  --run /path/to/EXP-20261029-001/data \
  --base-out outputs/urdf_base_parameters \
  --out outputs/robust_comparison --sg-window 41
```

当前两个主入口保留了实验专用的 SG41 划分和模型秩检查；物理约束脚本还要求 955/957 个训练/验证点。`--run` 用于定位这份记录，不表示任意新记录均可直接运行。复用到新记录时，必须重新生成与时间网格、周期和滤波支撑匹配的划分，并核对关节坐标与电流换算系数。不要直接套用归档索引。

## 示例结果与解释

- [当前结果索引](full_data_official_reference/sg41_all_100hz/full_data_report.md)
- [最小基报告](full_data_official_reference/sg41_all_100hz/urdf_base_parameters/base_parameter_identification_report.md)
- [物理约束报告](full_data_official_reference/sg41_all_100hz/physical_base_fit/physically_constrained_identification_report.md)
- [方法调研](IDENTIFICATION_METHOD_REVIEW.md)

最小基留出周期误差为 7.08%，物理约束候选为 7.45%；验证来自同一次记录。电流到力矩系数尚未针对本机标定，底座运动也未测量。24 维结构零空间使逐连杆质量、COM 和惯量无法唯一确定。物理约束解是 CAD 先验下的可行实现，报告标记 `suitable_for_control=false`。

算法文件与当前本地实验版本逐字同步；本次发布未重新运行完整辨识。先前审查发现的固定划分、缓存 seed 元数据和全量物理审计失败门槛等限制未在此次同步中改变。
