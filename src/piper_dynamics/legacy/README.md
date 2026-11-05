# legacy：旧 PiPER X 复算代码（仅存档）

这些脚本用于追溯 `outputs/reference/piper_identification_report.pdf` 里的历史数值。
它们依赖的原始 CAN 记录（原 `inputs/experiment_20261029/`）和旧周期划分
（原 `inputs/splits/`）已在输入整理时删除，因此**当前仓库里无法运行**。

| 文件 | 作用 |
| --- | --- |
| `minimum_base_svd.py` | 旧流程：URDF 结构最小基 + SVD 秩分析 |
| `physical_constraint_fit.py` | 旧流程：带几何/伪惯量约束的拟合 |
| `robust_estimator_comparison.py` | 旧流程：不同鲁棒估计器对比 |
| `geometry_bounds.py` | 从碰撞网格推导连杆几何边界 |
| `model_alignment.py` | URDF 与名义参数的一致性审计 |
| `validated_cycle_split.py` | 校验旧的奇数/偶数周期划分 |
| `legacy_paths.py` | 上述脚本使用的路径 |
| `snapshot/` | 冻结的旧源码快照（`identify.py`、`real_robot/`）|

当前标准 PiPER 流程完全不导入这个包；如果要复算旧报告，先从 git 历史恢复旧输入，
再运行 `python -m piper_dynamics.legacy.minimum_base_svd` 等入口。

注意：`snapshot/regressor.py` 只是把实现转发到
`piper_dynamics.core.rigid_body_regressor`，避免两份实现漂移。
