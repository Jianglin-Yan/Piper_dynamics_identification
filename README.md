# PiPER X 真机动力学参数辨识

本项目用 2026-10-29 的 PiPER X 真机记录，离线构造 URDF 回归器，辨识 54 维最小参数基，并求解带 CAD、几何和伪惯量约束的物理可行候选。`outputs/reference/piper_identification_report.pdf` 是原始对照报告，保留原文件字节。全过程不向机械臂发送指令。

## 目录

| 路径 | 内容 |
| --- | --- |
| `inputs/experiment_20261029/` | 原实验 `data/` 的完整副本，包括 CAN、采样、轨迹、配置及信号文件；保持原始字节 |
| `inputs/splits/` | PDF 所用 SG41 完整周期训练/留出划分 |
| `datasets/standard_piper/` | 8 条标准 PiPER 真机激励轨迹（raw/SG21/SG41、描述、划分和回归器重建脚本） |
| `models/` | PiPER X URDF、名义惯性参数及物理边界使用的碰撞网格 |
| `src/piper_dynamics/minimum_base_svd.py` | CAD 尺度化 SVD 结构秩检验与最小基 OLS |
| `src/piper_dynamics/physical_constraint_fit.py` | 凸优化、伪惯量正定、质量/质心/网格边界与 CAD 先验 |
| `src/piper_dynamics/geometry_bounds.py` | 从碰撞网格计算几何约束 |
| `src/piper_dynamics/robust_estimator_comparison.py` | OLS、目标 RMS 加权 WLS、Huber IRLS 对照 |
| `src/piper_dynamics/split.py`, `model_alignment.py` | 固定划分和模型/实测关节映射校验 |
| `src/piper_dynamics/reference/` | 构造回归器和读取数据所需的固定源码快照；模型文件通过相对链接指向 `models/` |
| `outputs/minimum_base/`, `physical_constraints/` | 可复算的数值结果、参数和审计文件 |
| `outputs/reference/` | 原 PDF，仅作对照 |

旧目录完整备份在 `/home/yjl-10096528/EmbodiedAI/dynamics_identification_backup_2026-11-02`，包括旧仿真辨识代码、历史中间文件和原 PDF。新结构只保留复算这份真机报告所需的源码、输入、模型和主要结果。

> `datasets/standard_piper/` 是标准 PiPER 数据；`inputs/experiment_20261029/` 的原始 manifest 曾把机型写成 `piper_x`，后来确认是普通 PiPER。两套数据不要混用模型：标准 PiPER 用 `datasets/standard_piper/model/`，X 复算用本项目 `models/`。

## 环境与复算

使用 Python 3.10，并在项目根目录执行：

```bash
python3.10 -m venv .venv
.venv/bin/pip install -r requirements.txt
PYTHONPATH=src .venv/bin/python -m piper_dynamics.minimum_base_svd
PYTHONPATH=src .venv/bin/python -m piper_dynamics.physical_constraint_fit
PYTHONPATH=src .venv/bin/python -m piper_dynamics.robust_estimator_comparison
```

本机已有的上层仿真项目环境也可使用：把上述 `.venv/bin/python` 换成 `../.venv/bin/python`。三个入口按顺序运行；后两步依赖第一步的 `outputs/minimum_base/base_parameter_fit.npz`。默认 SG 窗口为 41，使用保存的整周期划分；Huber 最多迭代 60 次，实际 34 次收敛。每个入口可用 `--help` 查看可替换输入、划分和输出目录。生成文件路径写入 JSON，目录移动后需重新运行以更新路径记录。

## PDF 数值对照

| 指标 | 复算值 | PDF 四舍五入值 |
| --- | ---: | ---: |
| 结构秩 | 54 / 78 | 54 / 78 |
| 最小基 OLS，训练/留出/全量残差范数比 | 7.0587% / 7.0764% / 7.3325% | 7.06% / 7.08% / 7.33% |
| 物理约束，训练/留出/全量残差范数比 | 7.4727% / 7.4474% / 7.6631% | 7.47% / 7.45% / 7.66% |
| OLS/WLS/Huber 留出残差范数比 | 7.0764% / 7.3454% / 7.3703% | 7.076% / 7.345% / 7.370% |
| 物理可行候选质量，link1–link6 (kg) | 0.709353 / 1.384638 / 0.907546 / 0.585000 / 0.525000 / 0.278277 | 见原 PDF |

精确数值见 `outputs/minimum_base/base_parameter_identification_report.json`、`outputs/physical_constraints/physically_constrained_identification_report.json` 和 `outputs/robust_comparison.json`。本次数据不能唯一确定逐连杆质量、质心和惯量；物理约束结果只是与数据和先验相容的候选值。电流到力矩系数未针对本机标定，结果不得直接用于控制或当作 CAD 真值。
