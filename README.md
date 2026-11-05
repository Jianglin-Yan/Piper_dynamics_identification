# PiPER 动力学辨识与控制实验

项目只支持普通 PiPER。离线辨识、实机运行、仿真与分析共用 `src/piper_dynamics`，仓库顶层沿用 main 的输入、模型、输出管理方式。

开发遵循 [代码规范](CODING_STANDARD.md) 与 [项目约定](AGENTS.md)；控制链路见 [架构说明](ARCHITECTURE.md)，本次整理的变化与验证见 [验收报告](docs/refactor_report.md)。

## 目录

完整目录树、src 内部分工及依赖边界见根目录 [仓库结构与代码架构](ARCHITECTURE.md)。下表为常用位置索引。

| 位置 | 职责 |
| --- | --- |
| `src/piper_dynamics/core/` | 刚体回归器、模型/重力、摩擦特征、QR 与 SVD 求解、参数表 |
| `src/piper_dynamics/pipeline/` | 数据划分、离线拟合、实测分析、摩擦分析、轨迹设计 |
| `src/piper_dynamics/real_robot/` | 入口/配置、运行编排、状态机、控制方式、后端、任务、记录与交互 |
| `src/piper_dynamics/utils/` | 通用哈希、绘图与序列化 |
| `scripts/` | 薄启动脚本及共享解释器选择 |
| `tests/` | 控制、通信、数值、会话和兼容验证；`fixtures/` 为观测基线和官方协议参考 |
| `inputs/raw/standard_piper/` | 8 条真实 PiPER 激励轨迹的 raw/SG21/SG41 与来源索引 |
| `inputs/train/`、`inputs/validation/` | 按整条轨迹划分的训练/留出输入 |
| `inputs/friction_runs.json` | 六轴低速摩擦采集记录索引，不复制原始 runs |
| `models/standard_piper/` | 唯一的 PiPER URDF、名义参数、回归模型与夹爪描述 |
| `outputs/` | 离线结果、参数、图表与报告 |
| `runs/` | 每次采集的原始记录，继续由 Git 忽略 |
| `docs/` | 架构补充、迁移与验证记录 |
| 根目录 `real_robot/`、`tools/`、`excitation.py` | 旧调用兼容转发或资源路径别名，不保留另一份业务实现 |

相邻 `../identification/` 继续管理实验索引、修订与验证归档，本次整理不改写其历史记录。共享网格仍保留在 `models/collision_meshes/`；项目的 X 专用实现、模型与结果已按 [文件清单](docs/removed_x_files.json) 删除，Git 历史仍可追溯。

## 环境与测试

Python >=3.10；本次验证使用 NumPy 2.2.6、SciPy 1.18.1、MuJoCo 3.8.1 与 piper-sdk 0.6.2。使用项目环境安装，不直接升级其他正在使用的控制环境：

```bash
python -m pip install -r requirements.txt
python -m pip install -e '.[hardware,test]'
python -m pytest -q
```

源目录方式也可直接运行：`PYTHONPATH=src python -m piper_dynamics --help`。轨迹设计额外需要 Pinocchio，见 `pyproject.toml` 的 analysis 可选依赖；纯拟合 `--no-plots` 可不加载 matplotlib。包配置用于仓库内/可编辑安装，输入与模型仍使用本仓库的目录管理。

## 离线辨识

```bash
PYTHONPATH=src python -m piper_dynamics split
PYTHONPATH=src python -m piper_dynamics fit
PYTHONPATH=src python -m piper_dynamics show
```

当前划分为 balanced_v2 整条训练、balanced_v1 整条留出，分别 3911、1912 点；78 列设计矩阵的本次数据数值秩为 54。复算训练/验证相对误差约 5.84364% / 8.90836%。验证数据不参与求解。

`fit` 输出 NPZ、JSON、逐参数 CSV、Markdown 和可选力矩对比图；`regressor` 单独生成 60 列刚体回归矩阵。候选力矩来自未针对本机标定的电流换算；拟合结果不代表唯一、已标定的逐连杆物理参数。

旧实测记录使用原 QR/最小二乘口径分析，不与 main 的 CAD 尺度化 SVD 混成同一方法：

```bash
PYTHONPATH=src python -m piper_dynamics.pipeline.measured_run RUN_DIRECTORY --out OUTPUT_DIRECTORY
```

该入口保留原有同轨迹周期划分和报告字段；它的验证口径与 main 的跨轨迹验证不同。

## 实机与仿真入口

```bash
bash scripts/test_mit.sh sim --ff-scale 0
bash scripts/test_mit.sh sim --ff-scale 0.25
bash scripts/test_friction.sh sim --help
bash scripts/test_drag.sh sim
bash scripts/pilot.sh sim --help
```

`scripts/test_static.sh`、`test_reciprocal.sh`、`test_payload.sh` 继续选择对应 MOVE J 任务。`test_drag.sh` 为零力拖动：LB+X 准备并渐变到 Kp 2／前馈 100%，LB+Y 开始记录（先静置再手推），再次 LB+X 反向渐变并经 MIT 回到停靠位。实机入口仍需显式选择 hardware，并保留当前的检查、手柄/自动运行、停止回收与失能流程。控制参数取当前实现，整理目录不重置参数。

解释器选择依次为 `PIPER_PYTHON`、项目 `.venv/bin/python`、已有 `$HOME/anaconda3/envs/lerobot/bin/python`，最后为 PATH 中的 python3；启动脚本设置本仓库的 src 导入路径。原 `bash real_robot/test_mit.sh ...` 等命令仍兼容。详细使用说明见 [实机运行说明](docs/real_robot.md)。

## 摩擦分析

```bash
PYTHONPATH=src python -m piper_dynamics.pipeline.friction_combined --help
PYTHONPATH=src python -m piper_dynamics.pipeline.friction_bins --help
PYTHONPATH=src python -m piper_dynamics.pipeline.friction_final --help
```

三种分析保留原估计口径，通过 `--run-root`、`--runs-file`、`--archive`、`--revision`、`--source`、`--out` 明确选择数据和路径；默认正式输出为 `outputs/friction_fit/`。默认 run 索引对应当前机器既有的六次采集，仓库缺少这些本地记录时必须指定实际数据，不生成替代测量值。

## 激励轨迹审计

在项目已有Python环境中运行：

```bash
PYTHONPATH=src:. python -m piper_dynamics.pipeline.trajectory_audit --output outputs/trajectory_audit_20261105
```

只读取8条既有实机轨迹、冻结候选库与六轴摩擦记录，检查逐轴行程覆盖、规划/实测速率分布、
低速连续停留、共同54列基条件数与D/E信息、非线性Stribeck局部敏感度；不会运行新轨迹优化或打开CAN。
输出JSON、CSV、图表与报告；目录已有结果时拒绝覆盖。参数来源和假设随报告保存。
