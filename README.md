# PiPER 动力学辨识与控制实验

项目只支持普通 PiPER。离线辨识、实机运行、仿真与分析共用 `src/piper_dynamics`，仓库顶层沿用 main 的输入、模型、输出管理方式。

使用入口见 [实机运行说明](docs/real_robot.md)，源码迁移与验证范围见 [验收报告](docs/refactor_report.md)。

## 目录

下表为常用位置与目录职责索引。

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
| 根目录 `real_robot/` | 外部归档仍依赖的少量接口、`pilot.sh`及配置/模型别名；业务实现在src |

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

解释器选择依次为 `PIPER_PYTHON`、项目 `.venv/bin/python`、已有 `$HOME/anaconda3/envs/lerobot/bin/python`，最后为 PATH 中的 python3；启动脚本设置本仓库的 src 导入路径。旧 `real_robot/test_*.sh`、`tools/fit_*.py`及根目录`excitation.py`已移除，使用上面的`scripts`和pipeline入口；`real_robot/pilot.sh`为外部归档命令暂留。详细使用说明见 [实机运行说明](docs/real_robot.md)。

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


## 标准 PiPER 数据

每条轨迹目录包含 `raw.npz`、`sg21.npz`、`sg41.npz` 和 `metadata.json`。前者不平滑；后两者使用 21 点或 41 点、三阶居中 Savitzky-Golay 处理。数据由 100 Hz 处理链抽样至约 20 Hz。`npz` 包含 `t`、六轴 `q`、`qd`、`qdd`、实测 `current_A`、候选力矩 `target_Nm` 和周期号 `cycle`。`manifest.json` 记录来源与哈希。

## 训练 / 验证划分

`splits.json` 采用**整条轨迹划分**：`balanced_v2` 整条用于拟合，`balanced_v1` 整条只用于留出验证。两者是同一台标准 PiPER 上独立采集的两组真机数据（实验 EXP-20261030-002 与 EXP-20261030-001，CAN 时间戳相隔约 2.7 小时），激励幅值分别为 1.0 和 0.8。

不使用同一条轨迹的奇数/偶数周期划分，是因为同一段激励内的划分只能检验插值，无法检验“换一组真机数据”后的泛化；跨轨迹划分更严格，也更接近实际部署。

`inputs/train/` 与 `inputs/validation/` 是 `splits.json` 在 `sg41` 上的物化副本，拟合只读训练文件，验证文件不参与求解。它们由下面命令生成，不要手工修改；其余 6 条轨迹保留在 `raw/` 作为备用数据，不参与当前拟合与验证：

```bash
PYTHONPATH=src .venv/bin/python -m piper_dynamics split
```

| 轨迹 | 来源 | 时长档位 |
| --- | --- | --- |
| `old_fast_0`, `old_fast_1234`, `old_fast_1111` | 2026-10-29 原激励及独立目标 | 约 120 s |
| `old_slow_0`, `old_slow_1234`, `old_slow_1111` | 2026-10-29 原激励及独立目标 | 约 240 s |
| `balanced_v1`, `balanced_v2` | 2026-10-30 修改激励 | 约 120 s、240 s |

`target_Nm` 由电流按 `[4.725, 4.725, 4.725, 0.7787701152, 0.7787701152, 0.7787701152]` N·m/A 换算；这些系数没有针对本机做力矩标定。

## 最小参数集

标准 PiPER 的 78 列回归器结构秩为 **54/78**（6000 组随机状态，1e-6～1e-10 阈值下稳定），其中纯刚体部分为 36/60；本次训练数据的数值秩同样是 54。因此能独立影响力矩的参数组合只有 54 个，另外 24 个方向属于零空间。

需要区分三件事：

- “最小参数集”指 **54 个相互独立的参数组合**，不是 54 个单独的物理量，且基底不唯一。
- `*_fit.npz` 保存的仍是 78 维 `theta`，零空间方向被钉在名义值；真正由数据确定的只有 54 个自由度。
- 当前的 54 维基底来自训练矩阵的 SVD，是数据相关的；若需要与数据无关的结构最小基，应对通用状态堆叠的回归矩阵做列主元 QR。

## 辨识模型的独立拖动候选

`src/piper_dynamics/zero_force_drag/` 使用当前标准 PiPER 的最终辨识文件
`outputs/standard_piper/balanced_v2_sg41_fit.npz`。每周期读取六关节位置和速度，
计算静止姿态的刚体力矩 `Y(q, 0, 0) @ theta[:60]`，乘逐关节补偿比例后作为 MIT
`t_ref` 下发。MIT 位置目标跟随实测位置，目标速度为零，逐关节 `kp` / `kd` 从配置
读取。模块检查反馈时效、关节位置与速度、力矩上限及机械臂故障状态。

这实现的是**重力补偿的低阻尼拖动候选控制**，不是力传感器闭环的零力控制；
没有加入摩擦、惯性或科氏力补偿。当前辨识力矩未经本机标定，真机上的力矩方向、
比例和拖动效果尚未验证。普通人工拖动记录轨迹可先评估 PiPER 自带拖动示教模式。

在本目录离线查看某姿态的候选重力力矩（弧度输入，不连接机械臂）：

```bash
PYTHONPATH=src python -m piper_dynamics.zero_force_drag --q 0 1 -1 0 0 0
```

真机运行需额外安装官方 `piper_sdk`，确认固件版本及 MIT 力矩方向和比例，按
[`zero_force_drag/README.md`](src/piper_dynamics/zero_force_drag/README.md) 创建六关节配置文件，
并支撑机械臂后运行：

```bash
PYTHONPATH=src python -m piper_dynamics.zero_force_drag \
  --live --config path/to/verified_drag.json --can can0 --protocol v3 \
  --acknowledge-uncalibrated-torque
```

`--protocol` 必须与固件匹配；V2 使用 `v2`。参数含义、配置示例和退出行为见模块
README。真机模式未经过硬件验证。

此独立候选与`scripts/test_drag.sh`的实机采集阶段机分别保留。实际采集入口使用前述脚本；该候选的离线计算通过不代表其真机模式已验证。

## 原始拖动实验记录

2026-11-05的7组实机拖动记录位于[data/zero_force_drag/20261105](data/zero_force_drag/20261105/README.md)，保留原始CAN、发送命令、配置、结束标签和校验清单。该批记录不自动改变inputs中的训练/验证划分。
