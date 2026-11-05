# PiPER 动力学参数辨识

本项目以标准 PiPER 的真机激励数据和 `models/standard_piper/` 模型为当前入口。旧 PiPER X 复算流程、输入和结果保留为历史参考，两种机型的模型与结果不能互换。所有流程离线运行，不连接机械臂。

## 目录

| 路径 | 内容 |
| --- | --- |
| `inputs/raw/standard_piper/` | 原始输入数据：8 条标准 PiPER 真机轨迹的 `raw`/`sg21`/`sg41`、元数据、文件清单及轨迹划分规则 |
| `inputs/train/` | 训练数据：`splits.json` 指定的整条 `balanced_v2` 的 `sg41`，附 `manifest.json` |
| `inputs/validation/` | 验证数据：`splits.json` 指定的整条 `balanced_v1` 的 `sg41`，附 `manifest.json` |
| `models/standard_piper/` | 标准 PiPER URDF、名义惯性 JSON 和用于回归器的 MuJoCo XML |
| `src/piper_dynamics/` | 源码包，职责划分见下方“代码结构” |
| `outputs/standard_piper/` | 标准 PiPER 的回归矩阵、拟合参数（`.npz` / `_parameters.csv` / `.md`）、评估报告 `.json` 和力矩对比图 `_torque_*.png` |
| `models/piper_x.urdf`, `models/nominal_x.json` | 旧 PiPER X 复算所需模型 |
| `src/piper_dynamics/legacy/` | 旧 PiPER X 复算代码（原始数据已清理，仅保留代码供追溯） |
| `outputs/minimum_base/`, `outputs/physical_constraints/`, `outputs/reference/` | 旧实验数值结果及保留的原 PDF |

旧输入的采集清单曾把机型标为 PiPER X，后来核对确认实际为标准 PiPER。旧流程及其保存的数值结果仍使用 PiPER X 模型，因此仅作历史对照；不要把这些结果当作标准 PiPER 的参数。`outputs/reference/piper_identification_report.pdf` 保留原文件字节。旧复算所需的 222 MB 原始 CAN 记录（原 `inputs/experiment_20261029/`）与旧划分（原 `inputs/splits/`）已从 `inputs/` 删除，避免与当前标准 PiPER 数据混淆；需要复算时从 git 历史恢复。

## 代码结构

```
src/piper_dynamics/
├── __main__.py / cli.py            # 命令行入口：split / regressor / fit / show
├── paths.py                        # 仓库路径（inputs、models、outputs）
├── core/                           # 可复用底层模块
│   ├── dataset_io.py               # 读取并校验测量 npz
│   ├── rigid_body_regressor.py     # 60 列刚体回归器（mj_inverse 有限差分）
│   ├── regression_model.py         # 摩擦特征、参数缩放、误差指标
│   ├── svd_fit.py                  # 缩放截断 SVD 最小二乘与力矩预测
│   ├── parameter_schema.py         # 78 维参数名、分组与单位
│   └── parameter_report.py         # 参数表、CSV、Markdown 报告与物理检查
├── pipeline/                       # 标准 PiPER 业务流程
│   ├── trajectory_split.py         # 整条轨迹训练/验证划分与物化
│   └── standard_piper_pipeline.py  # 端到端拟合编排与输出落盘
├── utils/                          # 通用工具
│   ├── hashing.py                  # 文件 SHA256
│   ├── validation.py               # 通用数值向量校验
│   └── plotting.py                 # 六关节实测/预测力矩对比图
├── zero_force_drag/                # 最终辨识参数的 MIT 重力补偿拖动候选控制器
└── legacy/                         # 旧 PiPER X 代码（仅存档，当前不可运行）
```

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

## 运行

在项目根目录使用 Python 3.10：

```bash
python3.10 -m venv .venv
.venv/bin/pip install -r requirements.txt
PYTHONPATH=src .venv/bin/python -m piper_dynamics split
PYTHONPATH=src .venv/bin/python -m piper_dynamics fit
PYTHONPATH=src .venv/bin/python -m piper_dynamics show
```

`fit` 会写出四份数据/报告文件：`*_fit.npz`（原始数组）、`*_fit.json`（指标与参数）、`*_fit_parameters.csv`（逐项参数表）和 `*_fit.md`（可直接阅读的中文报告，含分关节误差与物理一致性检查），并顺手画两张一页六行竖排（共享时间轴）的力矩对比图：`*_torque_train.png` 和 `*_torque_validation.png`（黑线实测、红线预测，纵轴标注各关节 RMSE）。`show` 不需要重新拟合，直接读取 npz 并打印连杆参数与摩擦项；`fit --no-plots` 可跳过画图（无 matplotlib 环境时）。

`regressor` 在 `outputs/standard_piper/` 生成 `*_Y.npz`，其中 `Y` 为 `(N, 6, 60)` 的刚体回归矩阵。`fit` 在刚体矩阵后追加每轴 `qd`、`tanh(qd/0.02)`、常数三列，共 78 列；在整条训练轨迹上用 CAD 尺度化截断 SVD 求解，未辨识方向取名义参数，再在整条验证轨迹上评估，将 `*_fit.npz` 与同名 JSON 报告写入 `outputs/standard_piper/`。该秩来自所选训练数据，并非独立验证的结构最小基。当前 `balanced_v2 -> balanced_v1` 的秩为 54/78，训练误差 5.84%，跨轨迹验证误差 8.91%。两个命令都支持 `--out`；`fit` 还支持 `--validation` 指定其他留出文件。

标准 PiPER 模型中的关节阻尼、转子惯量、摩擦损失、重力补偿和接触在构造刚体回归器时关闭。名义参数来自官方模型，不是真机质量或惯量真值。截断 SVD 拟合的 78 维结果是一组力矩预测候选，不是唯一可辨识的逐连杆物理参数，也未施加旧 PiPER X 流程的几何与伪惯量约束。

## 最小参数集

标准 PiPER 的 78 列回归器结构秩为 **54/78**（6000 组随机状态，1e-6～1e-10 阈值下稳定），其中纯刚体部分为 36/60；本次训练数据的数值秩同样是 54。因此能独立影响力矩的参数组合只有 54 个，另外 24 个方向属于零空间。

需要区分三件事：

- “最小参数集”指 **54 个相互独立的参数组合**，不是 54 个单独的物理量，且基底不唯一。
- `*_fit.npz` 保存的仍是 78 维 `theta`，零空间方向被钉在名义值；真正由数据确定的只有 54 个自由度。
- 当前的 54 维基底来自训练矩阵的 SVD，是数据相关的；若需要与数据无关的结构最小基，应对通用状态堆叠的回归矩阵做列主元 QR。

## 伺服控制逻辑

### MIT 重力补偿拖动

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
PYTHONPATH=src ../.venv/bin/python -m piper_dynamics.zero_force_drag --q 0 1 -1 0 0 0
```

真机运行需额外安装官方 `piper_sdk`，确认固件版本及 MIT 力矩方向和比例，按
[`zero_force_drag/README.md`](src/piper_dynamics/zero_force_drag/README.md) 创建六关节配置文件，
并支撑机械臂后运行：

```bash
PYTHONPATH=src ../.venv/bin/python -m piper_dynamics.zero_force_drag \
  --live --config path/to/verified_drag.json --can can0 --protocol v3 \
  --acknowledge-uncalibrated-torque
```

`--protocol` 必须与固件匹配；V2 使用 `v2`。参数含义、配置示例和退出行为见模块
README。真机模式未经过硬件验证。

PiPER 单关节的三环控制（位置 → 速度 → 电流/FOC）与 MIT 力矩前馈结构见
`docs/piper_servo_control_logic.png`（SVG 同名），由
`scripts/render_control_diagram.py` 生成：

```bash
PYTHONPATH=src .venv/bin/python scripts/render_control_diagram.py
```

## 历史复算

旧 PiPER X 代码和输出依赖固定源码快照及模型哈希。需要复算旧 PDF 数值时，按顺序运行：

```bash
PYTHONPATH=src .venv/bin/python -m piper_dynamics.legacy.minimum_base_svd
PYTHONPATH=src .venv/bin/python -m piper_dynamics.legacy.physical_constraint_fit
PYTHONPATH=src .venv/bin/python -m piper_dynamics.legacy.robust_estimator_comparison
```

旧结果的结构秩为 54/78；最小基 OLS 训练/留出/全量残差范数比分别为 7.0587% / 7.0764% / 7.3325%。旧 PDF 和 JSON 数值均保留，但模型机型与已核实的标准 PiPER 不符，不应用于控制或当作 CAD 校准结果。
