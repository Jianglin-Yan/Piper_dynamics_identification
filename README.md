# PiPER 动力学参数辨识

本项目以标准 PiPER 的真机激励数据和 `models/standard_piper/` 模型为当前入口。旧 PiPER X 复算流程、输入和结果保留为历史参考，两种机型的模型与结果不能互换。所有流程离线运行，不连接机械臂。

## 目录

| 路径 | 内容 |
| --- | --- |
| `inputs/standard_piper/` | 8 条标准 PiPER 轨迹、元数据、文件清单及整周期划分 |
| `models/standard_piper/` | 标准 PiPER URDF、名义惯性 JSON 和用于回归器的 MuJoCo XML |
| `src/piper_dynamics/standard_piper.py` | 标准 PiPER 回归矩阵重建与 CAD 尺度化 SVD 拟合入口 |
| `src/piper_dynamics/reference/regressor.py` | 唯一的 60 列刚体回归器实现，标准 PiPER 和旧流程共用 |
| `outputs/standard_piper/` | 标准 PiPER 的回归矩阵、拟合参数和评估报告 |
| `inputs/experiment_20261029/`, `inputs/splits/` | 旧实验原始记录和划分 |
| `models/piper_x.urdf`, `models/nominal_x.json` | 旧 PiPER X 复算所需模型 |
| `src/piper_dynamics/minimum_base_svd.py`, `physical_constraint_fit.py` | 旧实验的最小基和物理约束复算 |
| `outputs/minimum_base/`, `outputs/physical_constraints/`, `outputs/reference/` | 旧实验数值结果及保留的原 PDF |

旧输入的采集清单曾把机型标为 PiPER X，后来核对确认实际为标准 PiPER。旧流程及其保存的数值结果仍使用 PiPER X 模型，因此仅作历史对照；不要把这些结果当作标准 PiPER 的参数。`outputs/reference/piper_identification_report.pdf` 保留原文件字节。

## 标准 PiPER 数据

每条轨迹目录包含 `raw.npz`、`sg21.npz`、`sg41.npz` 和 `metadata.json`。前者不平滑；后两者使用 21 点或 41 点、三阶居中 Savitzky-Golay 处理。数据由 100 Hz 处理链抽样至约 20 Hz。`npz` 包含 `t`、六轴 `q`、`qd`、`qdd`、实测 `current_A`、候选力矩 `target_Nm` 和周期号 `cycle`。`manifest.json` 记录来源与哈希，`splits.json` 指定偶数周期训练、奇数周期留出。

| 轨迹 | 来源 | 时长档位 |
| --- | --- | --- |
| `old_fast_0`, `old_fast_1234`, `old_fast_1111` | 2026-10-29 原激励及独立目标 | 约 120 s |
| `old_slow_0`, `old_slow_1234`, `old_slow_1111` | 2026-10-29 原激励及独立目标 | 约 240 s |
| `balanced_v1`, `balanced_v2` | 2026-10-30 修改激励 | 约 120 s、240 s |

`target_Nm` 由电流按 `[4.725, 4.725, 4.725, 0.7787701152, 0.7787701152, 0.7787701152]` N·m/A 换算；这些系数没有针对本机做力矩标定。原完整 CAN 记录仍在实验目录，不在此压缩数据集中。

## 运行

在项目根目录使用 Python 3.10：

```bash
python3.10 -m venv .venv
.venv/bin/pip install -r requirements.txt
PYTHONPATH=src .venv/bin/python -m piper_dynamics.standard_piper regressor inputs/standard_piper/old_fast_0/sg41.npz
PYTHONPATH=src .venv/bin/python -m piper_dynamics.standard_piper fit inputs/standard_piper/old_fast_0/sg41.npz
```

`regressor` 在 `outputs/standard_piper/` 生成 `*_Y.npz`，其中 `Y` 为 `(N, 6, 60)` 的刚体回归矩阵。`fit` 在刚体矩阵后追加每轴 `qd`、`tanh(qd/0.02)`、常数三列，共 78 列；在偶数周期上用 CAD 尺度化截断 SVD 求解，未辨识方向取名义参数，并将 `*_fit.npz` 与同名 JSON 评估报告写入 `outputs/standard_piper/`。该秩来自所选训练数据，并非独立验证的结构最小基。两个命令都支持 `--out`，默认数据为 `old_fast_0/sg41.npz`。

标准 PiPER 模型中的关节阻尼、转子惯量、摩擦损失、重力补偿和接触在构造刚体回归器时关闭。名义参数来自官方模型，不是真机质量或惯量真值。截断 SVD 拟合的 78 维结果是一组力矩预测候选，不是唯一可辨识的逐连杆物理参数，也未施加旧 PiPER X 流程的几何与伪惯量约束。

## 历史复算

旧 PiPER X 代码和输出依赖固定源码快照及模型哈希。需要复算旧 PDF 数值时，按顺序运行：

```bash
PYTHONPATH=src .venv/bin/python -m piper_dynamics.minimum_base_svd
PYTHONPATH=src .venv/bin/python -m piper_dynamics.physical_constraint_fit
PYTHONPATH=src .venv/bin/python -m piper_dynamics.robust_estimator_comparison
```

旧结果的结构秩为 54/78；最小基 OLS 训练/留出/全量残差范数比分别为 7.0587% / 7.0764% / 7.3325%。旧 PDF 和 JSON 数值均保留，但模型机型与已核实的标准 PiPER 不符，不应用于控制或当作 CAD 校准结果。
