# inputs 目录说明

本项目只保留当前标准 PiPER 辨识流程需要的数据，按用途分成三个文件夹：

```
inputs/
├── raw/standard_piper/          # 原始输入数据（只读，不要改）
│   ├── manifest.json            # 8 条轨迹的来源、采样数和文件哈希
│   ├── splits.json              # 轨迹划分规则：整条 balanced_v2 训练、整条 balanced_v1 验证
│   └── <轨迹>/
│       ├── raw.npz              # 未平滑的原始测量
│       ├── sg21.npz             # 21 点 Savitzky-Golay 平滑
│       ├── sg41.npz             # 41 点 Savitzky-Golay 平滑（拟合使用）
│       └── metadata.json        # 激励计划、边界与运行信息
├── train/                       # 训练数据（由 raw 复制）
│   ├── manifest.json
│   └── balanced_v2_sg41.npz
└── validation/                  # 验证数据（由 raw 复制）
    ├── manifest.json
    └── balanced_v1_sg41.npz
```

## 数据内容

`npz` 文件包含 `t`、`q`、`qd`、`qdd`、`current_A`、`target_Nm` 和 `cycle`。
`raw`/`sg21`/`sg41` 是同一批测量的三种速度/加速度处理结果，按键名完全一致。

## 训练 / 验证划分

划分规则来自 `raw/standard_piper/splits.json`：

- 训练 = `balanced_v2`（整条，3911 样本，幅值 1.0，约 240 s）
- 验证 = `balanced_v1`（整条，1912 样本，幅值 0.8，约 120 s）

两者是同一台标准 PiPER 上独立采集的两组真机数据（实验 EXP-20261030-002 /
-001，CAN 时间戳相隔约 2.7 小时），因此这是跨采集的泛化验证，而不是同一段
激励内的奇偶周期插值。其余 6 条轨迹保留在 `raw/` 备用，不参与当前拟合/验证。

项目使用 `sg41`，因此 `train/` 与 `validation/` 只放 `sg41`。

重新生成（会清理旧的 `*_sg41.npz` 后重新复制）：

```bash
cd agx_piper_grasp_sim/dynamics_identification
PYTHONPATH=src ../.venv/bin/python -m piper_dynamics split
```

`train/manifest.json` 与 `validation/manifest.json` 记录源文件路径、源文件哈希、
周期列表和样本数，可用于校验副本是否与原始文件一致。

## 已清理的数据
