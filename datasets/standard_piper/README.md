# 标准 PiPER 真机激励数据集

本目录汇总 2026-10-29/30 采集的 8 条完整六轴激励轨迹，供离线动力学辨识和算法复核使用。**全部数据来自标准 PiPER，不是 PiPER X。**

> 重要：当时的采集配置和部分 `test_manifest.json` 把机型写成 `piper_x`，后来核对铭牌和 J3/J4 构型后确认是普通 PiPER。因此不要再用 PiPER X 的 URDF/名义惯性解释这些数据；本目录统一使用 `model/piper.urdf`、`model/nominal_piper.json` 和 `model/piper.xml`。

## 轨迹

| 目录 | 角色 | 来源实验 | 来源子目录 | 约时长 | 周期 |
|---|---|---|---:|---:|---:|
| `old_fast_0` | 原轨迹，快档 0.5/0.5，seed 0 | EXP-20261029-001 | `data` | 120 s | 10 |
| `old_slow_0` | 原轨迹，慢档 0.5/0.25，seed 0 | EXP-20261029-002 | `data` | 240 s | 10 |
| `old_fast_1234` | 独立目标，快档 seed 1234 | EXP-20261029-001 | `validation/data/seed1234` | 120 s | 10 |
| `old_fast_1111` | 独立目标，快档 seed 1111 | EXP-20261029-001 | `validation/data/seed1111` | 120 s | 10 |
| `balanced_v1` | 修改激励 v1 | EXP-20261030-001 | `data` | 120 s | 10 |
| `balanced_v2` | 修改激励 v2 | EXP-20261030-002 | `data` | 240 s | 10 |
| `old_slow_1234` | 独立目标，慢档 seed 1234 | EXP-20261029-002 | `validation/data/seed1234` | 240 s | 10 |
| `old_slow_1111` | 独立目标，慢档 seed 1111 | EXP-20261029-002 | `validation/data/seed1111` | 240 s | 10 |

原始采集的 run 目录和完整 CAN 仍保留在本地实验目录，未放进本数据集；这里提供归档后的测量量，足以重建回归器并复算力矩拟合。

## 文件

每个轨迹目录包含：

- `raw.npz`：不做平滑；
- `sg21.npz`：21 点、3 阶、居中 Savitzky–Golay；
- `sg41.npz`：41 点、3 阶、居中 Savitzky–Golay；
- `metadata.json`：轨迹计划、采集质量、源文件哈希、限位审计。

每个 npz 的键：

| 键 | 形状 | 含义 |
|---|---|---|
| `t` | `(n,)` | 时间，s |
| `q` | `(n,6)` | 关节位置，rad |
| `qd` | `(n,6)` | 关节速度，rad/s |
| `qdd` | `(n,6)` | 由实测速度求导得到的加速度，rad/s² |
| `current_A` | `(n,6)` | 实测电流，A |
| `target_Nm` | `(n,6)` | 电流换算的候选力矩，N·m |
| `cycle` | `(n,)` | 整周期编号，1–10 |

归档数据在 100 Hz 处理链上选点后约为 **20 Hz**（`dt = 0.05 s`），不是完整 100 Hz 序列；每个 10 s/20 s 周期约 191/391 个点。

## 力矩换算

主结果使用普通 PiPER 官方参考系数：

```text
[4.725, 4.725, 4.725, 0.7787701152, 0.7787701152, 0.7787701152] N·m/A
```

`target_Nm = current_A * 系数`。**该系数没有针对这台机械臂做台架标定**，所以绝对力矩尺度和控制可用性都未验证。

## 训练/留出划分

沿用当前修订的整周期划分：

- 训练：偶数周期 `2,4,6,8,10`；
- 留出：奇数周期 `1,3,5,7,9`。

`sg41.npz` 上 120 s 数据为 955/957 点，240 s 数据为 1955/1956 点。`splits.json` 记录每条轨迹的训练/留出周期和样本数。

## 回归器

辨识模型为：

```text
tau = Y(q,qd,qdd) theta
theta: 60 个刚体惯性参数 + 18 个摩擦/偏置参数 = 78
```

- `Y` 是 60 列刚体回归器，形状 `(n,6,60)`；
- 摩擦/偏置另加 18 列：每轴 `qd`、`tanh(qd/0.02)`、`1`；
- 最小可辨识基为 54 维（36 维惯性 + 18 维摩擦/偏置），存在 24 维惯性零空间；
- 模型没有单独的关节阻尼、armature、转子惯量或 Stribeck 项；MuJoCo 的 `dof_damping`、`dof_armature`、`dof_frictionloss` 在构建回归器时置零。

`Y` 没有随数据一起存，因为它可由 `q/qd/qdd` 和模型重新精确生成，且体积远大于测量量。重建：

```bash
python build_regressor.py old_fast_0/sg41.npz
# 输出 old_fast_0/sg41_Y.npz，键 Y=(n,6,60)
```

`build_regressor.py` 使用本目录 `model/piper.xml` 和 `model/regressor.py`，不依赖机器人或 CAN。已用 `old_fast_0/sg41.npz` 核对：重建的 `Y` 与归档 `Y` 逐元素相同。

## 已知问题

- 采集时桌面/底座曾晃动，后半程操作者扶过桌子/底座，准确时间未知；模型假定基座固定。
- 电流到力矩系数未标定；关节绝对零位和方向未独立校准。
- 数据来自同一次 120 s/240 s session；留出周期不是独立采集。
- `link6` 合并了固定开度的夹爪；夹爪开度改变后模型失效。
- 这些参数是力矩预测基参数，不是逐连杆质量/COM/惯量真值，不能直接用于控制或当作 CAD 校准结果。

## 复算

仓库 `src/piper_dynamics/` 是 PiPER X 的参考实现；本数据集是标准 PiPER，不能直接套用 X 的 URDF。标准 PiPER 的 CAD-SVD 最小基复算脚本和结果在实验仓库 `identification/tools/urdf_base_recheck.py` 与 `identification/revisions/PIPER-20261102-001/urdf_base_recheck/`，本次不重复放进数据目录。

## 文件清单

- `manifest.json`：8 条轨迹、来源、样本数、文件哈希；
- `splits.json`：训练/留出周期；
- `model/`：`piper.urdf`、`nominal_piper.json`、`piper.xml`、`regressor.py`；
- `build_regressor.py`：从 compact npz 重建 60 列 `Y`。
