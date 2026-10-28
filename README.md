# 机械臂动力学参数辨识（Piper / MuJoCo）

对 `agx_piper_grasp_sim` 里 6 自由度 Piper 机械臂做**刚体动力学参数辨识**：
不用真机、不用标定板，只靠仿真器输出的关节量测，恢复每个连杆的
**质量、质心位置、惯性张量**，并给出独立的验证结果。

> **依赖说明：本项目不是独立仿真器，必须配合仿真器项目运行。**
> 本辨识代码依赖 `/home/yjl-10096528/EmbodiedAI/deepseek_grasp/agx_piper_grasp_sim`
> 中的 `agx_piper_sim` Python 包、MuJoCo 场景 `assets/scene.xml` 和该项目的 Python 环境。
> 请将本仓库的文件放在该仿真器目录的 `dynamics_identification/` 子目录中；不要把它当作
> 单独的仓库根目录执行。若仿真器路径不同，需要同步修改脚本中的导入/场景路径。

```
excitation.py  →  data_collector.py  →  regressor.py  →  identify.py  →  validate.py
  激励轨迹          仿真采集           精确回归器        最小二乘         泛化验证
```

---

## 1. 快速开始

```bash
cd /home/yjl-10096528/EmbodiedAI/deepseek_grasp/agx_piper_grasp_sim

# 1) 辨识（默认 60 s 轨迹，oracle 力矩源）
.venv/bin/python -m dynamics_identification.identify

# 2) 在一条没见过的轨迹上验证
.venv/bin/python -m dynamics_identification.validate

# 3) 只验证、不重新辨识（回归器自检）
.venv/bin/python -m dynamics_identification.regressor
```

全部耗时约 20 s（60 s 轨迹 + 6001 个样本的回归器构造）。

| 命令 | 输出 |
|---|---|
| `identify` | `results/identified_params.npz` + `.json` |
| `validate` | `results/validation.json` |
| `regressor` | 自检结果打印到 stdout，退出码 0/1 |

### 常用参数

| 参数 | 含义 |
|---|---|
| `--duration` | 激励轨迹长度（默认 60 s） |
| `--stride` | 每隔 N 个样本取一个做回归（默认 5，即 100 Hz） |
| `--source oracle\|servo` | 力矩来源，见 §3 |
| `--ridge` | 相对 Tikhonov 权重，默认 `1e-8`；`0` = 纯最小二乘 |
| `--data PATH` | 缓存原始数据，重跑时不必重新仿真 |

---

## 2. 方法

### 2.1 参数化：为什么是 `I_origin` 而不是 `I_com`

刚体动力学对惯性参数是**线性**的，但只有在正确选择参数时才成立。
每个连杆取 10 个参数，全部表达在**关节轴所在的连杆坐标系**里：

$$\theta_i = [\,m,\; h_x, h_y, h_z,\; I_{xx}, I_{yy}, I_{zz}, I_{xy}, I_{xz}, I_{yz}\,]$$

- `m`：质量
- `h = m·c`：一阶矩（`c` 为质心）
- `I`：**相对坐标系原点**的转动惯量

这一点是本项目最关键的坑。牛顿-欧拉反向递推

$$f = m a + \alpha \times h + \omega \times (\omega \times h)$$
$$n = I_{\text{origin}}\,\alpha + \omega \times (I_{\text{origin}}\,\omega) + h \times a$$

对 `(m, h, I_origin)` 线性；而绕质心的惯量
`I_com = I_origin − m(c·c·1 − c cᵀ)` 对 `(m, h)` 是**非线性**的，
用 `I_com` 做参数向量根本不存在常数回归器。

实用参数（质量、质心、绕质心惯量）由 `to_com_params()` 事后换算。

### 2.2 回归器：直接探测 MuJoCo 的逆动力学

`mj_inverse` 本身就是递归牛顿-欧拉实现，因此对上述参数也线性。于是

$$\frac{\partial \tau}{\partial \theta_k} = \frac{\tau(\text{carrier} + \epsilon e_k) - \tau(\text{carrier})}{\epsilon}$$

就是 `Y` 的一列，**对任意 `ε` 都精确**。实现要点：

- 探测前给所有臂部 body 装一个固定的 “carrier” 惯量（`m=0.05, I=100`）。
  carrier 在基线里相减掉，对结果没有影响；它的作用是
  ① 避免零质量模型让约束分解奇异；② 保证 `carrier + εe_k` 始终是
  MuJoCo 能表示的**正定**质心惯量（一阶矩探测会用平行轴定理从主惯量里
  减掉最多 `h·h/m`）。
- 探测期间关闭接触（辨识数据本来就在无接触工况下采集）。
- 手指的等约束只作用在 `joint7/joint8`，对 1–6 号 DOF 的广义力贡献为零。

自检：`Y @ θ_gt` 与真实模型的 `mj_inverse` 吻合到 **1e-11 N·m（相对 7e-13）**。

> 早期版本用解析牛顿-欧拉手写 `Y`，关节坐标系里有 180° 翻转，
> 硬编码 `[0,0,1]` 轴导致 joint3/5 符号相反、joint2 差 20%。
> 改成探测 `mj_inverse` 之后，轴向/符号问题从结构上消失——
> 运动学交给 MuJoCo 自己算。

### 2.3 激励轨迹

Swevers 多正弦，每关节 5 阶谐波、基频 0.2 Hz：

$$q_j(t) = q_{0j} + \sum_{k=1}^{5}\big[ a_{jk}\sin(k\omega t) + b_{jk}\cos(k\omega t)\big]$$

系数随机但归一化到 `Σ(|a|+|b|) ≤ 0.6 ×` 半行程，因此**最坏情况也不越限**
（实测 `clipping: False`）。速度、加速度解析求导，不需要差分。

### 2.4 采集：两个必须注意的地方

1. **必须关掉重力补偿**。仿真器默认在 `step()` 里把 `qfrc_bias`
   当 `qfrc_applied` 加进去，正好抵消掉要辨识的惯性力。
   新增的 `Simulation(gravity_compensation=False)` 关掉它。
2. **`qdd` 用有限差分**。积分器是 `implicitfast`，`data.qacc` 是隐式
   求解前的显式加速度，不是真实的 `d(q̇)/dt`（关节 4/6 上差两个数量级：
   795/1165 vs 3.2/7.3）。采集端用 `np.gradient(qd, dt)`。

采集中还会把桌面物体挪到 `z = −5`，避免激励轨迹扫过桌面时产生碰撞冲量。

### 2.5 求解：基参数 + QR

60 个参数**不可全辨识**——关节力矩只约束基参数空间，其余方向落在
堆叠回归器的零空间里。用列主元 QR（先按列范数归一化，消除量纲差异）
选出相互独立的列：

- 实测 **rank = 47 / 60**。
- `--ridge 0`：只解 47 个基参数组合，其余留 0。
  此时**不能**逐元素和真值比——QR 选出的是**线性组合**，不是真值的子集。
- `--ridge 1e-8`（默认）：给全部 60 个参数加一个极小的 Tikhonov 项，
  把零空间方向钉在 CAD 名义值上。可观测方向几乎不受影响，
  但可以得到一组物理上完整、可写回 MuJoCo 的参数。

---

## 3. 两种力矩源

| | `--source oracle`（默认） | `--source servo` |
|---|---|---|
| τ | 用测到的 `(q, q̇, q̈)` 跑 `mj_inverse` | 采集时记下的伺服广义力 `qfrc_actuator` |
| 含义 | 精确的 `M q̈ + C q̇ + g` | 真实实验里唯一能测到的量 |
| 用途 | **验证整条链路**（激励→回归器→求解），必须精确复现 | **真实辨识**，带有限差分误差 |

`servo` 的固有噪声来自 §2.4 的 `qdd` 差分：实测
`M·q̈_fd + bias` 与 `qfrc_actuator` 差 **7.6%**，这是该方法的下界。

---

## 4. 结果（本机实测）

### 4.1 `oracle` —— 链路正确性

```
regressor rank: 47 / 60
torque fit: rel err 1.262e-10 %
per-joint RMSE [N*m]: [0, 0, 0, 0, 0, 0]
total arm mass: 3.6896 kg (ground truth 3.6896, err -0.00%)
```

验证（30 s 全新轨迹，seed 1234 ≠ 0）：

```
joint     rms[N*m]  rmse[N*m]      rmse_mj
joint1      1.8519     0.0000       0.0000
joint2      7.2706     0.0000       0.0000
joint3      4.6794     0.0000       0.0000
joint4      0.0746     0.0000       0.0000
joint5      0.8214     0.0000       0.0000
joint6      0.0054     0.0000       0.0000
result: PASS
```

`rmse_mj` 是把辨识出的参数**写回 MuJoCo 模型**后重新跑 `mj_inverse` 的误差，
为 0 说明参数不只是在代数上自洽，物理引擎也能直接使用。

### 4.2 `servo` —— 真实辨识精度

| link | m_hat | m_gt | 误差 | \|ΔCOM\| |
|---|---|---|---|---|
| link1 | 0.7100 | 0.7100 | 0.00% | 0.00 mm |
| link2 | 1.1600 | 1.1600 | 0.00% | 7.20 mm |
| link3 | 0.5074 | 0.5000 | +1.49% | 101.34 mm |
| link4 | 0.3856 | 0.3800 | +1.48% | 74.30 mm |
| link5 | 0.3886 | 0.3830 | +1.47% | 35.66 mm |
| link6 | 0.5622 | 0.5566 | +1.01% | 24.23 mm |

- 总质量 3.7139 kg vs 3.6896 kg（**+0.66%**）
- 辨识扭矩拟合：**2.95%**
- 未见轨迹验证：**5.56%**，逐关节 RMSE 0.12–0.29 N·m，`PASS`

**怎么读这张表**：

- 质量误差约 1.5%，方向一致（都偏大），来源是 §3 的有限差分噪声。
- `|ΔCOM|` 在 link3/4 上到 7–10 cm，看起来很大，但质心只通过
  `h = m·c` 进入方程，未辨识的连杆上 `h` 的可观测性本来就差；
  这几个方向的偏差被 `--ridge` 的名义值约束住，对**力矩预测没有影响**
  （见验证 RMSE）。
- 评判辨识质量请以**扭矩预测误差**为准，不要看单个物理量的偏差。

---

## 5. 验证判据

`validate.py` 对每个关节判断

$$\text{rmse}_j \le \max(\texttt{abs-tol},\; \texttt{tol}\cdot \text{rms}(\tau_j))$$

默认 `tol = 0.10`、`abs-tol = 0.30 N·m`。绝对下限是必要的：
腕部关节（joint4/6）的力矩本身只有 0.07 / 0.35 N·m 量级，
纯相对判据会因为分母太小而误判。

---

## 6. 文件说明

| 文件 | 作用 |
|---|---|
| `excitation.py` | Swevers 多正弦激励轨迹（解析导数，不越限） |
| `data_collector.py` | 驱动仿真、记录 `q/q̇/q̈/τ/M`，`q̈` 用有限差分 |
| `regressor.py` | 精确回归器 `Y`（探测 `mj_inverse`）+ 真值 + 自检 |
| `ground_truth.py` | 从 MuJoCo 模型读真值（另一种独立实现，可交叉核对） |
| `identify.py` | 采集 → 构 `Y` → QR 选基 → 求解 → 对比 → 存盘 |
| `validate.py` | 未见轨迹上的扭矩预测 + 写回 MuJoCo 复核 |
| `AGENT_HANDOFF.md` | 上一轮交接记录（历史文档，已完成） |

### 仿真器侧新增接口

辨识需要但原来取不到的 5 个量，已补进
`agx_piper_sim/sim/simulation.py`：

| 接口 | 返回 | 说明 |
|---|---|---|
| `joint_accelerations()` | `q̈` | `data.qacc`（注意 implicitfast 语义） |
| `joint_torques()` | `τ` | `data.qfrc_actuator`，广义力矩而非原始控制量 |
| `gravity_torques()` | `C q̇ + g` | `data.qfrc_bias` |
| `mass_matrix()` | `6×6` | `mj_fullM` 后取臂部子块（兼容 3.16 签名变更） |
| `inverse_dynamics(qacc=None)` | `τ` | 在临时 `MjData` 上跑 `mj_inverse`，不扰动仿真 |

外加构造参数 `Simulation(gravity_compensation=True|False)`。

---

## 7. 已知限制

- 只辨识 **臂部 6 个关节**。夹爪（`joint7/8`）行程小、惯量比腕部还小一个
  量级，单独激励它没有意义；辨识时夹爪固定在开合度 `--gripper-open`
  并把 `link6` 之后 5 个 body 合并进 link6。
- 模型里没有关节阻尼/摩擦项，所以也没有辨识它。真机上必须加
  `friction` / `damping` 回归列，否则低频轨迹上会系统性偏差。
- `servo` 路径的精度被 `qdd` 差分限制在 ~5%。要再提高，需要
  低通滤波 + 一致的滤波相位，或者记录加速度传感器。
- 没有画图（环境里没装 matplotlib）。`validate.py` 只输出数值与 JSON。
