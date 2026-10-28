# 给 VS Code Chat 的交接说明（人类写的，不要重写已验证的模块）

> **状态：已完成。** `regressor.py` 的符号/轴向问题通过「用 `mj_inverse` 数值探测」
> 从结构上消除（不再硬编码关节轴），自检 `Y·θ_gt` vs `mj_inverse` 达 1e-11 N·m。
> `identify.py` / `validate.py` / `README.md` 已补齐。
> 当前状态与实测结果见 `README.md`；本文件保留为历史记录。


> 这份文件是 DSH 助手在交接时写的。**先读完再动手**，能省掉大量重复工作。

## 环境事实（已验证）

- MuJoCo **3.16.0**、numpy 2.2.6，解释器用仓库里的 `.venv`
- 模型积分器是 **implicitfast**（`model.opt.integrator == 3`）
  → **`data.qacc` 不是真实的 d(qd)/dt**（关节 4/6 中位数 795/1165 vs 有限差分 3.2/7.3）
  → 采集端已经改成 `qdd = np.gradient(qd, dt)`，别再改回去
- 位置伺服很硬、腕部惯量 ~1e-4 kg·m²，所以 `gravity_compensation=False` 时跟踪 RMSE 0.04–0.13 rad 属正常

## 文件状态

| 文件 | 状态 | 说明 |
|---|---|---|
| `__init__.py` | ✅ 可用 | 包标记 |
| `excitation.py` | ✅ 已验证 | Swevers 多正弦，解析求导，60 s / 500 Hz / 5 阶 / 幅值 60% 行程，不越限 |
| `data_collector.py` | ✅ 已验证 | 1 s 冒烟通过；`gravity_compensation=False`；物体移到 z=-5 避免碰撞；**qdd 用有限差分** |
| `ground_truth.py` | ✅ 已验证 | 与世界坐标手算重力矩逐项一致（3.051 / 4.910 / 0.029 / 0.917 N·m）；link6 已合并 flange+gripper_base+双指 |
| `regressor.py` | ⚠️ **未通过自检** | `Y·θ` vs `mj_inverse` 不吻合，见下 |

## regressor 已定位到的信息（别从零开始）

1. 子连杆的力矩臂要在**子坐标系**里做叉乘再旋回父系 —— 已修
2. `n` 里漏了 `h × a`（一阶矩相对原点的力矩）—— 已补
3. 补完后用 `-h×a` 时 **joint3/joint5 幅值对上了**（4.89 vs 4.918、0.931 vs 0.919）但**符号相反**，
   joint2 差约 20%（-2.43 vs -3.06）
4. **最可疑**：link 坐标系的 z 方向交替（`body_quat` 里有 180° 旋转）。
   建议改用 `data.xaxis[jnt_id]` 取每个关节的世界轴向，而不是硬编码 `[0,0,1]`
5. 校验方法：关掉 `ACTUATION | DAMPER | CONTACT | CONSTRAINT`，`mj_forward` **之后**再写 `qacc`，
   然后 `mj_inverse` 取 `qfrc_inverse`，与 `Y·θ_gt` 比

## 还没做的

Step 11 `identify.py`（`θ_id = lstsq(Y, τ)`，注意 stacking 方式：
按样本行堆叠 `[τ_0(0..5); τ_1(0..5); …]`，`Y` 相应 reshape(-1, 60)）、
Step 12 `validate.py`、Step 13 `README.md`。

腕部参数可辨识性差，建议 QR 列主元选基参数，而不是直接 `lstsq` 全部 60 个。
