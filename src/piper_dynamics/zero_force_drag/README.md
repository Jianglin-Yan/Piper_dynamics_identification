# 标准 PiPER 零力拖动候选控制器

本目录将最终辨识文件 `outputs/standard_piper/balanced_v2_sg41_fit.npz` 的**刚体参数**用于重力补偿，并通过 PiPER MIT 命令的 `t_ref` 发送补偿力矩。位置目标每周期跟随反馈位置，速度目标为零，`kp` 和 `kd` 由配置逐关节指定。这里的“零力”是尽量减小手动拖动阻力，并非力传感器测得零力；减速器摩擦、通信延迟和未标定力矩都会留下阻力或造成漂移。

## 代码结构

- `model.py`：读取最终拟合、检查模型哈希和参数顺序，复用现有 MuJoCo 回归器计算 `Y(q,0,0) @ theta[:60]`。拟合的单个惯量不一定物理有效，不将其写回 MuJoCo 模型。
- `controller.py`：与硬件无关的逐关节控制计算和限幅检查。
- `piper_io.py`：官方 SDK V2/V3 的反馈读取与 MIT 发送适配层。
- `__main__.py`：离线计算和显式开启的真机循环。

辨识摩擦项在零速附近包含库仑项和偏置，可能主动推动关节，因此当前只使用**最终拟合文件中的刚体重力部分**。本目录没有复制现有回归器、模型加载、参数命名、路径或哈希实现。

## 离线检查

在 `dynamics_identification` 根目录运行：

```bash
PYTHONPATH=src ../.venv/bin/python -m piper_dynamics.zero_force_drag --q 0 1 -1 0 0 0
```

结果单位为弧度和 N·m。此命令不连接 CAN。

## 真机配置与运行

真机需在同一 Python 环境单独安装官方 `piper_sdk`（例如 `../.venv/bin/pip install piper_sdk`），激活 CAN 设备，确认固件与所选 `--protocol v2|v3` 匹配（V3 的 MOVE M 模式值为 `0x06`，V2 为 `0x04`），并在**机械臂有可靠支撑、力矩方向和比例已核实**的条件下提供六关节配置文件，例如：

```json
{
  "kp": [0, 0, 0, 0, 0, 0],
  "kd": [0.8, 0.8, 0.8, 0.8, 0.8, 0.8],
  "torque_limit_nm": [2, 2, 2, 1, 1, 1],
  "compensation_scale": [0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
  "max_speed_rad_s": [0.2, 0.2, 0.2, 0.2, 0.2, 0.2],
  "max_feedback_age_s": 0.1
}
```

以上仅展示**配置格式**，不是已验证的真机参数；若计算力矩超过配置限额，程序会停止，不会静默截断。验证配置后运行：

```bash
PYTHONPATH=src ../.venv/bin/python -m piper_dynamics.zero_force_drag \
  --live --config path/to/verified_drag.json --can can0 --protocol v3 \
  --acknowledge-uncalibrated-torque
```

当前拟合报告标记 `torque_calibrated=false`。确认实际关节力矩与 MIT `t_ref` 的比例、符号、饱和行为之前，**不能把拟合预测当成已标定力矩**。程序要求显式确认这点，并且检查反馈是否更新、速度是否超限、模型力矩是否超限、机械臂是否报告故障。退出或故障时发送 SDK 急停命令；急停可能使机械臂失去支撑力，因此真机试验必须预先支撑机械臂。仅凭软件无法保证外部接触、意外断电或 CAN 断线时的机械安全。

普通 PiPER 拖动示教已有官方固件模式；如果只需人工拖动记录轨迹，应优先评估该模式。这个模块用于研究自定义 MIT 重力补偿，并未在真机上验证。
