# PiPER 辨识与控制代码架构（简版）

## 1. 两个部分

| 目录 | 职责 |
|---|---|
| `identification/` | 离线辨识：激励数据、回归、拟合、验证、修订归档（`revisions/<修订号>/`） |
| `real_robot/` | 实机控制与仿真：CAN 通信、MIT/位置控制、试验任务、记录、安全 |

`real_robot/assets/` 放官方 URDF、名义惯量、MIT 官方驱动代码；`runs/` 是每次运行的原始数据（git 忽略）。

## 2. 控制主链路

```
config.json + 命令行
        │
        ▼
run.py 主循环（100 Hz）
        │  读反馈 / 写目标 / 监视 / 记录
        ▼
Controller（统一阶段机）
        │  当前阶段、参考轨迹、Kp/Kd/前馈、停止与回位
        ▼
后端：Hardware（CAN）或 Simulation（MuJoCo）
```

### 阶段机

```
WAITING → ARMING → BOUNDARY_ENTRY（仅 MOVE J 且起始位在限位外）
        → PREPARING → SETTLING_READY → READY
        → FF_RAMP_IN（MIT）→ TESTING / DRAG
        → FF_RAMP_OUT → BRAKING → RETURNING → SETTLING_HOME
        → RELEASING（仅实机 MIT）→ DISABLING → DONE
```

- `BOUNDARY_ENTRY`：MOVE J 不能直接指到限位外的停靠位，先以 2% 速度走到最近的合法边界；
- `FF_RAMP_IN/OUT`：MIT 下 Kp 与前馈同步渐变；
- `BRAKING`：收到停止后按当前参考速度制动；
- `RELEASING`：失能前把 Kp/Kd 平滑降到 0，避免突然掉落（仅实机）。

## 3. 任务（trial）

| 任务 | 文件 | 入口 | 说明 |
|---|---|---|---|
| 固定小轨迹 MIT | `mit_trial.py` | `test_mit.sh` | PD + 重力前馈，验证辨识模型 |
| 静态/正反/负载 | `validation_trials.py` | `test_static.sh` 等 | 位置控制诊断数据 |
| 低速摩擦 | `friction.py` | `test_friction.sh` | 六轴单轴小行程、多档速度、正反往复 |
| 多正弦/pilot | `multisine.py` / `trajectory.py` | `pilot.sh` | 原有激励轨迹 |

## 4. 关键文件

| 文件 | 职责 |
|---|---|
| `run.py` | 参数解析、主循环、试验切换、记录落盘 |
| `controller.py` | 统一阶段机、停止/回位、监视接入 |
| `hardware.py` | CAN 收发、使能/失能、模式切换、MIT 报文 |
| `simulation.py` | MuJoCo 仿真后端 |
| `monitoring.py` | 速度/电流/跟踪/温度/电压监视 |
| `standing.py` | 启动姿态校验与准备路径 |
| `test_data.py` | 样本记录与 `test_signals.npz/csv` 导出 |
| `ui.py` | 终端输出（标题/状态/警告/故障块） |
| `session_history.py` | 上次会话交接与恢复提示 |

## 5. 数据流

```
runs/<时间戳>/
  config.json            本次配置
  reference_plan.json    轨迹、增益、前馈、限值
  hardware.json          只读查询到的固件限位与版本
  samples.jsonl          全阶段样本（含 TESTING）
  test_samples.jsonl     仅测试段
  test_signals.npz/csv   导出信号与 trial_* 标签
  events.jsonl           时间线
  result.json            结果摘要
```

## 6. 安全分层

- **HARD**：驱动故障、反馈中断、通信中断、角度越界 → 故障停止；
- **SOFT**：持续超速、持续过流、持续跟踪误差 → 制动回位失能；
- **WARN**：瞬时超限 → 只打印；
- **RECORD**：只记录，不动作。

恢复：启动时若上次未正常交接，提示按 Enter 确认支撑；确认后发一次重置并验证，再继续。

## 7. 当前未决事项

- MOVE J 是否接受限位外的目标位置：待用现有 `BOUNDARY_ENTRY` 流程实测确认；
- `standing_start` 标志在 controller 与 preflight 中语义不一致（是否需要 workspace 确认），需要改成显式的“起始姿态模式”；
- 摩擦采集的正式流程：LB+X 到站立位 → LB+Y 开始 → 自动跑完 → 回位失能。
