# G8 控制的 PiPER 小幅单轴试采

当前范围：等待手柄 → 准备到起点 → 单轴小幅激励 → 制动/回位 → 结束；配套 MuJoCo 预览和原始数据记录。没有开放多轴激励，也没有把试采数据自动拟合为惯性参数。

本目录是独立入口，不依赖旧仿真工程的绝对路径。原库 excitation/data_collector/regressor/identify/validate 保持原样，后续以这里采集的真实数据接入原库回归器。

## 从当前蜷缩姿态直接进行实机 J6 试采

用户已指定从当前停放角度启动。此路径需要显式 `--from-current`：

```bash
cd ~/Documents/41Piper/Piper_dynamics_identification
bash real_robot/pilot.sh hardware --from-current
```

程序先读取实际六轴角度并等待，LB+X 提示无需移动，全部松开后 LB+Y 才使能并开始 J6 ±5° 试采。正常结束或 LB+RB 中止后回到这次启动时的角度。

当前姿态模式只允许 J6、`start_deg=null`、幅度不超过 5°、频率不超过 0.1 Hz：

- J2 低于配置下限、J3 高于配置上限，各允许最多 3° 的停放反馈偏差；其他方向或关节的越界仍拒绝。
- J1–J5 的位置目标始终固定为本次启动读数，不从文件套用停放角度，不清零。其实际反馈允许偏离启动值最多 0.5°；使能前若任一轴相对启动值变化超过 0.1°则拒绝，要求重新启动读取。
- J6 仍受原限位与 5° 余量约束，整条 ±5° 轨迹必须通过检查。
- 设备固件、零位、限位和碰撞检测等级均不改写。该模式按操作者的明确指令接受现有保护配置（当前碰撞检测为 0），终端打印它；`workspace_checked` 与 `collision_policy_reviewed` 的历史字段不自动改成 true。普通指定起点模式仍使用原来的检查。
- 若固件拒绝当前保持目标或报告故障，停止并锁定，不清错重试。软件预检通过不代表固件已经接受目标，或实机运动已经通过验收。

只验证实机启动条件，不读取按钮动作，也不使能或运动：

```bash
bash real_robot/pilot.sh hardware --from-current --preflight-only
```

`PREFLIGHT_ONLY` 表示仅检查完成，不是实机运动测试通过。此模式的 MuJoCo 预览也可用 `--sim --viewer --from-current`；只有仿真中的保持关节限位按锚点设置，不影响设备。

## 快速使用

在本机终端执行：

```bash
cd ~/Documents/41Piper/Piper_dynamics_identification
# 首先检查手柄，完全不连接 CAN
/home/shin/anaconda3/envs/lerobot/bin/python -m real_robot.run --monitor-pad --max-seconds 300
# Ctrl+C 退出后，运行独立仿真；也不连接 CAN
/home/shin/anaconda3/envs/lerobot/bin/python -m real_robot.run --sim --viewer --max-seconds 300
```

终端必须保持可见。仿真窗口显示状态、当前试采关节角度和目标角度。默认 J6、±5°、0.1 Hz、20 秒，前后各 5 秒平滑包络，动作很小。仿真初始姿态是用于演示的 `[0,45,-60,0,20,0]°`，不是当前实机姿态，也不是下发给实机的默认目标。

| 输入 | 行为 |
|---|---|
| LB+X | 准备。`start_deg=null` 时使用程序启动姿态，终端提示无需移动；指定起点时走受限五次多项式过渡。|
| LB+Y | 只有 READY 才开始；未准备时仅提示。|
| LB+RB | 最高优先级受控中止：取消激励，按当前实测位置/速度制动，检查回程后返回程序启动位置。|
| Ctrl+C / SIGTERM / 关闭运行中的仿真窗口 | 故障停止，不进行无人确认的自动回位。|

组合键之间先全部松开。启动时已经按住的键不会触发准备/开始。短按事件从 evdev 队列逐个读取，不依赖键盘式长按；同一批事件中 LB+RB 优先。完成后实机保持位置，不自动失能卸力；仿真窗口额外保留最多 60 秒供观察，也可手动关闭。再次测试需重新启动程序，重新读取初始姿态。

**LB+RB 不是安全认证急停。** 健康状态下按你的要求执行制动回位；反馈失联、驱动故障、失能、跟踪异常、回程越界等条件下，禁止回位，尝试发送固件快速停止并锁定。若 CAN 已断，软件不能保证停止指令送达。蓝牙断连检测依赖 Linux 驱动上报，不能保证无线丢包瞬间响应；独立硬件急停仍需可用。

## 手柄校准

本机已实际读取 G8（蓝牙名称 Wireless Controller）的 LB=310、RB=311、X=308、Y=307。校准文件 `g8.json` 按设备身份匹配，不按 event26 之类易变编号匹配。文件留在本机且不提交 Git；其他机器需要重新校准：

```bash
/home/shin/anaconda3/envs/lerobot/bin/python -m real_robot.run --calibrate-pad
```

按终端提示依次单独按下并松开 LB、RB、X、Y。仅读取游戏手柄设备，不读取键盘。若提示权限不足，先检查当前用户对相应游戏手柄 event 节点的读取权限，不建议用 root 运行整个运动程序。

## 实机执行前

只读检查可随时执行，不使能、不运动、不改保护：

```bash
/home/shin/anaconda3/envs/lerobot/bin/python -m real_robot.run --read-only
```

当前锁定的已核查固件标识为 `S-V1.7-3250618`。不会自动升级固件或 SDK。实际使用普通 MOVE J 位置模式，不使用 MIT，因此不设置实机 Kp/Kd。内部规划可能平滑目标，辨识只能使用实际测得的运动量。

`config.json` 记录实机确认状态：

- `model_confirmed`：用户已确认型号为 PiPER X，当前值为 true；实际关节方向仍需通过试采核验。
- `workspace_checked`：底座、工作区域、线缆、夹爪与完整往返路径已现场检查。
- `collision_policy_reviewed`：仅在任一碰撞检测等级为 0 时使用；必须先处理或明确评估这个实际保护配置。程序不会偷偷开启/关闭检测。

这些项只按已确认的现场事实更新。真实角度限位逐轴读取，和本地保守配置取交集，并再留 5° 余量。普通模式启动姿态必须在内部；当前 J2/J3 停放反馈略超返回限位，普通启动检查器会拒绝。显式 --from-current 的例外见上节。这是程序的启动条件，不代表机械臂姿态异常或固件已经报故障；错误信息区分“反馈超出配置范围”和“仅未满足程序余量”。程序不自动钳位角度、不清零、不扩大限位，也不规划越界停放姿态的恢复运动。

确认以上实际条件、完成仿真和手柄检查后，才在有人看护的终端运行：

```bash
/home/shin/anaconda3/envs/lerobot/bin/python -m real_robot.run --hardware --max-seconds 300
```

启动后仅查询并等待。只有手柄请求准备到不同起点或请求开始测试时才会下发使能与运动。只允许一个本程序实例持有同一 CAN 运动锁；仍需退出其他遥操作/控制程序，这个锁无法约束第三方软件。

## 配置和停止逻辑

- `joint`：1–6，一次一个关节。首轮默认 J6。
- `start_deg`：null 表示启动姿态；否则为六轴绝对目标，不能省略中间过渡。
- `amplitude_deg`、`frequency_hz`、`duration_s`、`ramp_s`：窗口化单正弦参数。
- 速度、加速度上限默认均 0.12（rad/s、rad/s²）；预检覆盖去程、激励和回程，命令也逐次检查边界。路径预检为密集离散采样，不是连续时间形式化证明。
- 100 Hz 控制/快照记录，原始 CAN 接收按各帧真实到达频率记录；本机角度与高速反馈约 200 Hz，驱动低速状态约 40 Hz。
- 反馈超时、状态异常、跟踪误差、实际超速、软件电流/温度/电压阈值、手柄设备移除、接收溢出、日志写入失败均终止会话。参数是首轮工程阈值，不是厂商认证的安全限值。
- 独立看门狗检测控制循环/接收异常；发出快速停止后设置锁存，后续位置和使能命令被拒绝。它不能保证 Python 进程崩溃、系统停机、CAN 或蓝牙故障下的安全。
- 位置目标在使能前初始化到当前测量值；完整六轴使能反馈确认后才执行试采。
- 不控制夹爪、不执行清错/复位/重新标零，不自动解除急停，不在停止后自动重试。

## 数据在哪里、包含什么

每次运行写入仓库的 `runs/日期时间/`（不进入 Git），也可用 `--output PATH` 指定：

| 文件 | 内容 |
|---|---|
| config.json / effective_config.json | 用户配置及与硬件限位合并后的实际配置 |
| hardware.json | 固件原始标识、逐轴新鲜限位回复、碰撞等级 |
| raw_can.jsonl | 实机原始接收帧、主机单调时间、主机墙上时间 |
| tx_can.jsonl | 每条发送查询/目标/停止/使能报文与时间，便于追溯 |
| samples.jsonl | 控制状态、按钮事件、目标 q、实测 q/qd/电流、各反馈帧时间、状态、温度、电压、夹爪反馈 |
| events.jsonl | 准备、开始、中止、回位及故障原因 |
| result.json | 仿真/实机标识、初始位置、完成/中止/拒绝结果 |

快照内不同信号是异步帧，保留各自时间，不冒充同步数据。原始 CAN 是后续对齐与滤波的依据。

已核查的旧固件候选电流换算：J1–J3 为 4.725 Nm/A，J4–J6 为 0.95844 Nm/A。字段明确命名 `tau_candidate_Nm`，尚未通过已知负载验证，不能视作校准力矩。仿真同字段为开启重力补偿后的执行器输出，只用于试采流程展示，不能用于原库动力学辨识的真值检验。

导出保留真实接收时间的逐帧 CSV：

```bash
/home/shin/anaconda3/envs/lerobot/bin/python -m real_robot.export_signals runs/某次记录/raw_can.jsonl
```

绘制并汇总实际/目标轨迹：

```bash
/home/shin/anaconda3/envs/lerobot/bin/python -m real_robot.analyze runs/某次记录
```

输出 `analysis.json` 与 `trial.png`。当前阶段不对无运动/失能数据执行惯性拟合；下一阶段再加入重采样、滤波、实测加速度估计与摩擦回归。

## 仿真与自动测试

不需要手柄、不连接 CAN 的快速回归：

```bash
/home/shin/anaconda3/envs/lerobot/bin/python -m real_robot.run --sim --scripted normal
/home/shin/anaconda3/envs/lerobot/bin/python -m real_robot.run --sim --scripted abort
/home/shin/anaconda3/envs/lerobot/bin/python -m unittest discover -s real_robot/tests -v
```

`--scripted` 在实机模式下被拒绝。测试覆盖短按、按键优先级、启动时按住按钮、平滑起停、不同相位中止回位、指定起点、越界拒绝、反馈故障、不自动恢复、停止锁存、看门狗及与本机 SDK 的报文字节一致性。

模型运动学和名义惯性来自 [AgileX agx_arm_urdf](https://github.com/agilexrobotics/agx_arm_urdf/tree/680fa925cd00db7e2628563d2edc5540963045de)，版本和参数见 assets/nominal_x.json。模型使用简化胶囊/盒状外观、固定闭合夹爪、仿真专用增益与重力补偿，关闭几何接触计算，不复现固件限幅、摩擦、碰撞保护或真实周边障碍；只能预览动作，不能证明实机安全。

截至 2026-10-28：已经有只读实机记录、软件测试和 G8→MuJoCo 中止回位记录；**尚未完成真实机械臂运动及实机中止验收**。通过后再考虑多关节激励。
