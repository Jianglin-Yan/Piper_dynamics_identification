# PiPER 运行与兼容说明

实现位于 `src/piper_dynamics/real_robot/`，顶层结构见 [README](../README.md)，控制层次见 [ARCHITECTURE](../ARCHITECTURE.md)。

## 入口

| 入口 | 任务 | 控制方式 |
| --- | --- | --- |
| `bash scripts/test_mit.sh sim|hardware --ff-scale 0..1` | 固定五姿态 PD/重力前馈对照 | 当前 MIT-only |
| `bash scripts/test_static.sh sim|hardware` | 静态姿态采集 | MOVE J |
| `bash scripts/test_reciprocal.sh sim|hardware` | 正反对照 | MOVE J |
| `bash scripts/test_payload.sh sim|hardware` | 负载标注/采集 | MOVE J，保留原夹爪与腕部约定 |
| `bash scripts/test_friction.sh sim|hardware` | 六轴低速摩擦采集 | MOVE J |
| `bash scripts/pilot.sh sim|hardware|pad|read|recover` | 原有激励、手柄与诊断入口 | 由原任务配置决定 |

`--help` 查看完整参数。正常手柄动作继续为 LB+X 准备、READY 后 LB+Y 开始、LB+RB 中止；任务自己的切姿态和自动运行规则不变。`--headless --scripted normal` 用于无窗口仿真，`--scripted abort` 验证中止回收。

MIT 默认 Kp=10、Kd=0.8；准备与回位容差为当前 8°，前馈从零渐入、结束渐出。`--gravity-model nominal|identified` 保留原模型选择，identified 读取相邻归档的当前 PiPER 修订，模型哈希与换算系数仍需匹配。

摩擦默认顺序为 `[1,6,4,5,2,3]`，默认站立位开始；`--friction-start current` 显式选择实测当前姿态。关节列表只要求非空、合法和唯一，保留指定顺序；本次修复了默认非升序列表被错误拒绝的问题。

## 数据与路径

默认配置位于 `src/piper_dynamics/real_robot/profiles`；`--config`、`--pad`、`--output` 继续覆盖路径。运行记录默认写 `runs`，TESTING 专用数据继续与全流程诊断分开。

旧命令及 `real_robot.*` Python 导入仍由薄兼容层转发到 src 同一实现。相邻实验归档工具仍有旧导入和旧模型路径，因此暂不删除这些别名；后续迁移外围调用后才可移除。兼容层不是新增业务代码的位置。

## 验证

```bash
PYTHONPATH=src:. python -m pytest -q
```

当前共有 206 项测试，包括准备、测试、停止、回位、失能、协议编码、默认摩擦配置和完整仿真会话。测试与仿真不等同于新增实机验证；本次整理没有执行实机动作。
