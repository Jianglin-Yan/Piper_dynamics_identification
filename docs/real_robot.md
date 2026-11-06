# PiPER 运行与兼容说明

实现位于 `src/piper_dynamics/real_robot/`，项目用途、目录职责和离线入口见 [README](../README.md)。

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

2026-11-06清理后，实机/仿真启动统一使用上表`scripts/`入口，旧`real_robot/test_*.sh`已删除。摩擦分析使用`python -m piper_dynamics.pipeline.friction_combined|friction_bins|friction_final`，不再提供`tools/fit_*.py`转发。依赖安装使用根目录requirements.txt和pyproject.toml。

根目录`real_robot/`暂留`identify_offline`、`designed_trajectory`、`design_trajectories`、`simulation`四个Python别名和`pilot.sh`，因为相邻归档工具仍有导入、指纹读取或生成旧命令的依赖。配置/模型软链接、包引导及本地`g8.json`手柄配置保留；默认路径和控制行为不变。不再保证其他旧模块名可导入。

## 验证

```bash
PYTHONPATH=src:. python -m pytest -q
```

本轮220项测试及17项子测试通过，包含准备、测试、停止、回位、失能、协议编码、默认摩擦配置和完整仿真会话；另有13项实际入口帮助检查通过。测试与仿真不等同于新增实机验证；本次整理没有执行实机动作。
