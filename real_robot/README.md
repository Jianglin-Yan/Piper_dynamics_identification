# 最小旧接口兼容目录

业务实现位于`src/piper_dynamics/`。2026-11-06删除无实际调用方的旧转发文件后，本目录仅保留以下依赖：

| 保留项 | 原因 |
| --- | --- |
| identify_offline.py、designed_trajectory.py | 相邻identification工具仍直接导入 |
| design_trajectories.py、pilot.sh | 外部轨迹概览工具仍生成这些入口的复现命令 |
| simulation.py | 外部复算工具仍按此路径读取源码指纹，同时兼容导入 |
| __init__.py | 上述旧模块名的包引导 |
| config.json、multisine_profile.json、assets/ | 正式代码默认配置、轨迹设计及归档仍引用这些路径；真实内容通过软链接共享 |
| .gitignore、本地g8.json | 保留手柄配置的本地存储约定，g8.json不进入Git |

新调用请使用[当前运行说明](../docs/real_robot.md)中的scripts和piper_dynamics模块。旧test_*.sh、其余Python转发、tests目录别名及requirements.txt已删除；生产算法和控制流程未迁回本目录。
