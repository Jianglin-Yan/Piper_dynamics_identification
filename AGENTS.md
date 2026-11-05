# 项目工作约定

- **准备／回位／急停的 Kp/Kd 是基础设置**：除用户明确说明外不得修改。任务只能改测试段的 Kp/Kd；进入和退出时 Kp、Kd、前馈必须由同一渐变一起过渡，急停接管使用回位基础增益。
- **限位外扩是公共约定**：`measurement_angle_tolerance_deg` 与 `startup_limit_tolerance_deg` 长期保持 10°，即实测反馈和启动停靠位允许超出配置/固件限位 10°。任何新任务不得用更窄的 `lower/upper` 覆盖它；需要从停靠位启动或返回的 MIT/拖动任务，命令区间必须同时覆盖固件范围和实测停靠位。缩小该范围必须得到明确任务授权。
- 修改前阅读 `CODING_STANDARD.md`、`README.md` 和 `ARCHITECTURE.md`；具体控制逻辑以当前实现为准，旧会话记忆及 main 的离线流程不能覆盖当前控制行为。
- 顶层沿用 main 的 src、inputs、models、outputs、scripts、docs 管理方式，src 内按职责组织；项目只支持普通 PiPER。
- 产品源码放 `src/piper_dynamics`。core 放领域算法，pipeline 放离线业务编排，real_robot 放运行与控制，utils 放通用工具；依赖保持单向。
- 函数承担明确职责；优先拆分配置、运行、数值计算和记录输出，不按行数机械切碎连续逻辑。数学惯用符号与已有文件/字段契约可以保留。
- 当前控制参数和流程的变化必须来自具体任务需求，不借结构整理顺带修改增益、阈值、模式、时序、任务顺序或训练/验证划分。
- 模型使用 `models/standard_piper`，路径使用 `paths.py`；复用现有运行记录、manifest、实验索引和相邻修订归档，不批量改写历史数据与哈希。
- 根目录 real_robot、tools、excitation.py 是过渡兼容入口；外围归档工具迁移前保留旧调用，业务实现只能有一份。兼容引导以外的源码不用 sys.path 拼接或动态加载替代正常导入。
- 本次已批准的整理及验证结果见 `docs/refactor_report.md`；后续按新任务范围继续维护规范。
- 验证使用 `PYTHONPATH=src:. python -m pytest -q` 和与改动相关的离线/仿真检查。明确报告静态、数值、仿真与实机证据；硬件操作按用户当前任务授权执行。
