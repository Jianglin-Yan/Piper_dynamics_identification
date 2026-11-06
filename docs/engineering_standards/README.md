# 个人全局代码规范 · 可迁移安装包

导出日期：2026-11-05。来自用户已确认的全局工程规则；不是阿里、华为等厂商完整官方规范的替代品。来源和项目适配说明保留在规则正文中。

## 最简单的用法

把整个 ZIP 发给其他 Agent，并发送这段话：

> 请解压这个规范包，先读 README.md 和 AGENT_INSTALL.md。将规则安装到你实际支持的用户级全局规则位置，保留原规则并备份，只合并本包的工程规则块；不要修改业务工程或执行全库格式化。安装后报告入口文件和规则目录，并说明是否需要新会话生效。如果没有用户级规则能力，明确说明，改为当前项目级安装，不能声称全局生效。

## 包里有什么

- `rules/`：8 份规范，包括分语言格式、方法及方法体注释、可读性、C++/C# 目录、C++/Python 选型、AI 生成要求、来源汇编及适用说明。
- `rules/code-format/`：EditorConfig、clang-format、Prettier 配置模板；**不自动覆盖工程现有配置**。
- `AGENT_RULES.template.md`：精简入口规则。安装时将占位路径替换为接收端实际路径，Agent 按任务读取对应章节。
- `Install.ps1`：安装/更新脚本，支持预览、备份、保留其他规则、重复安装不重复追加。
- `GLOBAL_CODE_RULES.md`：合并阅读版，适合一次上传阅读。长期使用建议安装目录版，减少每轮读取量。
- `manifest.json`：包内文件 SHA-256，用于检查拷贝完整性，不是数字签名。

规则重点：传统算法优先 C++；UI/流程/算法/外部适配分离；自有源码 `source`，第三方目录 `thidparty`；所有具名方法和类型有准确注释；变量含义和量值单位明确；保留接口、框架版本及既有行为，不无关重构。

## Windows / Codex 安装

解压后在包目录运行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\Install.ps1 -Preview
powershell -NoProfile -ExecutionPolicy Bypass -File .\Install.ps1
```

默认使用 `CODEX_HOME`；未设置时使用当前用户的 `.codex`，规范进入 `standards/engineering`，入口为 `AGENTS.md`。这两条命令只对本次 PowerShell 进程设置执行策略，不永久修改系统策略。

## 其他 Agent / 自定义位置

先确认目标 Agent 真正支持的用户级规则入口，然后显式指定，以下路径只是示例：

```powershell
.\Install.ps1 -RulesDirectory 'D:\AgentRules\engineering' -EntryFile 'D:\AgentConfig\AGENTS.md' -Preview
.\Install.ps1 -RulesDirectory 'D:\AgentRules\engineering' -EntryFile 'D:\AgentConfig\AGENTS.md'
```

指定一个文件名不代表目标 Agent 会自动加载它。若该产品使用设置面板、不同规则格式或项目级配置，按 `AGENT_INSTALL.md` 手动接入，不盲目使用上述示例。

## Linux / macOS

已安装 PowerShell 7 时，在解压目录运行 `pwsh -NoProfile -File ./Install.ps1`；也可以用同样参数指定路径。本次实测 Windows 上的 PowerShell 5.1 和 7.6.5；Linux/macOS 脚本运行未验收。

没有 PowerShell 时：复制完整 `rules/` 到接收端用户目录，将入口模板中的 `{{RULES_DIRECTORY}}` 替换为该目录绝对路径，再按标记合并到实际全局规则文件。详细步骤在 `AGENT_INSTALL.md`，不需要为手动复制安装额外运行环境。

## 更新与恢复

脚本保留入口中其他内容，遇到重复或损坏的工程标记会报错停止。受影响的旧文件先备份到入口文件旁的 `engineering-backup-*`，其中 `restore-index.json` 记录原路径与备份对应关系。

恢复时先核对安装后是否有新修改，再按索引复制 `.bak` 到原路径；`existed=false` 表示安装新增文件，不能盲目删除后来已修改的文件。备份不自动清理。

包不含个人凭据、聊天记录、相机 SDK、界面设计基准或三省六部流程。安装指导规范不等于安装了编译器/格式检查插件，也不代表旧项目自动全部合规。
