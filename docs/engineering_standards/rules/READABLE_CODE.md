# 可人工阅读与维护的分语言编码规则

版本 1.0 · 2026-11-05。适用于所有后续工程的新建和修改。目标：维护者能从名称找到功能、从入口看懂流程、修改一个规则时知道影响和验证方式。

本规则参考华为 LiteOS 的组织方式，按“项目 → 文件 → 类型 → 方法 → 变量 → 注释 → 错误”制定本地要求，不冒充任何厂家的完整规范。排版以 CODE_FORMAT.md 为准，方法注释以 METHOD_COMMENTS.md 为准。已有公开接口、配置协议、反射/绑定名称和语言版本不能为改名而破坏。

## 1. 通用强制规则

| 编号 | 对象 | 执行规则与例子 |
| --- | --- | --- |
| READ01 | 项目 | 名称表达业务，README 写明用途、入口、目录职责、依赖、构建、运行和测试命令。应用独立运行，不依赖 IDE 私有路径。 |
| READ02 | 文件/模块 | 按功能命名，例如 camera_session、VolumeCalculator、project-catalog；不出现 Common2、UtilsNew、final_final。已有约定保留，新文件与相邻模块一致。 |
| READ03 | 类/结构/接口 | 名词或名词短语；注释说明职责、持有的状态及重要生命周期。一个类型不混合控件布局、算法、数据库和设备协议。无状态小功能允许普通函数，不强制造类。 |
| READ04 | 方法 | 动词＋业务对象，例如 LoadBoxPreset、calculateCapacityCm3；名称反映读写或副作用。禁止用 Process、Do、Handle1 隐藏实际工作。框架规定的事件/重写名称例外。 |
| READ05 | 方法体 | 主要流程按输入检查、处理、输出排列；用具名中间结果解释复杂判断；不写嵌套三元、布尔参数串或副作用混杂的表达式。 |
| READ06 | 变量 | 使用业务词和单位，例如 frameWidthPx、timeoutMs、loadPercent；用 is/has/can 表达布尔意义。短循环索引 i/j、惯用坐标 x/y/z 可保留，坐标系须明确。 |
| READ07 | 常量/状态 | 阈值具有名称、单位、范围和来源；状态用枚举/明确类型表达。不得把未知、未执行、错误替换为有效零值。 |
| READ08 | 注释 | 所有具名方法（含构造、私有、事件、局部函数）说明用途；复杂步骤解释原因、边界和资源处理。类型说明职责；字段仅在含义、单位、状态归属不明显时补注释。不逐行翻译赋值。 |
| READ09 | 资源/并发 | 明确谁创建、谁释放、何时取消、在哪个线程回调。UI 不同步等待长任务；后台结果校验会话和输入归属。 |
| READ10 | 错误/界面 | 错误状态保留。界面用简短中文说明影响及下一步；内部堆栈、路径细节放日志，禁止自动反复弹整屏技术异常。编译诊断在输出/问题面板保留。 |
| READ11 | 人工修改 | 业务规则集中在相应模块；布局生成和手写事件分开；不依赖神秘数值下标，不用运行时生成代码替代普通可编辑源文件。 |
| READ12 | 兼容重构 | 使用语言重命名能力或逐引用修改，并检查字符串反射、XAML/Vue 绑定、P/Invoke、导出符号、JSON、数据库。名称改善与算法/阈值调整分开提交或明确区分。 |

方法约 60 个有效代码行、文件约 500 行、嵌套超过 3 层时需要检查职责；这是人工审阅信号，不是硬性截断。状态机、生成布局、数据表及有证据的性能热点可保留并解释。禁止为凑行数拆成大量无意义转发函数。

## 2. 按语言执行

### C

- 项目/模块与文件用业务名，小写下划线：camera_session.c/.h；接口头只放必要声明，内部函数用 static 限定范围。
- 新独立 C 模块类型用 PascalCase；普通函数使用模块前缀加明确动作，例如 camera_session_open；参数、字段和局部变量用 lowerCamelCase；宏/枚举常量用 MODULE_UPPER_SNAKE_CASE。既有 tv_* ABI、驱动或华为 LiteOS 模块保留原命名。
- Doxygen 说明缓冲区长度、单位、输出参数、错误码和释放方；一个声明只定义一个变量。输出参数只在成功条件下有效，调用者不得猜测。
- Allman 大括号、4 空格。上述命名为本地选择，不声称与 LiteOS 大驼峰函数规则完全相同。

### C++

- 文件 volume_calculator.cpp/.hpp；类/结构/枚举 VolumeResult；函数 calculateVolume；局部 cellHeightMm；私有字段 cellHeightMm_；常量 kMaxCellCount。遵守已有模块的一致风格。
- 类型注释说明职责；函数 Doxygen 说明所有权、单位、坐标系及失败条件。使用 RAII，默认不写裸 owning 指针，不用宏代替普通函数。
- 传统算法默认放独立 C++ 库；C ABI 不暴露 STL、不让异常跨边界；命名规范不改变 ABI、结构布局或调用约定。

### C# / WinForms / WPF

- 工程/命名空间采用明确业务层级，例如 Tote.Measurement.Core；文件与主要类型一致，如 VolumeCalculator.cs。
- 类型/方法/属性 PascalCase；接口 ICameraSource；参数/局部 camelCase；新模块私有字段 _cameraSession；常量 MaxCellCount。真实异步方法用 Async 后缀，不在同步方法上假加 Async。
- 类型、构造和所有具名方法用 XML summary；必要时 param/returns/exception。控件使用 openCameraButton、resultGrid 等业务名，避免 button1/textBox2；改旧控件时同步 Designer、事件和资源绑定。
- .Designer.cs/.g.cs 与手写事件/应用服务分开；WPF XAML、绑定 ViewModel 和服务分开。生成文件不手工塞入业务逻辑。
- 保留工程目标 C#/.NET 版本；不因示例使用新语法而升级旧 Framework 项目。

### Java / Android Java

- 包名全小写，公开类与文件同名；类 BoxPresetService，方法 loadBoxPreset，局部 presetId，常量 MAX_RETRY_COUNT。
- 类型与全部具名方法使用 Javadoc，说明职责、参数和异常；不要给所有成员机械增加无效 getter/setter。
- UI Activity/Fragment 只做显示和生命周期协调；业务规则不塞进 onCreate/onClick。Android 资源名小写下划线，例如 button_open_camera。
- 阿里 P3C 用于 Java；现有 Android/AOSP 子工程遵守原规定，不跨项目强制改成 AOSP 的 mField 风格。

### Kotlin / Android Kotlin

- 类型/文件 PascalCase，函数与属性 lowerCamelCase，const val 常量 UPPER_SNAKE_CASE；KDoc 描述类型和具名函数。
- 明确 nullable 语义，避免无理由 !!；协程注明所属 scope、取消和线程要求，不把全局协程用于页面生命周期任务。
- Compose 组件按框架惯例 PascalCase；资源和 Gradle 结构保持框架规范，不强行搬进通用 source 目录。

### JavaScript / TypeScript / Node.js

- 类型/组件 PascalCase，函数/变量 camelCase，真正固定常量 UPPER_SNAKE_CASE；组件文件 PascalCase，普通文件沿用模块的 kebab-case 或既有约定。
- 使用 JSDoc/TSDoc；每个具名方法说明用途，复杂异步流程说明取消、过期结果、失败传播；禁止用类型断言代替外部输入校验。
- 一个模块负责一种业务；网络、文件、进程访问封装在适配模块，渲染代码不直接持有系统权限。命令、输入、路径使用结构化参数，不拼接不可信 shell 字符串。
- new 项目采用 strict TS；现有 JS 不为符合规范而自动迁移 TS。4 空格、起始大括号同行，分号保留。

### Vue / HTML / CSS

- Vue 组件使用多词业务名，如 BoxPresetEditor.vue；props 说明类型/含义，事件说明携带数据，复杂计算进入具名函数或 composable。
- 模板只保留展示和简单绑定，不塞算法、复杂三元及长链式表达式；单文件组件不是把所有业务写在一个文件的理由。
- HTML 使用语义标签，CSS 类表达用途，例如 project-list、is-selected；不使用 div1、blueText1 绑定业务。样式令牌集中维护，禁止为格式要求改变已有 class/selector 协议。
- HTML/CSS/XML 用 2 空格；脚本遵循 JS/TS 规则。对组件边界和特殊布局约束写注释，不给每个 div 重复说明。

### Python

- 模块 snake_case.py，类 PascalCase，函数/变量 snake_case，常量 UPPER_SNAKE_CASE；内部约定用单下划线前缀。
- 类及所有具名函数写 docstring；必要时说明 Args、Returns、Raises，算法/模型注明单位、形状、数据类型和设备假设。
- 路径/模型加载/推理与 UI 分离，资源使用 context manager 或 finally。类型提示按工程版本执行。
- Python 的使用范围仍遵守 CPP_PYTHON_RULES.md：直接 AI 模型工作或用户明确指定；不是所有算法默认转 Python。

### JSON / YAML / XML / XAML

- 键名、序列化字段、路由和绑定是契约，不因统一大小写而重命名。
- JSON 不写注释；schema 的 title/description 说明含义、单位、范围，或写相邻配置说明。schema 的 default 不等于运行时自动填充值。
- YAML/XML/XAML 用注释说明复杂配置块与绑定约束。环境路径和凭据不硬编码进通用项目模板。

### PowerShell / Bash / CMD / CMake / SQL

- PowerShell 用批准动词＋业务名，如 Get-ProjectStatus，参数 PascalCase、局部 camelCase；函数写注释帮助，保留实际错误退出码。
- Bash 函数/局部变量 snake_case，常量与导出环境变量 UPPER_SNAKE_CASE；函数前说明输入、输出、副作用及退出码，路径引用加引号。4 空格是本地选择。
- CMD 子程序标签和 CMake 函数写业务名及用途注释；声明输入、输出和工作目录；不使用系统变量名承载业务临时值。
- SQL 表列名沿用数据库契约；新模块用一致 snake_case，查询别名可读，禁止 a1/b2 代表复杂业务。迁移与回滚条件清楚，事务边界有说明。

尚未列出的语言：先查语言官方/维护方规范，再形成项目差异表；继续执行通用规则，不虚构“全部语言已经配置自动检查”。

## 3. 如何改旧工程

1. 先记录该模块入口、调用方、公开接口、持久化字段、构建命令与当前验证基线。
2. 先补准确注释和业务命名；一次只处理一个功能边界。使用符号重命名，禁止全目录字符串替换所有相同短词。
3. 方法过长时按实际步骤拆分，保留处理顺序、线程、异常和资源归属；不擅改算法、标定、容差或用户配置。
4. 运行对应编译、关键回归及适用静态检查；说明改了哪些模块、剩余范围、哪些工具没运行。
5. 第三方、自动生成布局、构建缓存、发布包不做可读性重写。业务代码人工能修改并独立构建是验收重点，不以注释数量代替质量。

## 4. 可核实来源

查阅日期：2026-11-05。以下为参考和摘要入口，不复制整套原文；具体执行要求是本文件的本地适配。

| 语言/对象 | 官方或维护方来源 | 选取内容 |
| --- | --- | --- |
| C | [华为 LiteOS 编程规范](https://support.huaweicloud.com/contribute-LiteOS/zh-cn_topic_0145350121.html) | 命名、函数、注释、实现意图；不照搬内核专用命名 |
| C++ | [C++ Core Guidelines](https://github.com/isocpp/CppCoreGuidelines) | 接口表达、职责、资源和可读性 |
| Java | [阿里 P3C](https://github.com/alibaba/p3c) | 命名、注释和工程约定，仅 Java 原生规则 |
| C# | [Microsoft 命名约定](https://learn.microsoft.com/en-us/dotnet/csharp/fundamentals/coding-style/identifier-names) | 类型、成员、参数命名；保留现有语言版本 |
| Kotlin | [Kotlin Coding conventions](https://kotlinlang.org/docs/coding-conventions.html) | 源文件、类型、函数及属性 |
| Android Java | [AOSP Java code style](https://source.android.com/docs/setup/contribute/code-style) | Android 平台参考，应用项目保留自身一致规则 |
| JS/TS | [Google TS/JS Style Guide](https://google.github.io/styleguide/tsguide.html) | 模块、命名、注释；缩进使用本地 4 空格 |
| Vue | [Vue Style Guide](https://vuejs.org/style-guide/) | 组件组织，按项目 Vue 主版本选取 |
| HTML/CSS | [Google HTML/CSS Style Guide](https://google.github.io/styleguide/htmlcssguide.html) | 语义与命名，不照搬所有排版选项 |
| Python | [PEP 8](https://peps.python.org/pep-0008/) | 类型/函数/变量命名及一致性 |
| PowerShell | [Microsoft Approved Verbs](https://learn.microsoft.com/en-us/powershell/scripting/developer/cmdlet/approved-verbs-for-windows-powershell-commands) | 动词＋名词的命令命名 |
| Bash | [Google Shell Style Guide](https://google.github.io/styleguide/shellguide.html) | 函数、变量、引用和失败路径 |
| JSON | [JSON Schema Annotations](https://json-schema.org/understanding-json-schema/reference/annotations) | 用 schema 描述配置，而非往 JSON 加注释 |

本规则与 CODE_FORMAT.md、METHOD_COMMENTS.md 共同执行。命名可由部分语言工具检查，业务语义、方法体注释准确性和职责划分仍需人工/代码审阅；不能声称格式化器已自动证明可维护性。
