# C++ / C# 混合工程目录规则

适用：C++ 算法 DLL + C# 桌面应用。版本 1.0。以一个可独立构建和发布的应用作为一个工程根目录；按职责分目录，需要时再拆项目，不为目录规则创建空模块。

## 1. 标准目录

```text
ApplicationName/
├─ source/                         自有源代码
│  ├─ native/                      C++ 算法与 DLL
│  │  ├─ include/                 对外头文件、C ABI 声明
│  │  ├─ src/                     算法实现、内部头文件、C ABI 实现
│  │  ├─ tests/                   原生单元与回归测试
│  │  └─ tools/                   自有原生回放/诊断工具（按需）
│  ├─ managed/                    C# 应用
│  │  ├─ ui/                      窗体、控件、布局与显示绑定
│  │  ├─ application/             流程编排、配置模型与结果处理
│  │  ├─ interop/                 P/Invoke、原生结构映射和托管封装
│  │  ├─ camera/                  相机生命周期与帧适配（按需）
│  │  ├─ infrastructure/          文件、日志、网络等外部访问
│  │  ├─ resources/               图标、图片、语言资源（按需）
│  │  ├─ Program.cs               应用启动与组件组合
│  │  └─ ApplicationName.csproj   C# 项目配置
│  ├─ capture/                    自有独立采集程序（按需）
│  ├─ tests/managed/              C# 测试
│  └─ examples/                   最小调用示例（按需）
├─ thidparty/                      第三方库、SDK、工具及环境说明
├─ config/                         出厂默认配置
├─ data/                           用户配置、标定和现场数据
├─ samples/                        小型回放与测试样本
├─ scripts/                        编译、测试、启动、打包脚本
├─ docs/                           开发与使用文档
├─ build/                          自动生成的编译中间文件
├─ app/                            整理好的可运行程序目录
├─ dist/                           ZIP、安装包等分发文件
├─ reports/                        构建与测试报告
├─ CMakeLists.txt                  C++ 构建入口
├─ ApplicationName.sln            可选：确有多个项目时提供解决方案
├─ README.md                      环境、构建、启动和目录说明
├─ AGENTS.md                       AI 编码约定入口
├─ .editorconfig                   通用/C# 格式配置
├─ .clang-format                   C++ 格式配置（启用格式化时提供）
└─ .gitignore                      排除中间产物和现场数据
```

保留用户已指定的 `thidparty` 拼写，不再同时创建 `thirdparty`、`third_party` 或根目录 `libs`。已有工程的其他有效目录先保留，只有本次涉及的模块才按规则调整。

## 2. 文件放置规则

| 文件或功能 | 放置位置 | 规则 |
| --- | --- | --- |
| C ABI 声明 | `source/native/include/` | C# 调用边界的原生声明唯一来源；不混入 UI 或厂商私有类型。 |
| C++ 内部实现与头文件 | `source/native/src/` | 内部实现不全部暴露到 include；文件沿用 `volume_calculator.cpp` 风格。 |
| C# 原生调用 | `source/managed/interop/` | DllImport、结构映射集中维护，窗体不散落 DllImport。 |
| 窗体布局 | `ui/MainForm.Designer.cs` | 仅布局和控件初始化；生成器只覆盖明确归属的布局文件。 |
| 窗体事件与展示 | `ui/MainForm.cs` | 输入和调用应用流程；不堆积分公式和设备协议。 |
| 窗体关联资源 | `ui/MainForm.resx` | 保持 WinForms 配对关系；通用图标和语言文件放 resources。 |
| 检测流程 | `application/DetectionService.cs` 等 | 负责组织调用；不引用具体窗体控件。 |
| 自己编写的采集器 | `source/capture/` 或现有 camera 适配目录 | 自有代码不放 thidparty；第三方 SDK 放 thidparty。 |
| 默认参数 / 现场设置 | `config/` / `data/` | 分开存储，软件升级不能无条件覆盖现场设置。 |
| 自动生成文件 | 生成器声明的专用目录或 `*.g.cs` 等标记文件 | 明确能否重建；不手工修改生成输出，不把布局产物与业务混写。 |

一个 `.csproj` 可以先管理多个职责目录。需要独立复用或测试隔离时再拆为多个类库；通过 ProjectReference 引用，不手工复制 C# 源码到多个项目。新增生成源码时检查编译包含项，避免重复编译。测试目录不得被生产项目递归编译进去。

## 3. C++ 与 C# 的连接方式

```text
C# 界面 → C# 应用流程 → C# interop 封装 → C ABI → C++ 算法
                         ↓
                 相机/文件等适配器
```

相机采集和算法输入通过明确的数据结构连接。原生 include 中声明的结构与 managed/interop 中的映射同步修改；用契约验证核对大小、对齐、调用约定、单位、版本及资源所有权。算法不反向依赖 C# 或控件。

## 4. 第三方依赖与环境

第三方按名称集中管理，例如 `thidparty/opencv/`、`thidparty/hp60c/`，内部按实际需要分为 `include/`、`lib/`、`bin/`；不同版本、平台或架构并存时增加版本和平台子目录。保留许可证、来源和版本说明。

- `thidparty/tools/`：可以合法随工程分发的工具。
- `thidparty/environment/`：系统环境要求、安装说明及允许分发的离线安装包。
- 不能简单复制的 Visual Studio、Windows SDK、.NET 系统环境，记录安装要求，不宣称整个编译环境已便携化。
- 若使用 NuGet 等包管理器，项目中保留依赖声明和适用的锁定文件；需要离线分发时才建立 thidparty 内的本地源，并记录版本，禁止直接依赖某个人缓存目录里的 DLL。

## 5. 编译、发布与 Git

| 目录 | 用途 | 管理要求 |
| --- | --- | --- |
| `build/<平台>/<配置>/` | CMake、MSBuild 中间文件 | 不手工编辑、不作为源码提交；旧 build 结构不强行迁移。 |
| `app/<平台>/` | 可直接运行的程序及依赖副本 | 由构建脚本装配；禁止从这里反向取依赖编译。 |
| `dist/` | 安装包、压缩包 | 由打包脚本产生，通常不放普通源码 Git 历史。 |
| `reports/` | 测试和构建结果 | 保留必要交付证据，避免提交无界日志和敏感现场数据。 |
| `data/` | 用户和现场内容 | 默认不进源码仓库、不带入通用安装包；可保留说明和无敏感信息的示例。 |

新项目建议配置中间输出到 build；遗留 `.csproj` 的 bin/obj 先排除版本控制，改变路径时同时验证构建和清理脚本。native 编译结果按脚本装配到 app；必要 SDK DLL 可以复制到 app 满足加载要求，但开发依赖原件仍在 thidparty。

运行程序、原生 DLL、SDK DLL 架构必须匹配，例如统一 `win-x64`。不混装不同配置或不同版本的 ABI。路径从工程根目录、程序位置或明确配置解析，不写死开发电脑绝对路径，不依赖相邻应用或 Forge IDE 安装目录。

建议脚本入口：`Check-Environment.ps1`、`Build.ps1`、`Test.ps1`、`Start.ps1`、`Package.ps1`。只有实际实现的脚本才列入 README。目录标准不等于已支持所有平台；当前 WinForms 工程仍保持 Windows、C++17、C# 5/.NET Framework 4.8。

## 6. 新文件放哪里：简明判断

1. 自己写的代码 → source；第三方提供的 → thidparty。
2. 界面 → managed/ui；流程 → managed/application；原生调用 → managed/interop；算法 → native。
3. 出厂配置 → config；用户现场数据 → data；回放样本 → samples。
4. 编译中间文件 → build；可运行文件 → app；交付安装包 → dist。
5. 新功能沿用所属目录，不再平行创建 `src/source`、`common/utils/helpers` 等含义重复的容器。

本规则已按当前 ToteVolumeMeasurement 目录整理；本次仅制定规则，不移动源码、不调整构建或部署路径。复制此文到独立工程 docs 后即可单独使用。
