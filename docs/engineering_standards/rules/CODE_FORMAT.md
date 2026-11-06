# 分语言代码格式规则

版本 1.0。这里只规定命名、缩进、换行、空格、注释和文件排版，不扩展架构或业务要求。这是项目选择的统一风格，不宣称逐条等同于阿里、微软或其他厂家的原文。

已有文件保持一致；公开接口、C ABI、配置键、数据库字段不因格式要求改名。新模块采用本表；格式化仅处理自有源代码，不处理第三方、生成文件和发布产物。语言版本保持原工程设置。

## 1. 所有语言通用

- UTF-8 编码；新文件默认 LF 换行，现有工程有明确要求时沿用。Windows 批处理 `.cmd/.bat` 用 CRLF。
- 每级缩进 4 个空格，不用 Tab；JSON、YAML、HTML、CSS、XML/XAML 用 2 个空格。Makefile 的命令缩进必须用 Tab，是例外。
- 一行一条语句；运算符两侧加空格，逗号后加空格；行尾不留空格，文件末尾保留一个换行。
- 新工程每行建议不超过 120 字符；方法间空一行，最多一行连续空行。长 URL、字符串等不为了限长改变内容。
- 不挤成单行分支；有大括号的语言，`if/else/for/while` 即使单条语句也写完整括号。
- 长参数列表按参数换行，每个参数一行；调用、运算和布尔表达式在自然边界换行，不压缩代码。
- 标识符用明确英文，不用拼音、随意缩写和 `data1/data2`；局部循环索引 `i/j` 可用。测量量值用 `Mm`、`Cm3`、`Px`、`Percent` 或对应蛇形后缀标明单位。
- 注释优先使用清楚的中文，保留必要英文术语；说明用途、单位和复杂部分的原因，不逐行翻译语句。

- 所有具名方法（含私有方法、构造函数、事件及局部函数）须有用途说明；复杂方法体说明关键步骤、约束与原因。各语言格式、例外与检查口径见 [方法与方法体注释](METHOD_COMMENTS.md)。

## 2. 命名与排版速查

| 项目 | C++ | C# | Java | TypeScript / JavaScript | Python | PowerShell |
| --- | --- | --- | --- | --- | --- | --- |
| 类型/类 | `VolumeResult` | `VolumeResult` | `VolumeResult` | `VolumeResult` | `VolumeResult` | 类用 `VolumeResult` |
| 函数/方法 | `calculateVolume` | `CalculateVolume` | `calculateVolume` | `calculateVolume` | `calculate_volume` | `Get-CellVolume` |
| 参数/局部变量 | `cellHeightMm` | `cellHeightMm` | `cellHeightMm` | `cellHeightMm` | `cell_height_mm` | `$cellHeightMm` |
| 私有实例字段 | `cellHeightMm_` | `_cellHeightMm` | `cellHeightMm` | `cellHeightMm` | `_cell_height_mm` | 按类型成员约定 |
| 常量 | `kMaxCellCount` | `MaxCellCount` | `MAX_CELL_COUNT` | `MAX_CELL_COUNT` | `MAX_CELL_COUNT` | 具名常量变量 |
| 枚举成员 | `Measured` | `Measured` | `MEASURED` | `Measured` | `MEASURED` | `Measured` |
| 接口命名 | 类型命名，无强制前缀 | `ICameraSource` | `CameraSource` | `CameraSource` | 不强制接口类 | 不强制接口类 |
| 大括号 | 独占一行 | 独占一行 | 起始括号同行 | 起始括号同行 | 使用冒号和缩进 | 起始括号同行 |
| 语句分号 | 保留 | 保留 | 保留 | 保留 | 不使用 | 不使用 |
| 公共接口注释 | Doxygen | XML 文档注释 | Javadoc | JSDoc / TSDoc | docstring | 注释帮助 |

本表规定新模块默认值；原生导出函数如 `tv_*`、第三方接口及既有模块风格优先保留。C# 常量不照搬 Java 全大写规则；Java 方法不照搬 C# 大写规则。

## 3. C++（现有算法使用 C++17）

- 头文件 `.h/.hpp`，实现文件 `.cpp`；文件名沿用工程的小写下划线风格，如 `volume_calculator.cpp`。
- 大括号采用 Allman 风格；`else` 独立一行。命名空间内也缩进 4 空格，访问修饰符与类左括号对齐。
- 指针/引用符号贴类型：`const Frame& frame`、`const float* depthMm`；每行只声明一个变量。
- `#include` 按本模块头文件、项目头文件、第三方头文件、标准库分组，组间空一行；既有顺序不自动调整。
- 公共接口用 `/** ... */`，适用时写 `@brief`、`@param`、`@return`；函数内用 `//`。

```cpp
/**
 * @brief 根据净尺寸计算矩形箱格容量。
 * @param lengthMm 内部净长，单位 mm。
 * @param widthMm 内部净宽，单位 mm。
 * @param heightMm 内部净高，单位 mm。
 * @return 容量，单位 cm³。调用方保证尺寸有效且为正。
 */
double calculateCapacityCm3(double lengthMm, double widthMm, double heightMm)
{
    constexpr double kCubicMillimetersPerCubicCentimeter = 1000.0;
    return lengthMm * widthMm * heightMm / kCubicMillimetersPerCubicCentimeter;
}
```

## 4. C#（现有容积应用保持 C# 5）

- 文件名与主要类型一致，如 `VolumeCalculator.cs`；类型、方法、属性 PascalCase，接口加 `I`。
- Allman 大括号；`using` 放在命名空间外，`System` 组在前，项目组在后；组间空一行。
- 顺序建议为字段、构造函数、属性/事件、公共方法、私有方法；相关成员靠近，不无故大范围重排旧类。
- 公共 API 使用 `///` XML 注释；复杂实现用 `//`。字符串使用双引号。
- 保留普通块形式 `namespace`、`using (...) { ... }`；格式示例不引入 C# 5 不支持的新语法。

```csharp
/// <summary>计算载货率，单位 %。</summary>
/// <param name="cargoVolumeCm3">货物体积，单位 cm³。</param>
/// <param name="capacityCm3">容量，单位 cm³，必须大于零。</param>
/// <returns>载货率，单位 %。</returns>
public double CalculateLoadPercent(double cargoVolumeCm3, double capacityCm3)
{
    if (capacityCm3 <= 0)
    {
        throw new ArgumentOutOfRangeException("capacityCm3");
    }

    return cargoVolumeCm3 / capacityCm3 * 100.0;
}
```

## 5. Java

- 文件名与公开类一致，如 `VolumeCalculator.java`；包名全小写，方法/变量 lowerCamelCase。
- 起始大括号同行；写成 `} else {`、`} catch (...) {`；控制关键字与左括号之间留空格，方法名与左括号之间不留空格。
- 常量大写下划线，字符串双引号；注释采用 Javadoc。导入按项目分组，不为格式化擅自改变依赖。

```java
/**
 * 计算载货率。
 * @param cargoVolumeCm3 货物体积，单位 cm³
 * @param capacityCm3 容量，单位 cm³，必须大于零
 * @return 载货率，单位 %
 */
public double calculateLoadPercent(double cargoVolumeCm3, double capacityCm3) {
    if (capacityCm3 <= 0) {
        throw new IllegalArgumentException("capacityCm3 must be positive");
    }

    return cargoVolumeCm3 / capacityCm3 * 100.0;
}
```

## 6. TypeScript / JavaScript / React

- 4 空格；起始大括号同行；语句分号保留；普通字符串单引号，JSX 属性双引号。
- 多行对象、数组和参数列表保留格式化器支持的尾逗号；箭头函数参数保留括号。
- 类型、接口、React 组件 PascalCase；函数和变量 camelCase；组件文件使用 `VolumePanel.tsx`，普通模块跟随既有目录风格。
- 导出接口用 JSDoc/TSDoc。方法间空一行；短对象可以同行，禁止手工对齐多列空格。
- Forge IDE 当前 `.prettierrc.json` 的 110 字符行宽继续有效；120 为新工程默认值。

```typescript
/** 计算载货率（%）；两个输入均使用 cm³，容量须大于零。 */
export function calculateLoadPercent(cargoVolumeCm3: number, capacityCm3: number): number {
    if (capacityCm3 <= 0) {
        throw new RangeError('capacityCm3 must be positive');
    }

    return (cargoVolumeCm3 / capacityCm3) * 100;
}
```

## 7. Python 与 PowerShell 工具脚本

Python：类 PascalCase、函数/变量 snake_case、常量全大写；顶层函数/类之间空两行，类内方法间空一行，这是通用空行规则的例外。注释用 `#`，接口说明用三双引号 docstring；普通字符串统一双引号，不写行尾分号。

```python
def calculate_load_percent(cargo_volume_cm3, capacity_cm3):
    """计算载货率（%）；输入为 cm³，容量必须大于零。"""
    if capacity_cm3 <= 0:
        raise ValueError("capacity_cm3 must be positive")

    return cargo_volume_cm3 / capacity_cm3 * 100.0
```

PowerShell：函数用标准动词加业务名，例如 `Get-CellVolume`；公开参数用 PascalCase、局部变量 camelCase；起始括号同行，运算符两侧空格。字面字符串单引号，需要变量插值时双引号；公共脚本用 `<# .SYNOPSIS / .PARAMETER / .EXAMPLE #>` 注释帮助。禁止复用 `$HOME` 等系统变量作为业务变量。

## 8. 配置与标记文件

- JSON：2 空格、双引号、无注释、无尾逗号；配置键保持已有协议，不因命名风格改变键名。
- YAML：2 空格，不使用 Tab，列表项用 `- `。
- HTML/XML/XAML：2 空格，属性值双引号；多属性长行拆行并统一缩进。
- CSS：2 空格，每个属性一行，冒号后一个空格，分号结尾；选择器及类名延续原工程。
- Markdown：标题/段落/列表间空一行，代码块标明语言，列表不手工用空格排表。

## 9. 编辑器配置模板

配置模板放在 [code-format](code-format/) 目录，可复制到新应用根目录。现有配置先合并差异，不直接覆盖；本次未对现有源代码运行批量格式化。

| 文件 | 管理范围 | 不负责 |
| --- | --- | --- |
| `.editorconfig` | 编码、缩进、行尾；支持相应属性的 C# 编辑器还可提示命名/括号 | 不保证所有编辑器支持扩展规则，不校验业务 |
| `.clang-format` | C++ 空格、大括号、换行、缩进 | 不自动重命名，不保证插入缺少的分支括号 |
| `.prettierrc.json` | JS/TS、JSON、HTML、CSS 等排版 | 默认不格式化 C#、Java、Python；不负责重命名 |
| `.prettierignore` | 排除依赖、构建和发布目录 | 不改变其他格式化器的扫描范围 |

120 字符是排版目标，不是所有工具都能强制的最大值。EditorConfig 的命名诊断依赖支持的 C# 工具；当前 Framework `csc.exe` 单独编译不代表执行了这些诊断。Java/Python/PowerShell 本次提供规则，没有安装格式化插件。

示例用于展示格式，不是完整数值校验实现，不应替换项目已有算法。需要修改旧代码时单独做格式变更，不夹带公式、阈值或接口调整。

## 2026-11-05 可维护性补充

项目、文件、类、方法、变量的语义命名及按语言维护要求，统一见 [READABLE_CODE.md](READABLE_CODE.md)。它补充现有格式和注释规则；既有公开接口和持久化契约保持兼容，旧代码按当前任务涉及模块迁移。
