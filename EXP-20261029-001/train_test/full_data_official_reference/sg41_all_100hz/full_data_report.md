# PiPER X 全量记录辨识结果索引

## 当前结论

本次已按正确顺序重做：**先由快照中的 PiPER X URDF 建立逆动力学回归器，生成结构最小参数集；再用真机训练周期拟合基参数；最后在留出周期上验证。** 旧的直接拟合 78 个标准参数并把 QR 解换算成逐连杆质量的结果已作废。

最小基关系为

\[
\tau=Y(q,\dot q,\ddot q)\theta,\qquad
Y_gS=U\Sigma[V_r\;V_0]^T,
\]

\[
\theta=S(V_r\beta+V_0\gamma),\qquad
Y_{\min}=YSV_r.
\]

本 PiPER X 模型有 60 个标准惯性参数；加入逐关节粘性摩擦、平滑库仑摩擦和偏置后共 78 个回归参数。按 URDF 通用关节状态和 CAD 惯性特征尺度生成的数值结构秩为 **54/78**，标准参数零空间为 **24 维**。因此数据拟合的是 54 个可辨识组合 \(\beta\)，它们不是逐 link 的质量、质心或惯量。

本次 SG41 训练集含 955 点、留出周期含 957 点。基参数训练误差为 **7.06%**，留出周期误差为 **7.08%**（残差范数/目标力矩范数）；全量 11,956 点重拟合误差 **7.33%** 是训练内误差。目标力矩使用 PiPER X SDK 的型号参考电流系数，未针对这台机械臂做台架标定，因此这些指标只说明该换算尺度和模型下的力矩拟合，不证明真实惯性参数已测出。

## 为什么之前会得到近零质量

此前直接对 78 维标准参数求秩亏最小二乘，QR 求解器会把未选中的方向置零。再把这个不唯一的标准参数向量解释成每个实体 link 的质量，就得到近零质量、COM 无法计算和非物理惯量。零质量是秩亏代表解的数值伪象，不是机械臂的物理结论。

旧的列 RMS 归一化还会放大有限差分回归器在结构零列中的舍入噪声，使秩被误报为 65。当前报告改用从 URDF 名义 CAD 参数得到的自然尺度 \(S\)：每 link 惯性尺度按 \(m_0\)、\(m_0\ell\)、\(m_0\ell^2\) 设置。三个通用状态批次和一个独立状态集均支持 54 维；保留与丢弃奇异值间隔约 \(2.3\times10^9\)，独立回归器重构误差为 \(4.2\times10^{-14}\)。

此外，单独质量坐标在可观测子空间中的投影范数（link1–6）约为 `1.7e-14, 7.8e-15, 0.0052, 0.0072, 0.0073, 0.0074`。这不是置信区间；它表明质量坐标几乎完全处于可由其他惯性分量补偿的结构零空间中，单靠本次关节力矩轨迹不能分别确定六个 link 的质量。

## 物理约束候选的含义

另有一组在最小基框架中重拟合的物理可行标准参数。它加入伪惯量半正定、URDF/CAD 质量界、PiPER X 碰撞网格 COM 与二阶矩几何界及 CAD 正则。该候选的留出误差为 **7.45%**，物理约束使基向量相对无约束 OLS 改变约 **71.8%**，且 link2、link4、link5、link6 有约束边界活跃。因此逐连杆数值是由先验和边界选择的一种可行实现，不是测得的真实质量，不能用于控制。

## 应查看的文件

- [URDF 最小基辨识报告](urdf_base_parameters/base_parameter_identification_report.md)：生成步骤、公式、秩、留出验证与限制。
- [URDF 最小基结果 JSON](urdf_base_parameters/base_parameter_identification_report.json)：机器可读来源、结构秩、训练/验证指标和质量可观测性审计。
- `urdf_base_parameters/urdf_structural_base_parameters.csv`：54 个最小基系数；这些不是 link 质量。
- `urdf_base_parameters/urdf_structural_base_directions.csv`：每个基方向对原 78 个参数的映射。
- [物理约束候选报告](physical_base_fit/physically_constrained_identification_report.md)：先验约束、候选逐连杆参数及活跃边界。
- [过期结果说明](STATUS_SUPERSEDED.md)：说明旧零质量 CSV/NPZ 的来源与失效原因。

旧文件 `full_data_parameter_estimates.csv`、`per_link_derived_inertial_parameters.csv`、`nominal_vs_full_fit_comparison.csv` 和 `full_data_fit.npz` 仅保留历史复算记录，**不得当作当前辨识结果或真实质量/惯量使用**。
