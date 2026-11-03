# 机器人臂动力学辨识方法调研与 PiPER X 重算

## 调研结论

GitHub 上找到两类更直接相关的 PiPER 项目。第一类是明确支持 PiPER 六轴模型的 [Robot-Parameter-Indentification-Simulation](https://github.com/windiff886/Robot-Parameter-Indentification-Simulation)：它提供 MuJoCo PiPER 模型、Piper real backend、傅里叶激励和 OLS/WLS/IRLS 代码；但手册说明其当前辨识对全标准惯性向量拟合，不生成最小基参数集，也没有逐连杆物理半正定约束。仓库的 `results/identification.yaml` 是 benchmark 结果，`tests/piper_real` 使用模拟适配器；没有找到能证明其中参数来自真实 PiPER 称重/力矩标定的可审计记录，因此它适合参考采集和算法工程，不适合拿其参数当真机质量。

第二类是 [iit-DLSLab/sim2real-robot-identification](https://github.com/iit-DLSLab/sim2real-robot-identification)，其中机型是 **PiPER-L，不是 PiPER X**，仓库包含 Piper-L 模型和一条采集数据。它的 MuJoCo SysID 代码支持 `--identify-inertia-tensor`，用 MuJoCo `InertiaType.Pseudo` 表示完整惯性，并为质量、COM 和惯量伸缩/剪切设置相对 CAD 的范围；拟合损失直接比较整段仿真 rollout 的关节位置/速度，可同时识别 PD、摩擦、armature 和时延。这提供了值得借鉴的物理参数化、有限范围和轨迹级验证方法。不过仓库把 `sysid_mujoco/results/` 加入 gitignore，公开的 `isaaclab_identified_parameters.py` 是 armature、摩擦和 delay 等执行器参数，不是已发布的逐连杆惯性辨识结果；README 中的 “Piper-L identified” 不能单独作为真实质量正确的证据。我们这台 PiPER X 当前采集数据也缺少已核准的执行器增益/输入模型，故没有直接套用其 rollout 目标。

更成熟的通用方法参考是 [BIRDy](https://github.com/TUM-ICS/BIRDy) 和 [FloBaRoID](https://github.com/kjyv/FloBaRoID)：BIRDy 对比 OLS/WLS/IRLS、IV、直接/间接辨识与物理一致性变体，并强调从模型生成基参数和设计激励；FloBaRoID 实现了伪惯量半正定约束、基参数到标准参数的 CAD 邻近恢复、可观测性加权和几何先验。需要留意 FloBaRoID 当前 `sdp.py` 仍声明 `identifyFeasibleBaseParameters` 的 CVXPY 后端尚未实现；其固定基参数后恢复标准参数的路径是已实现的。它不会替代对本机数据、坐标和尺度的审计。

此前出现零质量的直接原因是：对 78 个标准参数做秩亏最小二乘后，把 QR 未选方向置零，再将这个数值解换算成 link 质量。零质量不是机械臂没有质量，而是不可辨识方向被求解器置零。更进一步的代码复核发现，第一版“最小基”的列 RMS 归一化也不稳妥：MuJoCo 有限差分在结构零列留下约 (10^{-13}) 的舍入噪声，逐列除以这些微小 RMS 会把噪声放大，导致把秩报成 65，而可靠的 CAD 尺度审计给出 54。第一版 65 维结果已被本报告当前版本取代。

## URDF 派生的最小基参数

快照 PiPER X 模型的逆动力学线性回归写为

\[
\tau=Y(q,\dot q,\ddot q)\theta,\qquad \theta\in\mathbb R^{78},
\]

其中 60 个标准刚体惯性参数加 18 个逐关节摩擦/偏置参数。用快照 URDF 与名义 CAD 惯性构造特征尺度矩阵 (S)：每个 link 的尺度按 (m_0, m_0\ell, m_0\ell^2) 设置，(\ell) 由 CAD 质心惯量估计；摩擦列用固定的逐关节特征尺度。用 URDF 关节范围内的通用状态构造 (Y_g)，再对

\[
Y_g S=U\Sigma[V_r\;V_0]^T
\]

做 SVD。得到 36 个刚体惯性基方向、18 个摩擦/偏置方向，共 54 维最小基；剩余 24 维是标准参数零空间。独立随机状态验证的重构误差为 (4.2\times10^{-14})，三个通用状态种子都得到 54/78 的相同秩；第 54 和第 55 个奇异值相差约 (2.3\times10^9) 倍。此处 SVD 是对指定 PiPER X 回归器的数值结构审计，不是对所有机构的符号证明。

坐标映射为

\[
\theta=S(V_r\beta+V_0\gamma),\qquad
Y_{\min}=YSV_r,\qquad
\beta=V_r^TS^{-1}\theta.
\]

数据预测只依赖 (\beta)，不依赖 (\gamma)。因此数据直接支持的是 54 个最小基系数，不是逐 link 质量、COM 和惯量。

结构审计进一步显示，单独质量坐标投影到可观测子空间的范数，link1–6 依次约为 (1.7\times10^{-14}, 7.8\times10^{-15}, 0.0052, 0.0072, 0.0073, 0.0074)。这些是 CAD 特征尺度下的结构投影量，不是统计置信区间：link1 与 link2 的质量回归列是结构零列；其余质量坐标也几乎完全落在可与一阶矩和惯量互相补偿的零空间里。**所以当前力矩数据即使拟合得很好，也不能单独确定六个 link 的质量。**

## 拟合与物理可行性恢复

第一步在 54 维基上对真机训练周期拟合：

\[
\hat\beta=\arg\min_\beta\|W(YSV_r\beta-\tau)\|_2^2.
\]

完整周期留出误差为 7.08%；用全量 11,956 点重拟合的训练内误差为 7.33%。这是当前运动学、力矩换算和固定基座假设下的力矩预测结果，不是逐 link 惯性参数的成功证明。

第二步只为获得物理可行的标准参数候选，联合优化 (\beta) 与零空间实现 (\gamma)：

\[
\min_{\theta}\;\frac{1}{N}\|W(YSV_rV_r^TS^{-1}\theta-\tau)\|_2^2
+\lambda\,D_{\mathrm{CAD}}(P,P_{\mathrm{CAD}}),
\]

约束包括：每个 link 的伪惯量 (P_i\succeq0)；质量限定在 CAD 的 0.5–1.5 倍；COM 位于官方 PiPER X 碰撞网格凸包外扩 10 mm 内；伪惯量二阶矩对角项受网格外包 AABB 限制；黏性和光滑库仑摩擦非负。网格约束是几何外界，不描述内部真实密度。

这项恢复不是固定第一步的 (\hat\beta)：固定它时，标准惯性参数在伪惯量半正定约束下不可行，因此约束优化会重拟合基参数。重拟合后的留出误差为 7.45%，全量训练内误差为 7.66%，但 54 维训练基向量相对无约束 OLS 改变 71.8%。候选质量为 link1–6：0.709、1.385、0.908、0.585、0.525、0.278 kg；link4/5 触及质量上界，link6 触及下界，link2 的网格约束接近活跃。它是一组满足指定约束的 CAD 先验选解，不能称为已辨识真机质量。

## 鲁棒拟合对照

额外在同一 955/957 个训练/留出点和同一 54 维最小基上比较普通最小二乘、按关节目标 RMS 归一化的加权最小二乘，以及目标 RMS 归一化后按关节残差 MAD 做 Huber IRLS。留出结果如下；宏平均为六轴各自 `RMSE / target RMS` 的算术平均：

| 拟合损失 | 整体残差范数比 | 宏平均逐关节相对 RMSE |
|---|---:|---:|
| 普通最小二乘 | 7.076% | 20.58% |
| 目标 RMS 加权最小二乘 | 7.345% | 17.29% |
| 目标 RMS 加权 Huber IRLS | 7.370% | 17.47% |

Huber IRLS 按逐关节 MAD 尺度迭代 34 次后收敛。加权损失改善了低力矩轴在宏平均中的占比，但使整体误差从 7.08% 增至约 7.35%。Huber 相对普通加权最小二乘没有改善整体或宏平均验证指标；J4 稍有改善，J6 稍有变差，其他关节也有小幅权衡。因此当前不把 Huber 当作主结果，也不以增加迭代次数作为提高辨识真实性的手段。可用 `compare_robust_base_estimators.py` 重算，逐关节数值见 `full_data_official_reference/sg41_all_100hz/robust_loss_comparison.json`。若后续新采集明确出现离群点，应另做异常点审计、按周期验证和独立会话确认后，再考虑鲁棒损失。

## 结果应如何判读

- **最小基参数拟合**回答：当前模型和记录能否预测留出周期力矩。
- **物理约束候选**回答：在额外 CAD、网格和惯性约束下，能否找到一组非零、物理可行的标准参数解释相近的预测。
- **当前不能回答**：link1–6 的真实质量、质心和惯量各是多少。质量方向大部分不可辨识，物理候选还明显受先验边界影响。
- 旧零质量表和旧 65 维最小基结果都已被标记为过期，不用于物理结论或控制。
- 当前电流到力矩比例没有对这台机器台架标定；采集时底座/桌面移动未测量；关节零点和方向也没有独立核准。这些误差会被模型吸收到等效参数中。

要获得可靠的实体质量，下一步应优先称量连杆与夹爪、校准本机输出力矩/电流关系、固定并测量基座、校准关节零位，然后设计多频激励并检查基回归器信息矩阵的最小特征值/条件数；训练/测试应按完整周期或独立会话切分。若进一步确认位置命令和伺服增益，可借鉴 Piper-L 项目做轨迹级 rollout 拟合，以 q/qd 残差避免直接对噪声 qd 求二阶导；但要先把真机控制输入、延迟和电流扭矩链建模正确。当前 120 秒数据缺少执行器参数核验，故这一步不能仅靠增加优化迭代次数可靠完成。

## 公开参考

- [PiPER 模型、真机采集与辨识入口](https://github.com/windiff886/Robot-Parameter-Indentification-Simulation)
- [该项目的 IRLS 与其他算法实现](https://github.com/windiff886/Robot-Parameter-Indentification-Simulation/blob/main/src/identification/src/algorithms.cpp)
- [该项目的 benchmark 结果文件](https://github.com/windiff886/Robot-Parameter-Indentification-Simulation/blob/main/results/identification.yaml)
- [PiPER-L 真机/仿真系统辨识项目](https://github.com/iit-DLSLab/sim2real-robot-identification)
- [仓库中的 PiPER-L 轨迹数据](https://github.com/iit-DLSLab/sim2real-robot-identification/blob/main/datasets/piper_l/trajectory_1.pt)
- [PiPER-L MuJoCo SysID：位置/速度 rollout、质量/COM/伪惯量选项](https://github.com/iit-DLSLab/sim2real-robot-identification/blob/main/sysid_mujoco/my_fit.py)
- [该项目的惯性参数辨识模式与物理一致性说明](https://github.com/iit-DLSLab/sim2real-robot-identification/blob/main/sysid_mujoco/README.md)
- [MuJoCo SysID inertial parameter bounds and pseudo-inertia setup](https://github.com/iit-DLSLab/sim2real-robot-identification/blob/main/sysid_mujoco/common.py)
- [PiPER-L identified actuator parameters](https://github.com/iit-DLSLab/sim2real-robot-identification/blob/main/robot_model/piper_l/isaaclab_identified_parameters.py)
- [该仓库忽略本地生成的拟合结果目录](https://github.com/iit-DLSLab/sim2real-robot-identification/blob/main/.gitignore)
- [BIRDy：机器人动力学辨识算法基准](https://github.com/TUM-ICS/BIRDy)
- [FloBaRoID：物理一致性 SDP 实现](https://github.com/kjyv/FloBaRoID/blob/master/identification/sdp.py)
- [Sousa 与 Cortesão：惯性参数物理一致性半正定约束](https://journals.sagepub.com/doi/10.1177/0278364913514870)
