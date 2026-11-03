# PiPER X 几何约束网格来源

这里的碰撞网格用于给物理参数恢复提供 COM 支撑范围和二阶矩外界，不用于替代惯性辨识或推断内部质量分布。

- 来源仓库：[agilexrobotics/agx_arm_urdf](https://github.com/agilexrobotics/agx_arm_urdf)
- 固定来源提交：`680fa925cd00db7e2628563d2edc5540963045de`
- 依据快照 URDF/Xacro 中的 PiPER X 网格引用复制 `piper_x` 连杆、法兰及夹爪 STL。
- `link6` 约束使用 link6、法兰和夹爪网格的并集；夹爪两指覆盖 Xacro 滑动关节范围。
- 网格凸包向外扩展 10 mm；伪惯量二阶矩约束使用同一几何的外扩 AABB。外扩用于包容碰撞几何近似和装配差异，不表示实际质量分布。
- 文件的 SHA-256、具体几何变换、边界和限制由 `train_test/piper_x_geometry.py` 写入 `physical_base_fit/piper_x_geometry_constraints.json`。

这些 STL 源自 AgileX 仓库；请遵循该仓库及各文件随附的许可证要求。
