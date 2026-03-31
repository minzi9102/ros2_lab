# 任务 4D 规划文档：URDF 升级 xacro 参数化

## 1. 任务目标
- 将简化 URDF 改造为可参数化 xacro 结构。
- 建立“模型结构 + 配置参数”分离的维护方式。

## 2. 任务范围
- 包含：xacro 宏拆分、参数入口、launch 接入。
- 不包含：高精度动力学模型重建、复杂碰撞建模。

## 3. 前置条件
- 已阅读 `ur.urdf.xacro` 与 `ur_macro.xacro` 的参数模式。
- 已理解 `joint_limits.yaml` 的关节约束含义。

## 4. 实施步骤
1. 把单体 URDF 拆为 `*.urdf.xacro` 与参数定义。
2. 提炼关节限位与几何参数入口。
3. 更新 launch：从 xacro 动态生成 robot_description。
4. 在 RViz 验证模型加载和关节联动。
5. 整理参数映射关系与命名规范。

## 5. 交付物
- 代码：参数化 xacro 与配套 launch。
- 文档：`notes/concepts/urdf_xacro_parameterization.md`。

## 6. 验收标准
- 能通过参数切换影响模型或限位行为。
- 能说明参数从 launch 到模型生效的路径。

## 7. 风险与回退
- 风险：宏层级过深，调试难度上升。
- 回退：先单宏实现，再逐步拆分细化。

## 8. 完成记录
- 状态：`[ ] 未开始` `[#] 进行中` `[x] 已完成`
- 日期：
- 备注：
