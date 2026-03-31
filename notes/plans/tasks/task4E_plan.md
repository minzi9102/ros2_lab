# 任务 4E 规划文档：ros2_control 最小链路

## 1. 任务目标
- 跑通 `controller_manager + joint_state_broadcaster` 最小闭环。
- 理解 read / update / write 控制循环在系统中的位置。

## 2. 任务范围
- 包含：ros2_control 配置、controller_manager 启动、控制器状态检查。
- 不包含：真机驱动接入、实时内核调优。

## 3. 前置条件
- 已有可发布 joint_states 的模型与仿真基础。
- 具备基础 controller YAML 配置经验。

## 4. 实施步骤
1. 在模型中补充 ros2_control 相关描述。
2. 编写 controllers.yaml（先 broadcaster，再轨迹控制）。
3. 编写启动链路并验证节点启动顺序。
4. 使用 `ros2 control` 命令检查控制器状态。
5. 记录最小可用配置与排错经验。

## 5. 交付物
- 代码：最小 ros2_control 练习包与配置。
- 文档：`notes/concepts/ros2_control_minimal_chain.md`。

## 6. 验收标准
- 控制器可加载并处于预期状态。
- 能解释 manager、broadcaster、controller 的职责边界。

## 7. 风险与回退
- 风险：硬件接口配置不一致导致控制器无法激活。
- 回退：先仅启用 broadcaster 验证状态流，再加执行控制器。

## 8. 完成记录
- 状态：`[ ] 未开始` `[#] 进行中` `[x] 已完成`
- 日期：
- 备注：
