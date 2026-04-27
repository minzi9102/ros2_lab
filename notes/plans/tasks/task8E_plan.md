# 任务 8E 规划文档：异常场景与安全停机逻辑

## 1. 任务目标
- 在 8D 完成最小真机动作后，整理并验证异常情况下的拒绝执行、取消 goal、停止 program、人工恢复路径。
- 建立“失败先停、记录、人工判断”的异常处理矩阵。
- 明确哪些操作永远不做默认自动化，例如自动解锁 protective stop。

## 2. 当前基线（来自仓库现状）
- 8C 提供状态门闩。
- 8D 提供受限动作入口和执行日志。
- 阶段 2/3 已有 URSim 排障经验，但真实机器人异常处理需要更保守的规则。

## 3. 任务范围（单功能约束）
- 包含：
  - 目标越界、状态门闩失败、controller inactive、External Control 停止等低风险异常验证；
  - Action goal cancel 或拒绝执行路径；
  - 停止 program 的人工确认流程；
  - 异常处理矩阵。
- 不包含：
  - 故意制造高风险碰撞或大范围运动；
  - 自动 unlock protective stop；
  - 自动 restart safety；
  - 自动 power cycle；
  - 无人值守恢复。

## 4. 包与文档关系
- 可复用 `ur3_real_bringup_lab` 的状态检查。
- 可复用 `ur3_real_guarded_motion_lab_cpp` 的拒绝执行与 cancel 逻辑。
- 本任务优先沉淀文档和测试矩阵，只有必要时补代码。

## 5. learn mode 分工
- 智能体负责：
  - 异常处理矩阵；
  - 日志字段补齐；
  - cancel / reject / stop-program 的接口边界说明；
  - 低风险异常的验证脚本骨架。
- 人类负责：
  - 判断哪些异常可以现场验证；
  - 执行任何示教器或 Dashboard 恢复操作；
  - 决定是否停止阶段测试；
  - 对 protective stop 等安全事件进行现场复盘。

## 6. 核心学习问题
1. 目标越界、controller inactive、protective stop 分别应触发什么处理路径？
2. 为什么“不发送 goal”通常比“发送后再 cancel”更安全？
3. 什么时候应该取消 Action goal，什么时候应该停 External Control program？
4. 为什么恢复类操作必须保留人工确认？

## 7. 实施步骤

### 练习 1：整理异常分类
1. 将异常分成执行前、执行中、执行后。
2. 每类异常定义检测方式、处理动作、日志字段、人工确认要求。
3. 将不确定项标为 `block`。

### 练习 2：验证执行前拒绝路径
1. 使用越界目标或过大 delta 触发拒绝执行。
2. 使用 controller inactive 或状态门闩失败场景触发拒绝执行。
3. 确认没有 Action goal 被发送。

### 练习 3：验证可控 cancel 路径
1. 只在低速、小范围、安全动作中测试 cancel。
2. 记录 cancel 请求、controller result、最终 joint state。
3. 如果 cancel 行为不可解释，停止继续验证。

### 练习 4：整理人工恢复 runbook
1. 记录 protective stop、safeguard stop、program stopped 等场景的人工处理路径。
2. 明确哪些服务只能人工触发，不能被实验脚本自动调用。
3. 形成 8F 阶段验收前的异常处理检查表。

## 8. 关键命令（执行阶段使用）
```bash
# 状态查询
ros2 control list_controllers
ros2 service call /dashboard_client/get_robot_mode ur_dashboard_msgs/srv/GetRobotMode "{}"
ros2 service call /dashboard_client/get_safety_mode ur_dashboard_msgs/srv/GetSafetyMode "{}"

# 动作入口应先 dry-run 验证拒绝路径
ros2 launch ur3_real_guarded_motion_lab_cpp task8D_guarded_home_ready.launch.py \
  execute:=false \
  target_name:=out_of_range_test
```

> Dashboard 恢复类服务只作为人工 runbook 记录对象，不作为本任务默认自动调用对象。

## 9. 异常处理矩阵草案

| 异常 | 检测阶段 | 默认动作 | 是否人工确认 | 是否允许自动恢复 |
|---|---|---|---|---|
| 目标越界 | 执行前 | 拒绝执行 | 否 | 否 |
| delta 过大 | 执行前 | 拒绝执行 | 否 | 否 |
| `/joint_states` 过期 | 执行前 | 拒绝执行 | 否 | 否 |
| controller inactive | 执行前 | 拒绝执行并记录 | 是 | 否 |
| External Control 未运行 | 执行前 | 拒绝执行 | 是 | 否 |
| Action 执行异常 | 执行中 | cancel goal / 停止观察 | 是 | 否 |
| protective stop | 任意阶段 | 停止脚本，人工处理 | 是 | 否 |
| driver 断连 | 任意阶段 | 停止脚本，记录日志 | 是 | 否 |

## 10. 交付物
- 文档：`notes/labs/task8E_safe_stop_and_fault_handling.md`
- 可选代码：补强 `ur3_real_guarded_motion_lab_cpp` 的 reject / cancel 日志。
- 结果：异常处理矩阵和人工恢复 runbook。

## 11. 验收标准
- 能演示至少一种执行前拒绝路径，且不发送 goal。
- 能说明 cancel goal、停止 program、急停、保护停止的边界。
- protective stop 等恢复操作不会被默认自动化。
- 每个异常都有日志记录字段和下一步处理建议。

## 12. 风险与回退
- 风险 1：为验证异常而制造高风险现场状态。
  - 回退：只验证低风险软件拒绝路径，高风险场景写 runbook 不实测。
- 风险 2：把恢复服务做进自动流程。
  - 回退：删除自动调用，只保留人工确认说明。
- 风险 3：cancel 测试不可控。
  - 回退：停止动作测试，回到 8D 低速目标复核。
