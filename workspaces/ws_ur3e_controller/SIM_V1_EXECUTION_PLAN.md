# ws_ur3e_controller 仿真 v1 执行计划

本文档定义 `ws_ur3e_controller` 下一阶段的仿真优先计划。目标是在 fake hardware / MoveIt 仿真链路中先完成主要功能闭环，再把经过验证的最小能力迁移到真机。

## 1. 阶段目标

仿真 v1 只追求一个小而完整的闭环：

- 启动仿真 bringup。
- 通过 service 请求命名目标。
- 支持 `home` 和 `ready` 两个目标的 plan-only。
- 支持 `home` 和 `ready` 两个目标的单次 execute。
- 执行后能通过 final-target gate。
- 用 launch + service 级测试覆盖主要成功路径和关键拒绝路径。

不在本阶段扩展连续控制、目标队列、自动恢复、Action cancel、UI 或批量目标管理。

## 2. YAGNI / KISS 原则

本阶段遵守以下约束：

- 只保留 `home` 和 `ready` 两个仿真目标。
- `ready` 目标应比当前配置有更明显的运动幅度，用于观察规划和执行效果，但仍控制在真机可接受范围内。
- 不新增 catalog schema 字段，除非现有字段无法表达需求。
- 不新增真机专用门闩。
- 不删除已有真机门闩代码，但仿真 v1 不围绕它们做复杂测试。
- 优先使用现有 service 接口和 launch 文件，不引入新的交互协议。
- 每次只执行一个命名目标，不做队列、重试或自动串联。

## 3. final-target gate 说明

final-target gate 是执行后的最终到位检查。

控制器调用 MoveIt `execute(plan)` 后，并不只相信 `execute()` 的返回值，还会继续读取 `/joint_states`，比较当前各关节位置和目标 `positions_rad` 的误差。如果所有关节误差都小于 `final_position_tolerance_rad`，并且在 `final_state_timeout_sec` 内完成，就认为最终到位通过。

它解决的问题是：轨迹执行调用已经返回成功，但机器人或仿真状态没有真正停在目标点附近。

仿真 v1 保留 final-target gate，但保持参数简单：

- 不新增额外状态机。
- 不做自动重试。
- 如果 fake hardware 下偶发不稳定，优先调整仿真容差或超时，而不是增加复杂逻辑。

## 4. 实施步骤

### 4.1 整理仿真入口

目标：

- 让启动、调用、预期响应有一条固定路径。

预计修改：

- `README.md` 或现有功能定义文档。
- 必要时补充 `sim_named_motion_bringup.launch.py` 的参数说明。

验收：

- 从干净终端按文档执行，可以启动仿真控制器。
- `home` 和 `ready` 的 plan-only 请求返回 `status=planned`。

### 4.2 调整 `sim.ready` 目标幅度

目标：

- 保持 catalog 只有 `home` 和 `ready`。
- 让 `ready` 相对 `home` 有更明显但仍适合仿真、并且后续有机会迁移到真机的关节位移。

预计修改：

- `src/ur3e_named_motion_controller_cpp/config/ur3e_named_targets.yaml`
- `src/ur3e_named_motion_controller_cpp/test/test_target_catalog.py`

人工判断点：

- `ready` 应主要移动哪些关节。
- 单关节最大位移是否仍受 `sim.max_joint_delta_rad` 约束。
- 目标姿态是否在真机可接受范围内，避免只为仿真演示设计夸张姿态。

验收：

- catalog 测试通过。
- `ready` 规划成功。
- `ready` execute 后 final-target gate 通过。

### 4.3 建立 launch + service 级测试

目标：

- 启动完整 MoveIt 仿真，用接近真实使用方式的测试验证 service 行为。

优先覆盖：

- `home` plan-only 成功。
- `ready` plan-only 成功。
- `execute=true` 且 launch `execute=false` 时返回 `rejected_execution_disabled`。
- 未知目标返回 `rejected_unknown_target`。

暂缓覆盖：

- Dashboard mock。
- speed scaling mock。
- Remote Control mock。
- 真机 safety mode 分支。

验收：

- `colcon test --packages-select ur3e_named_motion_controller_cpp` 能运行新增测试。
- 失败时日志能看出 service 返回的 `status` 和 `message`。

### 4.4 覆盖仿真关键拒绝路径

目标：

- 只测试仿真阶段真正会影响主流程的问题。

优先覆盖：

- joint state 缺失时返回 `rejected_joint_state`。
- joint state 关节名不完整时返回 `rejected_joint_state`。
- 当前状态到目标 delta 超限时返回 `rejected_delta`。
- disabled target 返回 `rejected_disabled_target`。

设计约束：

- 测试入口应优先复用 `sim_named_motion_bringup.launch.py`，避免维护第二套仿真启动流程。
- 如果测试需要临时 catalog，优先使用测试专用 YAML 文件。
- 不为了测试引入生产代码中的复杂测试开关。

验收：

- 拒绝路径都有明确 `status`。
- 测试不依赖真机服务。

### 4.5 完成仿真 execute 验收

目标：

- 在 fake hardware 下完成 `home` 和 `ready` 的单次执行。

验收：

- `home` execute 返回 `status=executed`。
- `ready` execute 返回 `status=executed`。
- 两者都通过 final-target gate。
- 执行过程不需要人工确认 token。

## 5. 暂不做的事项

本阶段明确不做：

- 新增第三个命名目标。
- 连续 Servo 控制。
- waypoint 队列。
- 自动重试。
- 自动恢复 Dashboard / protective stop。
- Action cancel。
- 完整真机门闩 mock 测试。
- 真机一键 bringup。
- 复杂参数外显。

## 6. 真机迁移原则

真机迁移放在仿真 v1 稳定之后，作为单独阶段处理。

迁移时：

- 不删除当前 Dashboard / speed scaling 相关代码。
- 先按最低必需路径执行，不把所有门闩都作为第一轮迁移目标。
- 先迁移 `home`，再考虑 `ready`。
- 每次真实执行前先 plan-only。
- 每次真实执行只发一个 service 请求。

最低必需检查暂定为：

- MoveIt 语义层在线。
- controller active。
- `/joint_states` 可用且关节名完整。
- 目标在 real catalog 中已人工复核并启用。
- 执行请求带人工确认 token。

Dashboard、External Control、Remote Control、speed scaling 是否继续作为硬性门闩，等仿真 v1 稳定后再根据实际风险决定。

## 7. 已确认决策

- `sim.ready` 的新目标值选择真机可接受范围内的稍大范围运动。
- launch + service 测试启动完整 MoveIt 仿真。
- final-target gate 的仿真容差继续使用 `0.03` rad。
- 文档需要把真机已启用的 `home` / `ready` 状态和旧 TODO 对齐。

## 8. 推荐下一步

下一步先执行 4.2：

1. 人类确定 `sim.ready` 想主要移动的关节和大致幅度。
2. 智能体修改 catalog 和 catalog 测试。
3. 启动完整 MoveIt 仿真，确认 `ready` plan-only 和 execute 均通过。

完成后再进入 launch + service 级测试建设。
