# 阶段 2 学习计划：UR3 仿真控制入门

## 1. 计划目标
- 在 2 周内在 fake hardware 模式下打通 UR3 控制链路。
- 完成 3 个单功能学习任务，并同步沉淀 3 篇学习文档。
- 对齐阶段验收目标：让 UR3 模型执行关节轨迹、能解释控制器/驱动/MoveIt 三者分工。

## 2. 当前基线（来自仓库现状）
- 已完成：阶段 1 全部 8 个任务（4A–4H），包含 ros2_control 最小链路、joint_trajectory_controller 联调、控制器对比总结。
- 阶段 2 当前状态：UR 官方驱动 mock hardware、控制器体系理解、Python 最小控制闭环三项任务均已完成。

## 3. 任务拆分与进度跟踪

### 3.1 进度统计
- 已完成：`3 / 3`

### 3.2 可勾选任务清单
- [x] 任务 5A：跑通 UR ROS 2 Driver（mock hardware）— 完成于 2026-04-11
- [x] 任务 5B：理解控制器体系 — 完成于 2026-04-12
- [x] 任务 5C：最小控制闭环（Python）— 完成于 2026-04-12

## 4. 每个任务的规划文档目录
1. 5A -> `notes/plans/tasks/task5A_plan.md`
2. 5B -> `notes/plans/tasks/task5B_plan.md`
3. 5C -> `notes/plans/tasks/task5C_plan.md`

## 5. 每个任务的文档交付路径
1. 5A -> `notes/labs/task5A_ur_driver_fake_hardware.md`
2. 5B -> `notes/concepts/task5B_ur_controller_system.md`
3. 5C -> `notes/labs/task5C_minimal_control_node.md`

## 6. 执行约束
- 每次只做一个功能，完成后再进入下一项。
- 每完成一个任务，同步更新对应文档模板。
- 文档优先记录：目标、步骤、日志、结论、风险与下一步。

## 7. 阶段验收标准
- 能在 fake hardware 模式下让 UR3 模型执行一段关节轨迹。
- 能清楚解释"控制器、驱动、MoveIt"三者分别负责什么。

## 8. 当前结论
- 阶段 2 三个任务已全部完成。
- 已在 `ws_stage2` 新建 `ur3_minimal_control_lab_py`，完成最小控制闭环。
- 下一阶段若继续推进，建议单开新任务处理：
  - C++ 版 Action Client 迁移；
  - URSim 接入；
  - 更适合手术机器人场景的轨迹参数化与安全约束。
