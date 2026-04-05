# 阶段 1 下一步学习计划（含文档交付）

## 1. 计划目标
- 在 2-3 周内补齐 ROS 2 机械臂开发基础能力。
- 完成 8 个单功能学习任务，并同步沉淀 8 篇学习文档。
- 对齐阶段验收目标：独立建包、解释 action 适配性、读懂机械臂 description 包结构。

## 2. 当前基线（来自仓库现状）
- 已完成：最小 JointState 发布（Python）、FollowJointTrajectory Action Client（Python）、状态监控节点（C++）、简化 URDF + RViz 联动、Service 实战、QoS 对比实验。
- 待补齐：tf2 主动查询、ros2_control 最小控制链、控制器对比总结。

## 3. 任务拆分与进度跟踪

### 3.1 进度统计
- 已完成：`3 / 8`

### 3.2 可勾选任务清单
- [x] 任务 4A：Service 最小闭环
- [x] 任务 4B：QoS 对比实验
- [x] 任务 4C：tf2 主动查询
- [ ] 任务 4D：URDF 升级 xacro 参数化
- [ ] 任务 4E：ros2_control 最小链路
- [ ] 任务 4F：joint_trajectory_controller 联调
- [ ] 任务 4G：forward_command_controller 对比
- [ ] 任务 4H：阶段 1 验收复盘

## 4. 每个任务的规划文档目录
1. 4A -> `notes/plans/tasks/task4A_plan.md`
2. 4B -> `notes/plans/tasks/task4B_plan.md`
3. 4C -> `notes/plans/tasks/task4C_plan.md`
4. 4D -> `notes/plans/tasks/task4D_plan.md`
5. 4E -> `notes/plans/tasks/task4E_plan.md`
6. 4F -> `notes/plans/tasks/task4F_plan.md`
7. 4G -> `notes/plans/tasks/task4G_plan.md`
8. 4H -> `notes/plans/tasks/task4H_plan.md`

## 5. 每个任务的文档交付路径
1. 4A -> `notes/labs/task4A_service_vs_topic_action.md`
2. 4B -> `notes/labs/task4B_qos_experiment.md`
3. 4C -> `notes/labs/task4C_tf2_lookup.md`
4. 4D -> `notes/concepts/urdf_xacro_parameterization.md`
5. 4E -> `notes/concepts/ros2_control_minimal_chain.md`
6. 4F -> `notes/runbooks/follow_joint_trajectory_debug.md`
7. 4G -> `notes/labs/task4G_controller_comparison.md`
8. 4H -> `notes/reports/stage1_review.md`

## 6. 执行约束
- 每次只做一个功能，完成后再进入下一项。
- 每完成一个任务，同步更新对应文档模板。
- 文档优先记录：目标、步骤、日志、结论、风险与下一步。

## 7. 阶段验收标准
- 能独立创建并构建一个 ROS 2 package（Python 与 C++ 各至少一个）。
- 能清楚解释为何轨迹执行通常使用 Action 而非 Topic/Service。
- 能阅读并说明一套机械臂 description 包的结构与参数入口。
