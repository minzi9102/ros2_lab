# 阶段 3 学习计划：UR3 + MoveIt 2 运动规划

## 1. 计划目标
- 在 `2.5` 周内从“会发轨迹”升级到“会做运动规划”。
- 基于官方 `ur_moveit_config` 跑通 UR3 的 MoveIt 2 规划链路，而不是在本阶段自建一套 MoveIt 配置。
- 完成 `5` 个单功能学习任务，并为每个任务准备独立的任务计划、代码骨架和实验记录入口。
- 对齐阶段验收目标：独立写出一个 MoveIt 2 C++ 控制节点，解释“规划”和“控制”的差异，并处理桌面级别的简单碰撞约束。

## 2. 阶段启动基线（来自当时仓库现状）
- 已完成：阶段 1 全部任务（4A–4H），具备 ROS 2 基础通信、tf2、controller、Action 的最小经验。
- 已完成：阶段 2 核心任务（5A–5C、6A、6B），已打通：
  - `ur_robot_driver` fake hardware 链路；
  - `FollowJointTrajectory` Python / C++ 最小闭环；
  - URSim speed scaling 真实观测与执行节奏验证。
- 阶段启动时仓库还没有：
  - `ws_stage3` MoveIt 2 专用工作区；
  - 自定义 MoveIt 2 C++ 练习包；
  - 阶段 3 的计划文档与实验记录模板。
- 本机环境已具备：
  - `ur_moveit_config`
  - `moveit_ros_planning_interface`
  - `moveit_servo`
  - `ompl`

## 3. 阶段策略
- 主线环境：`先 fake hardware，后 URSim`
- 配置策略：复用官方 `ur_moveit_config`
- 规划组默认值：`ur_manipulator`
- 末端参考框架默认值：`tool0`
- 执行控制器：
  - `7A`–`7D` 使用 `scaled_joint_trajectory_controller`
  - `7E` 使用 `forward_position_controller`
- 本阶段不包含：
  - MoveIt Setup Assistant 自建配置；
  - Pick and Place；
  - MoveIt Task Constructor；
  - 自定义夹爪或末端执行器建模；
  - Servo 的 URSim 必做验收。

## 4. 任务拆分与进度跟踪

### 4.1 进度统计
- 已准备骨架：`5 / 5`
- 已完成实验：`5 / 5`
- 当前状态：阶段 3 已完成；`7A`–`7E` 均已有独立包、实验记录与主路径验收证据

### 4.2 可勾选任务清单
- [x] 任务 7A：MoveIt 2 bringup 与 RViz Quickstart
- [x] 任务 7B：C++ MoveGroupInterface 最小规划节点
- [x] 任务 7C：Planning Scene、Collision Object 与 Cartesian Path
- [x] 任务 7D：点击目标位姿 -> 自动规划并执行
- [x] 任务 7E：MoveIt Servo 连续控制入门

## 5. 每个任务的规划文档目录
1. `7A` -> `notes/plans/tasks/task7A_plan.md`
2. `7B` -> `notes/plans/tasks/task7B_plan.md`
3. `7C` -> `notes/plans/tasks/task7C_plan.md`
4. `7D` -> `notes/plans/tasks/task7D_plan.md`
5. `7E` -> `notes/plans/tasks/task7E_plan.md`

## 6. 每个任务的实验记录文档目录
1. `7A` -> `notes/labs/task7A_moveit_quickstart.md`
2. `7B` -> `notes/labs/task7B_move_group_interface_cpp.md`
3. `7C` -> `notes/labs/task7C_planning_scene_collision.md`
4. `7D` -> `notes/labs/task7D_goal_pose_auto_plan.md`
5. `7E` -> `notes/labs/task7E_moveit_servo.md`

## 7. 工作区与包目录
- 工作区：`workspaces/ws_stage3`
- 计划中的阶段 3 包：
  - `ur3_moveit_bringup_lab`
  - `ur3_moveit_move_group_lab_cpp`
  - `ur3_moveit_scene_lab_cpp`
  - `ur3_moveit_goal_pose_lab_cpp`
  - `ur3_moveit_servo_lab_cpp`

## 8. 推荐节奏（2.5 周）
- 第 1 周：
  - `7A` 跑通 MoveIt bringup 与 RViz Quickstart
  - `7B` 写出第一个 MoveGroupInterface C++ 节点
- 第 2 周：
  - `7C` 理解 Planning Scene / Collision Object / Cartesian Path
  - `7D` 完成点击目标位姿自动规划 demo
- 第 3 周前半：
  - `7E` 跑通 MoveIt Servo 最小速度链路
  - 完成阶段 3 复盘与验收

## 9. learn mode 分工原则
- 智能体负责：
  - 工作区与包骨架；
  - launch / 参数 / 配置胶水；
  - 文档模板、记录入口、构建验证；
  - 与现有仓库结构对齐的最小可运行框架。
- 人类负责：
  - 关键目标设定与规划语义判断；
  - 关节目标 / 位姿目标 / 笛卡尔路径的取舍；
  - 桌面碰撞体尺寸与抓取点位设计；
  - `goal_pose` 到抓取姿态的映射策略；
  - Servo 速度上限、停止策略与状态解释。

## 10. 执行约束
- 每次只推进一个子任务，完成后再进入下一个任务。
- 每个子任务都保持独立应用包，避免把多个学习目标塞进同一个包。
- 先构建骨架，再做联调；先 fake hardware，再决定是否进入 URSim。
- 文档优先记录：
  - 目标
  - 启动命令
  - 观察矩阵
  - 失败现象
  - 解释与结论

## 11. 阶段验收标准
- 能独立写出一个 MoveIt 2 C++ 控制节点。
- 能清楚解释“规划”和“控制”不是一回事。
- 能处理简单的桌面碰撞约束。
- 能完成一次基于 RViz 目标输入的自动规划并执行。
- 能说明 MoveGroup 离线路径规划与 MoveIt Servo 连续速度控制的边界。

## 12. 当前结论
- 阶段 3 已完成 `7A`、`7B`、`7C`、`7D`、`7E` 主路径，完成日期记录为 2026-04-27。
- 阶段验收标准已覆盖：MoveIt 2 C++ 控制节点、规划/控制边界、桌面碰撞约束、RViz 目标输入自动规划执行、MoveGroup 离线路径规划与 MoveIt Servo 连续速度控制边界。
- `7A` 已在 fake hardware 下完成 MoveIt bringup、RViz Quickstart、joint goal / pose goal 规划与执行。
- `7B` 已验证 `joint goal`、`pose goal`、`plan_only`、`plan_and_execute`，成功路径会自动退出；`ros2 launch` 主路径下的 kinematics warning 已通过参数注入收敛。
- `7C` 已验证 Planning Scene 桌面碰撞体、pose planning、Cartesian Path，以及 fake hardware 与 URSim 两条执行路径；本轮还完成了 `External Control`、`speed_scaling`、`scaled_joint_trajectory_controller` 的执行链路排障闭环。
- `7D` 已在 fake hardware 与 URSim 下完成 `/goal_pose` 点击自动规划与执行验证，并能对非法 frame 或越界目标做前置拒绝。
- `7E` 已在 fake hardware + RViz 下完成 MoveIt Servo 最小闭环，连续两轮、每轮 8 次 `start_motion` 触发均通过，并沉淀了 Servo 启动门闩、静止态 joint state relay 与重复 Trigger harness 的调试经验。
- 已知边界：`7B` 的 `ros2 run` 直跑仍会绕过 launch 注入并复现本地 kinematics warning；当前决定将其视为已知边界，不影响阶段 3 完成判定。
