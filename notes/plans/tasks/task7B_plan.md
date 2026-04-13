# 任务 7B 规划文档：C++ MoveGroupInterface 最小规划节点

## 1. 任务目标
- 在 `ws_stage3` 新建一个独立 C++ 包 `ur3_moveit_move_group_lab_cpp`。
- 写出第一个基于 `MoveGroupInterface` 的 UR3 规划节点。
- 支持四种最小路径：
  - `joint goal`
  - `pose goal`
  - `plan_only`
  - `plan_and_execute`

## 2. 当前基线（来自仓库现状）
- `7A` 将提供已跑通的 MoveIt bringup。
- 仓库已有阶段 2 C++ Action Client 经验，但还没有 MoveIt 2 C++ 规划节点。

## 3. 任务范围（单功能约束）
- 包含：
  - 新建 `ur3_moveit_move_group_lab_cpp`；
  - 提供最小 MoveGroupInterface 节点；
  - 在 fake hardware 下验证一次 joint goal 与 pose goal。
- 不包含：
  - Collision Object；
  - Cartesian Path；
  - `goal_pose` 点击 demo；
  - MoveIt Servo。

## 4. 包设计
- 包名：`ur3_moveit_move_group_lab_cpp`
- 位置：`workspaces/ws_stage3/src/ur3_moveit_move_group_lab_cpp`
- 目录骨架：
  ```text
  ur3_moveit_move_group_lab_cpp/
  ├── CMakeLists.txt
  ├── package.xml
  ├── launch/
  │   └── task7B_move_group_interface.launch.py
  └── src/
      └── move_group_planner_node.cpp
  ```

## 5. 接口约束
- 默认 `planning_group`：`ur_manipulator`
- 默认 pose 目标 link：`tool0`
- 默认参考系：`base_link`
- 规划成功后是否执行，由布尔参数 `execute_plan` 控制

## 6. learn mode 分工
- 智能体负责：
  - 包脚手架、依赖、参数声明、日志与 launch。
- 人类负责：
  - `setJointValueTarget` / `setPoseTarget` 的关键调用；
  - `plan()` 成功 / 失败分支；
  - “规划失败时是否退出、重试或降级”的处理策略。

## 7. 核心学习问题
1. `MoveGroupInterface` 与 `FollowJointTrajectory` Action Client 在职责上有什么不同？
2. 为什么 pose goal 是“给末端期望姿态”，而不是“直接给关节值”？
3. 如果 pose 不可达，最先失败的是 IK、碰撞检查还是执行层？

## 8. 实施步骤
1. 新建 C++ 包并确保构建通过。
2. 先支持 `joint goal` + `plan_only`。
3. 再增加 `pose goal` + `plan_and_execute`。
4. 记录：
   - `joint goal` 和 `pose goal` 的规划结果；
   - 不可达 pose 时的失败现象；
   - `plan_only` 与 `plan_and_execute` 的日志差异。

## 9. 关键命令（执行阶段使用）
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_move_group_lab_cpp
source install/setup.bash

ros2 launch ur3_moveit_bringup_lab task7A_moveit_quickstart.launch.py \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true \
  launch_rviz:=false

ros2 launch ur3_moveit_move_group_lab_cpp task7B_move_group_interface.launch.py
```

## 10. 交付物
- 代码：`workspaces/ws_stage3/src/ur3_moveit_move_group_lab_cpp`
- 文档：`notes/labs/task7B_move_group_interface_cpp.md`

## 11. 验收标准
- C++ 包独立构建通过。
- 节点能完成一次 `joint goal` 规划。
- 节点能完成一次 `pose goal` 规划。
- 对不可达 pose，能明确记录规划失败并且不发送执行请求。
