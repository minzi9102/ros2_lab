# 任务 6A 规划文档：C++ Action Client 迁移

## 1. 任务目标
- 在 `ws_stage2` 新建一个独立 C++ 应用包 `ur3_minimal_control_lab_cpp`。
- 将 5C 中已经跑通的 Python Action Client 思路迁移到 `rclcpp_action`。
- 保持任务聚焦在一个功能：C++ 版 `FollowJointTrajectory` 发送链路。

## 2. 当前基线（来自仓库现状）
- 5C 已完成：`ur3_minimal_control_lab_py` 已在 mock hardware 下完成最小控制闭环验证。
- 阶段 1 已有 C++ 基础：
  - `ur3_state_monitor_cpp` 提供了参数声明、订阅、定时器与日志风格的参考。
- 当前仓库尚无阶段 2 的 C++ Action Client 包。

## 3. 任务范围（单功能约束）
- 包含：
  - 新建 `ur3_minimal_control_lab_cpp` 包骨架；
  - 实现或补完一个 C++ `FollowJointTrajectory` Action Client；
  - 在 mock hardware 环境下验证：能连上 action server，并收到结果。
- 不包含：
  - URSim 接入；
  - speed scaling 真实链路验证；
  - C++ 状态观察节点；
  - MoveIt 集成。

> 说明：用户同时提出“C++ 迁移”和“URSim + speed scaling”两个方向。按仓库单功能约束，本任务只推进前者；URSim 将作为后续独立任务处理。

## 4. 新建应用包设计（本任务硬性要求）
- 包名：`ur3_minimal_control_lab_cpp`
- 位置：`workspaces/ws_stage2/src/ur3_minimal_control_lab_cpp`
- 目录骨架：
  ```text
  ur3_minimal_control_lab_cpp/
  ├── CMakeLists.txt
  ├── package.xml
  ├── launch/
  │   └── task6A_cpp_action_client.launch.py
  └── src/
      └── joint_trajectory_sender.cpp
  ```

## 5. learn mode 分工
- 智能体负责（低学习价值，可直接完成）：
  - 包脚手架、依赖声明、CMake 安装规则；
  - 关节名、参数、轨迹校验、日志与 shutdown 外层胶水；
  - 与 5C Python 版对齐的轨迹构造辅助函数。
- 人类负责（关键学习点，执行时需你亲自决策）：
  - `rclcpp_action::Client<...>::SendGoalOptions` 的回调绑定；
  - `async_send_goal` 的调用链路；
  - 理解 C++ 中 goal response / feedback / result 三类回调的签名与生命周期。

## 6. 核心学习问题
1. `rclcpp_action` 的 callback wiring 与 `rclpy.action` 有哪些结构差异？
2. 为什么 C++ 版也应该先等 `/joint_states`，再发轨迹？
3. 如果 `goal accepted` 但 result 失败，最先该检查哪一层？

## 7. 实施步骤（推荐 3 次练习）

### 练习 1：建包并构建通过
1. 在 `ws_stage2/src` 新建 `ur3_minimal_control_lab_cpp`。
2. 配置 `package.xml`、`CMakeLists.txt`、launch 安装规则。
3. 先做到 `colcon build --packages-select ur3_minimal_control_lab_cpp` 通过。

### 练习 2：补完 Action Client 发送逻辑
1. 打开 `src/joint_trajectory_sender.cpp`。
2. 在 `TODO(human)` 位置补写：
   - `SendGoalOptions`；
   - `goal_response_callback` / `feedback_callback` / `result_callback` 绑定；
   - `async_send_goal` 调用。
3. 保持其余辅助逻辑不改，降低排障面。

### 练习 3：在 mock hardware 下验证
1. 启动 UR 官方 mock hardware。
2. 运行 C++ 发送节点。
3. 记录：
   - 是否成功连上 action server；
   - goal 是否 accepted；
   - result 的状态码与错误码；
   - 与 Python 版相比，代码结构哪里更繁琐、哪里更显式。

## 8. 关键命令（执行阶段使用）
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_minimal_control_lab_cpp
source install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true \
  launch_rviz:=true

ros2 run ur3_minimal_control_lab_cpp joint_trajectory_sender
```

## 9. 交付物
- 代码：`workspaces/ws_stage2/src/ur3_minimal_control_lab_cpp`
- 文档：`notes/labs/task6A_cpp_action_client.md`

## 10. 验收标准
- C++ 包能独立构建通过。
- C++ 节点能连上 `/scaled_joint_trajectory_controller/follow_joint_trajectory`。
- 你能解释 Python 与 C++ Action Client 在“回调绑定”和“类型显式性”上的主要差异。

## 11. 风险与回退
- 风险 1：`rclcpp_action` 回调签名写错，导致编译失败。
  - 回退：先固定 callback 成员函数签名，再绑定 `SendGoalOptions`。
- 风险 2：C++ 节点未等到 `/joint_states` 就发送轨迹。
  - 回退：保留“先拿当前位姿再发送”的门槛。
- 风险 3：把 URSim 接入一起做，排障面骤增。
  - 回退：本任务只在 mock hardware 下验 C++ Action Client。

## 12. 完成记录
- 状态：`[x] 已完成`
- 日期：`2026-04-12`
- 备注：已补完 `SendGoalOptions` 与 `async_send_goal`，通过 `colcon build --packages-select ur3_minimal_control_lab_cpp`，并在 mock hardware 下完成运行验证；运行结果为 `status=4 error_code=0`。
