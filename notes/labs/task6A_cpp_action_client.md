# Task 6A：C++ Action Client 迁移

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[x] 已完成 mock hardware 联调验证`

---

## 1. 目标
- 将 5C 的 Python `FollowJointTrajectory` 发送链路迁移到 C++。
- 理解 `rclcpp_action` 的回调绑定方式。

## 2. 应用包位置
- 包路径：`/home/minzi/ros2_lab/workspaces/ws_stage2/src/ur3_minimal_control_lab_cpp`
- 关键文件：`src/joint_trajectory_sender.cpp`

## 3. 实验环境
- ROS 2 版本：Jazzy
- 驱动：`ur_robot_driver` 3.7.0
- 模式：mock hardware（`use_mock_hardware:=true`）

## 4. 练习 1：构建骨架

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_minimal_control_lab_cpp
source install/setup.bash
```

### 观察记录
- 是否构建通过：是
- 若失败，报错集中在哪个依赖或头文件：本轮补写后，主要问题集中在 `SendGoalOptions` 类型限定、回调绑定里的类名拼写，以及残留的 `throw logic_error`；修正后已重新构建通过。

## 5. 练习 2：补写 Action Client 核心逻辑

### 位置
- 文件：`workspaces/ws_stage2/src/ur3_minimal_control_lab_cpp/src/joint_trajectory_sender.cpp`
- 函数：`send_goal_request()`

### 已准备内容
- 参数声明、`/joint_states` 订阅、当前位姿重排、轨迹构造、结果回调函数签名已搭好。

### TODO(human)
- 已补写：
  - `SendGoalOptions` 的三个回调绑定；
  - `async_send_goal` 调用。
- review 时补充修正：
  - `SendGoalOptions` 的类型限定；
  - 回调绑定中的类名引用；
  - 残留的 `throw logic_error`。

### 你要回答的问题
- Python 里 `send_goal_async(..., feedback_callback=...)` 与 C++ 里 `SendGoalOptions` 的差别是什么：
- 哪个版本更显式，代价是什么：

## 6. 练习 3：mock hardware 下验证

### 执行命令
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true \
  launch_rviz:=true

ros2 run ur3_minimal_control_lab_cpp joint_trajectory_sender
```

### 发送后记录
- action server 是否可用：是
- goal 是否 accepted：是
- 反馈是否正常返回：是，持续收到 `desired_len=6 actual_len=6`
- result 状态码：`4`
- error_code：`0`
- 关键日志：
  ```text
  [INFO] [1776001534.308458842] [ur3_joint_trajectory_sender_cpp]: joint_trajectory_sender_cpp started. action=/scaled_joint_trajectory_controller/follow_joint_trajectory topic=/joint_states
  [INFO] [1776001535.308544581] [ur3_joint_trajectory_sender_cpp]: Waiting for FollowJointTrajectory action server...
  [INFO] [1776001535.309266123] [ur3_joint_trajectory_sender_cpp]: Goal accepted, waiting for result...
  [INFO] [1776001540.359796502] [ur3_joint_trajectory_sender_cpp]: Result received: status=4 error_code=0 error_string='Goal successfully reached!'
  [INFO] [1776001540.359840050] [ur3_joint_trajectory_sender_cpp]: Shutting down node: Goal finished
  ```
- 结论：C++ 版 `FollowJointTrajectory` Action Client 已在 mock hardware 下验证通过。

## 7. 对比总结
- Python 版更顺手的地方：`send_goal_async(..., feedback_callback=...)` 写法更短，异步发送入口更直接。
- C++ 版更清晰的地方：`SendGoalOptions` 把 goal response / feedback / result 三类回调显式拆开，更容易看清生命周期与类型边界。
- 你对 `rclcpp_action` 的理解：它本质上仍是“异步发 goal，再分别处理响应、反馈、结果”，只是 C++ 版要求更明确地声明回调类型和绑定方式。

## 8. 完成记录
- 日期：2026-04-12
- 备注：已完成 C++ Action Client 迁移与 mock hardware 联调验证，结果为 `status=4 error_code=0`。
