# Task 6A：C++ Action Client 迁移

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[#] 已创建 C++ 包骨架，待补写 Action Client 发送逻辑`

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
- 是否构建通过：
- 若失败，报错集中在哪个依赖或头文件：

## 5. 练习 2：补写 Action Client 核心逻辑

### 位置
- 文件：`workspaces/ws_stage2/src/ur3_minimal_control_lab_cpp/src/joint_trajectory_sender.cpp`
- 函数：`send_goal_request()`

### 已准备内容
- 参数声明、`/joint_states` 订阅、当前位姿重排、轨迹构造、结果回调函数签名已搭好。

### TODO(human)
- 亲自补写：
  - `SendGoalOptions` 的三个回调绑定；
  - `async_send_goal` 调用；
  - 发送失败时的日志与 shutdown 路径。

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
- action server 是否可用：
- goal 是否 accepted：
- result 状态码：
- error_code：

## 7. 对比总结
- Python 版更顺手的地方：
- C++ 版更清晰的地方：
- 你对 `rclcpp_action` 的理解：

## 8. 完成记录
- 日期：
- 备注：
