# Task 5C：最小控制闭环实验记录

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[#] 已补写轨迹点，待在 mock hardware 下做联调验证`

---

## 1. 目标
- 在 `workspaces/ws_stage2/src/ur3_minimal_control_lab_py` 中完成一个最小控制闭环。
- 跑通：`读取 /joint_states -> 发送轨迹 -> 验证状态收敛`。
- 为后续更复杂的手术机器人执行链路积累最小可解释样例。

## 2. 应用包位置
- 包路径：`/home/minzi/ros2_lab/workspaces/ws_stage2/src/ur3_minimal_control_lab_py`
- 关键节点：
  - `joint_state_observer`
  - `joint_trajectory_sender`

## 3. 实验环境
- ROS 2 版本：Jazzy
- 驱动：`ur_robot_driver` 3.7.0
- 模式：mock hardware（`use_mock_hardware:=true`）

## 4. 练习 1：先观察状态

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_minimal_control_lab_py
source install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true \
  launch_rviz:=true

ros2 launch ur3_minimal_control_lab_py task5C_minimal_control.launch.py
```

### 观察记录
- `/joint_states` 是否持续更新：
- 六个关节的初始顺序是否与代码参数一致：
- 初始位姿里哪些关节不是 0：

## 5. 练习 2：补写轨迹点

### 位置
- 文件：`workspaces/ws_stage2/src/ur3_minimal_control_lab_py/ur3_minimal_control_lab_py/joint_trajectory_sender.py`
- 函数：`plan_demo_points()`

### 已完成补写
- 已补写 2 到 3 个轨迹点。
- 当前建议回顾：
  - 是否只让少量关节发生小幅变化。
  - `time_from_start` 是否严格递增。
- 接下来运行：
```bash
ros2 run ur3_minimal_control_lab_py joint_trajectory_sender
```

### 你设计这条轨迹时的判断
- 为什么选这些关节：
- 为什么选这个幅度：
- 为什么选这个执行时间：

## 6. 练习 3：闭环验证

### 发送前记录
- 当前关节角：

### 发送后记录
- goal 是否 accepted：
- result 的 `error_code`：
- `/joint_states` 是否收敛到目标附近：

### 结论
- 这次轨迹是如何经过控制器被执行的：
- fake hardware 下为什么仍能验证控制闭环：
- 如果改发给 `joint_trajectory_controller`，你预计会有什么区别：

## 7. 风险与排障
- Action server 名称不对：
- `joint_names` 顺序不一致：
- 轨迹点过大导致反馈难以解释：

## 8. 完成记录
- 日期：
- 备注：
