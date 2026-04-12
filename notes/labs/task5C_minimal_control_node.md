# Task 5C：最小控制闭环实验记录

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[x] 已完成最小控制闭环验证`

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
- `/joint_states` 是否持续更新：是
- 六个关节的初始顺序是否与代码参数一致：不一致。话题顺序是 `elbow -> shoulder_lift -> shoulder_pan -> wrist_1 -> wrist_2 -> wrist_3`，代码中的 `joint_names` 顺序是 `shoulder_pan -> shoulder_lift -> elbow -> wrist_1 -> wrist_2 -> wrist_3`。
- 为什么仍然能正确发送轨迹：`joint_trajectory_sender` 在发送前会按 joint name 重排当前位姿，而不是直接假设 `/joint_states` 的顺序与目标顺序一致。
- 初始位姿里哪些关节不是 0：`shoulder_lift_joint=-1.57`、`wrist_1_joint=-1.57`

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
- 为什么选这些关节：选择前三个关节，能更明显地看到大臂链路的变化。
- 为什么选这个幅度：当前使用约 `1.0 rad` 的幅度，mock hardware 下现象更明显，但若迁移到 URSim 或真机，建议先收敛到更小幅度。
- 为什么选这个执行时间：`1s -> 3s -> 5s` 既满足严格递增，也给了控制器足够时间完成往返动作。

## 6. 练习 3：闭环验证

### 发送前记录
- 当前关节角：
  ```yaml
  name:
  - elbow_joint
  - shoulder_lift_joint
  - shoulder_pan_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint
  position:
  - 0.0
  - -1.57
  - 0.0
  - -1.57
  - 0.0
  - 0.0
  ```

### 发送后记录
- goal 是否 accepted：是
- result 的 `error_code`：`0`
- Action 最终结果：
  ```text
  [INFO] [1775986024.842056233] [ur3_joint_trajectory_sender]: Goal accepted, waiting for result...
  [INFO] [1775986029.892510865] [ur3_joint_trajectory_sender]: Result received: status=4 error_code=0 error_string='Goal successfully reached!'
  ```
- 本次联调补充日志：
  ```text
  [WARN] [1775986459.469411855] [ur3_joint_trajectory_sender]: Waiting for current /joint_states before sending trajectory.
  [INFO] [1775986460.469254736] [ur3_joint_trajectory_sender]: Waiting for FollowJointTrajectory action server...
  [INFO] [1775986460.469811478] [ur3_joint_trajectory_sender]: Sending FollowJointTrajectory goal...
  [INFO] [1775986465.471630490] [ur3_joint_trajectory_sender]: Result received: status=4 error_code=0 error_string='Goal successfully reached!'
  ```
- `/joint_states` 是否收敛到目标附近：是，但这里收敛到的是“最终点”，而最终点被设计成 `current_positions`，所以发送前后两次 `ros2 topic echo /joint_states --once` 看起来完全相同，这是符合预期的。
- 发送后关节角：
  ```yaml
  name:
  - elbow_joint
  - shoulder_lift_joint
  - shoulder_pan_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint
  position:
  - 0.0
  - -1.57
  - 0.0
  - -1.57
  - 0.0
  - 0.0
  ```
- 对“前后相同”的解释：轨迹是 `起点 -> 中间点 -> 起点`，所以最终状态返回起点。Action result 成功说明中间轨迹已被控制器执行，前后状态相同只说明“回到了起点”，不说明“没有运动”。
- 关于 `A message was lost!!!`：这次两次 `ros2 topic echo /joint_states --once` 都拿到了完整样本，因此这条告警没有影响本轮结论。更稳妥的观测方式仍然是并行运行 `joint_state_observer`。

### 结论
- 这次轨迹是如何经过控制器被执行的：已验证 `joint_trajectory_sender -> /scaled_joint_trajectory_controller/follow_joint_trajectory -> 控制器执行 -> Action result 返回成功` 这条链路可用。
- fake hardware 下为什么仍能验证控制闭环：mock hardware 会模拟 hardware interface 的状态读写，因此即使没有真实 RTDE 硬件链路，也能验证控制器是否接受并执行轨迹。
- 如果改发给 `joint_trajectory_controller`，你预计会有什么区别：在 mock hardware 下大概率也能执行成功；但在真机上它不会感知 speed scaling，不如 `scaled_joint_trajectory_controller` 适合作为默认入口。
- 对这次“前后 `/joint_states` 相同”的最终理解：这是因为轨迹终点被故意设为当前位姿，不是控制失败。

## 7. 风险与排障
- Action server 名称不对：
- `joint_names` 顺序不一致：
- 轨迹点过大导致反馈难以解释：

## 8. 完成记录
- 日期：2026-04-12
- 备注：已完成最小控制闭环验证。证据包括：Action goal accepted、`status=4 error_code=0`、发送前后 `/joint_states` 一致且与“起点 -> 中间点 -> 起点”的轨迹设计相符。
