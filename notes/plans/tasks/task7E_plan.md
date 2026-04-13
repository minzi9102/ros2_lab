# 任务 7E 规划文档：MoveIt Servo 连续控制入门

## 1. 任务目标
- 在 `ws_stage3` 新建一个独立 C++ 包 `ur3_moveit_servo_lab_cpp`。
- 跑通最小 MoveIt Servo 速度控制链路。
- 向 `/servo_node/delta_twist_cmds` 发送 `TwistStamped`，理解 Servo 与离线路径规划的边界。

## 2. 当前基线（来自仓库现状）
- 本机已具备 `moveit_servo` 和 `ur_moveit_config/config/ur_servo.yaml`。
- 当前仓库还没有任何 Servo 练习包或 `forward_position_controller` 联动骨架。

## 3. 任务范围（单功能约束）
- 包含：
  - Servo bringup 包装；
  - `forward_position_controller` 联动；
  - 最小 `TwistStamped` 发送节点；
  - fake hardware 下的最小验收。
- 不包含：
  - URSim 下 Servo 必做验收；
  - 视觉伺服；
  - 遥操作输入设备；
  - 多控制模式切换策略。

## 4. 包设计
- 包名：`ur3_moveit_servo_lab_cpp`
- 位置：`workspaces/ws_stage3/src/ur3_moveit_servo_lab_cpp`
- 目录骨架：
  ```text
  ur3_moveit_servo_lab_cpp/
  ├── CMakeLists.txt
  ├── package.xml
  ├── launch/
  │   └── task7E_moveit_servo.launch.py
  └── src/
      └── servo_twist_commander_node.cpp
  ```

## 5. 接口约束
- 输入话题：`/servo_node/delta_twist_cmds`
- 状态话题：`/servo_node/status`
- 控制器：`forward_position_controller`
- 本任务不使用 `scaled_joint_trajectory_controller`

## 6. learn mode 分工
- 智能体负责：
  - launch 胶水、控制器切换包装、命令发布骨架。
- 人类负责：
  - 线速度 / 角速度上限；
  - 持续时间；
  - 停止策略；
  - 状态解释与风险判断。

## 7. 核心学习问题
1. 为什么 Servo 属于“连续速度控制”，而不是“先规划整条轨迹再执行”？
2. 为什么 Servo 更适合主从操作、遥操作、人机协同这类场景？
3. 为什么本任务要切到 `forward_position_controller`？

## 8. 实施步骤
1. 新建包并先构建通过。
2. 跑通 Servo bringup。
3. 发布一段短时 `TwistStamped`。
4. 停止发布后，确认机械臂停下。
5. 记录：
   - 命令发送频率；
   - 运动响应；
   - 停止行为；
   - `status` 话题输出。

## 9. 关键命令（执行阶段使用）
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_servo_lab_cpp
source install/setup.bash

ros2 launch ur3_moveit_servo_lab_cpp task7E_moveit_servo.launch.py \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true
```

## 10. 交付物
- 代码：`workspaces/ws_stage3/src/ur3_moveit_servo_lab_cpp`
- 文档：`notes/labs/task7E_moveit_servo.md`

## 11. 验收标准
- Servo 链路能启动。
- 节点能向 `/servo_node/delta_twist_cmds` 发送最小命令。
- 停止发送后机械臂能停下。
- 你能解释 Servo 与 MoveGroup 离线路径规划的边界。
