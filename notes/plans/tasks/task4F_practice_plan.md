# 任务 4F 规划文档：joint_trajectory_controller 联调（实践版）

## 1. 任务目标
- 新建独立应用包 `ur3_joint_trajectory_controller_lab_py`，集中存放 4F 任务代码。
- 将 FollowJointTrajectory Action Client 接入本地控制器执行链路。
- 完成一次完整的 goal -> feedback -> result 闭环验证。
- 能解释 `joint_trajectory_controller` 与 `action interface` 的职责边界。

## 2. 当前基线（来自 4E 完成状态）
- 4E 已完成：`controller_manager` + `joint_state_broadcaster` 最小闭环可用。
- 现有包：`ur3_follow_joint_trajectory_client_py`（客户端）、`ur3_ros2_control_lab_py`（控制器基础）。
- 4F 需要：在 4E 基础上添加 `joint_trajectory_controller`，并联调客户端。

## 3. 任务范围（单功能约束）
- 包含：
  - 新建应用包骨架与安装配置；
  - 在 4E 的 URDF/YAML 基础上添加 `joint_trajectory_controller` 配置；
  - 最小轨迹点准备与联调脚本；
  - Action 接口对齐与关节名校验；
  - 4F 学习文档沉淀。
- 不包含：
  - 复杂运动学约束优化、碰撞规避规划；
  - 真机驱动接入、实时内核优化；
  - MoveIt 集成、轨迹平滑算法。

## 4. 新建应用包设计（本任务硬性要求）
- 包名：`ur3_joint_trajectory_controller_lab_py`
- 位置：`workspaces/ws_tutorials/src/ur3_joint_trajectory_controller_lab_py`
- 目标目录骨架（计划）：
  - `launch/ur3_joint_trajectory_controller.launch.py`
  - `config/ur3_controllers_with_trajectory.yaml`
  - `urdf/ur3_simplified_ros2_control.urdf.xacro`（从 4E 复用或链接）
  - `scripts/test_trajectory_client.py`（最小轨迹联调脚本）
  - `package.xml` / `setup.py` / `setup.cfg`
  - `resource/ur3_joint_trajectory_controller_lab_py`
  - `ur3_joint_trajectory_controller_lab_py/__init__.py`

## 5. 关键配置对齐清单
- [ ] 确认 `joint_trajectory_controller` 的 action 接口名（通常 `/follow_joint_trajectory`）
- [ ] 校对关节顺序与 URDF 中的关节名一致性
- [ ] 准备 2-3 个平滑轨迹点（起点、中间点、终点）
- [ ] 验证时间参数（time_from_start）的合理性

## 6. 实施步骤（3 天节奏）

### Day 1：建包与配置落盘
- 新建 `ur3_joint_trajectory_controller_lab_py`；
- 复用或链接 4E 的 URDF 文件；
- 编写 `ur3_controllers_with_trajectory.yaml`（包含 `joint_state_broadcaster` + `joint_trajectory_controller`）；
- 编写 launch 文件启动 `controller_manager` 与 `spawner`。

### Day 2：启动链路闭环
- 启动 `ros2_control_node` 与两个控制器；
- 验证 `ros2 control list_controllers` 显示两个 active 控制器；
- 编写最小轨迹联调脚本（单点目标 -> 多点轨迹）；
- 观测 feedback 与 result 返回。

### Day 3：验证与文档沉淀
- 完成 A/B/C 验证矩阵并记录日志；
- 记录常见失败场景（关节名不匹配、时间参数错误、action 超时）；
- 形成排障 runbook；
- 更新 `notes/concepts/follow_joint_trajectory_debug.md`。

## 7. 实验矩阵（4F 最小可复现）

| 组别 | 输入条件 | 预期现象 | 证据 |
|---|---|---|---|
| A 基线 | 启动 controller_manager，激活两个控制器 | 两个控制器均为 active | `ros2 control list_controllers` |
| B 单点 | 发送单点轨迹目标（仅起点） | goal 被接受，result 返回成功 | action client 日志 + feedback 消息 |
| C 多点 | 发送 3 点平滑轨迹 | goal 被接受，feedback 持续更新，result 返回成功 | action client 日志 + 完整 feedback 序列 |
| D 异常 | 人为改错关节名或时间参数 | action 返回失败或超时 | 错误日志 + 恢复步骤 |

## 8. 关键命令（执行阶段使用）
```bash
cd /home/minzi/ros2_lab/workspaces/ws_tutorials
ros2 pkg create --build-type ament_python ur3_joint_trajectory_controller_lab_py
colcon build --packages-select ur3_joint_trajectory_controller_lab_py
source install/setup.bash
ros2 launch ur3_joint_trajectory_controller_lab_py ur3_joint_trajectory_controller.launch.py
ros2 control list_controllers
ros2 action list
ros2 action send_goal /follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {header: {frame_id: base_link}, joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], points: [{positions: [0, 0, 0, 0, 0, 0], time_from_start: {sec: 1, nanosec: 0}}]}}"
```

## 9. 交付物
- 代码：`ur3_joint_trajectory_controller_lab_py` 最小应用包（仅 4F 所需文件）。
- 脚本：`scripts/test_trajectory_client.py`（最小轨迹联调脚本）。
- 文档：`notes/concepts/follow_joint_trajectory_debug.md`（含步骤、日志、结论、排障 runbook）。

## 10. 验收标准
- `joint_trajectory_controller` 能稳定激活；
- 单点与多点轨迹目标均能被接受并返回结果；
- 能复述 action 接口、关节映射、时间参数的对齐过程；
- 能独立说明 1 个成功案例与 1 个失败案例的排障过程；
- 不引入 4G 范围内的额外功能。

## 11. 风险与回退
- 风险 1：关节名与控制器配置不一致。
  - 回退：先用 `ros2 control list_hardware_interfaces` 确认可用关节，再对齐 YAML。
- 风险 2：action 接口名不匹配。
  - 回退：用 `ros2 action list` 确认实际接口名，再调整客户端。
- 风险 3：轨迹时间参数不合理导致超时。
  - 回退：先用单点目标验证接口连通，再恢复多点轨迹。

## 12. 完成记录
- 状态：`[ ] 未开始` `[#] 进行中` `[x] 已完成`
- 日期：
- 备注：
