# Task 4G 控制器对比实验记录

## 1. 目标
对比 `joint_trajectory_controller` 与 `forward_command_controller` 在相同场景下的行为差异，
为手术机器人系统开发中的控制器选型提供依据。

## 2. 对比对象

| 控制器 | 包 | 接口类型 |
|---|---|---|
| `joint_trajectory_controller` | `ur3_joint_trajectory_controller_lab_py` (4F) | Action (`FollowJointTrajectory`) |
| `forward_command_controller` | `ur3_forward_command_controller_lab_py` (4G) | Topic (`Float64MultiArray`) |

## 3. 对比维度
1. 接口类型与指令粒度
2. 反馈机制（有无 goal/feedback/result）
3. 插值行为（控制器内部 vs 无插值直透）
4. 实现复杂度（客户端代码量）
5. 响应延迟与稳态精度
6. 异常行为（超限指令、指令中断）

## 4. 实验步骤

### 统一测试动作
- 目标：`shoulder_pan_joint` 从 0 移动到 0.5 rad，其余关节保持 0
- 硬件：`mock_components/GenericSystem`（仿真）

### 实验 A：forward_command_controller
```bash
# 终端 1：启动链路
ros2 launch ur3_forward_command_controller_lab_py ur3_forward_command.launch.py

# 终端 2：发送单帧位置指令
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]" --once

# 终端 3：观测关节状态
ros2 topic echo /joint_states --once
```

### 实验 B：joint_trajectory_controller
```bash
# 终端 1：启动链路
ros2 launch ur3_joint_trajectory_controller_lab_py ur3_joint_trajectory_controller.launch.py

# 终端 2：发送轨迹目标（使用 4F 的 action client 或 ros2 action send_goal）
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
      points: [{
        positions: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 2, nanosec: 0}
      }]
    }
  }"

# 终端 3：观测关节状态
ros2 topic echo /joint_states
```

### 实验 C：异常测试（forward_command_controller）
```bash
# 发送超出关节限位的指令（ur3 shoulder_pan_joint 限位约 ±3.14 rad）
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]" --once
```

## 5. 关键命令
```bash
# 查看控制器状态
ros2 control list_controllers

# 查看硬件接口
ros2 control list_hardware_interfaces

# 持续观测关节状态（带时间戳）
ros2 topic echo /joint_states

# 查看 action 服务器状态
ros2 action list
ros2 action info /joint_trajectory_controller/follow_joint_trajectory
```

## 6. 观测结果

### 实验 A：forward_command_controller
- 执行日期：2026-04-10
- 启动状态：`forward_position_controller` active，`joint_state_broadcaster` active
- 指令发送：`ros2 topic pub ... --once` 发布 `[0.5, 0.0, 0.0, 0.0, 0.0, 0.0]`
- 关节响应：`shoulder_pan_joint` position = 0.5（即时到达，无过渡过程）
- 反馈：无（需自行订阅 `/joint_states`）
- 备注：`A message was lost!!!` 提示为 topic 订阅时序问题，不影响结果

### 实验 B：joint_trajectory_controller
- 执行日期：2026-04-11
- 启动状态：`joint_trajectory_controller` active，`joint_state_broadcaster` active
- 指令发送：`ros2 action send_goal` 发送单点轨迹 `[0.5, 0.0, ...]`，`time_from_start: 2s`
- 关节响应：2 秒内平滑到达 0.5 rad（控制器内部插值）
- feedback 内容：执行过程中持续返回中间状态（actual/desired/error）
- result 状态：`error_code: 0`，`Goal successfully reached!`，status: `SUCCEEDED`

### 实验 C：异常测试（两种控制器）
- 执行日期：2026-04-11

**C1：forward_command_controller，指令 [10.0, ...]**
- 关节响应：`shoulder_pan_joint` position = 10.0，直接透传，无截断、无报错、无警告

**C2：joint_trajectory_controller，指令 [200.0, ...]，time_from_start: 2s**
- result：`error_code: 0`，`Goal successfully reached!`，status: `SUCCEEDED`
- 关节响应：同样直接到达 200.0，无任何限位保护

**根本原因分析：**
- URDF 中 `<limit lower="-3.14" upper="3.14"/>` 是描述性信息，不是执行层约束
- `mock_components/GenericSystem` 不执行关节限位，直接接受任何值（mock 的设计意图）
- JTC 的 `constraints`（`trajectory: 0.5`、`goal: 0.5`）是**轨迹跟踪误差约束**，不是角度限位；mock 硬件无物理惯性，实际位置始终跟上期望，误差为 0，约束从未触发
- 真实硬件驱动（如 `ur_robot_driver`）会在驱动层做限位保护；MoveIt 在规划层做限位检查

## 7. 对比结论

| 维度 | joint_trajectory_controller | forward_command_controller |
|---|---|---|
| 接口类型 | Action（FollowJointTrajectory） | Topic（Float64MultiArray） |
| 指令粒度 | 多点轨迹 + 时间约束 | 单帧位置向量（实时覆盖） |
| 反馈机制 | goal/feedback/result 完整闭环 | 无内置反馈 |
| 插值 | 控制器内部插值（cubic/linear） | 无插值，直接透传到硬件接口 |
| 响应延迟 | 按轨迹时间参数执行（可预测） | 即时（下一个控制周期生效） |
| 实现复杂度 | 高（需构造 trajectory_msgs） | 低（Float64MultiArray） |
| 超限行为 | 不做限位检查（mock 硬件直接到达 200.0） | 不做限位检查（mock 硬件直接到达 10.0） |
| 安全保护来源 | 硬件驱动层 + MoveIt 规划层 | 硬件驱动层（无规划层缓冲） |

## 8. 适用场景建议

### 决策树

```
需要精确轨迹跟踪 / 有时间约束 / 需要执行结果确认？
  └─ YES → joint_trajectory_controller
       适用：预规划手术路径执行、多点平滑运动、生产级控制

需要快速调试 / 实时流控制 / 遥操作透传？
  └─ YES → forward_command_controller（必须在上层加安全层）
       适用：主从遥操作从端、调试硬件响应、实时力控研究原型
```

### 手术机器人场景建议

| 场景 | 推荐控制器 | 原因 |
|---|---|---|
| 预规划手术路径执行 | `joint_trajectory_controller` | 有时间约束、插值平滑、结果可确认 |
| 主从遥操作从端 | `forward_command_controller` + 上层安全层 | 低延迟透传，但**必须**在上层做关节限位检查 |
| 硬件调试 / 单关节标定 | `forward_command_controller` | 实现简单，快速验证硬件响应 |
| 安全关键运动（靠近组织） | `joint_trajectory_controller` | 有约束检查，不会因单帧错误指令造成冲击 |

### 核心风险边界
**两种控制器在 mock 硬件上都不做关节限位保护**，区别不在于"谁更安全"：
- 安全保护不应依赖控制器层，而应在**规划层（MoveIt）或硬件驱动层**实现
- 生产级系统的标准架构：`MoveIt（规划层限位）→ JTC（轨迹执行）→ 硬件驱动（物理限位）`
- `forward_command_controller` 跳过了规划层，安全边界完全依赖硬件驱动，上层必须自行实现软件限位

`forward_command_controller` 的"透传"特性是双刃剑：
- 优点：延迟极低，适合实时控制研究
- 风险：无规划层缓冲，单帧错误指令直接作用于硬件，在手术机器人中使用时**上层必须实现软件限位**

## 9. 下一步
- [x] 完成实验 A（forward_command_controller）
- [x] 完成实验 B（joint_trajectory_controller 对比侧）
- [x] 完成实验 C（异常行为测试）
- [x] 填写完整对比结论与适用场景决策树
- [X] 更新 task4G_plan.md 完成记录
