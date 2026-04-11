# 任务 5B 规划文档：理解控制器体系

## 1. 任务目标
- 深入理解 `scaled_joint_trajectory_controller` 与 `joint_trajectory_controller` 的差异。
- 理解 speed scaling 机制及 teach pendant 速度限制对外部控制的影响。
- 输出"控制器、驱动、MoveIt 三者分工"的清晰解释，达到阶段 2 验收要求。

## 2. 当前基线（来自仓库现状）
- 5A 完成后：已能在 fake hardware 模式下启动 UR3，controller 状态可见。
- 4F/4G 完成后：具备 `joint_trajectory_controller` 实际使用经验，理解 Action 接口。
- 尚未接触 `scaled_joint_trajectory_controller` 的 speed scaling 参数与行为。

## 3. 任务范围（单功能约束）
- 包含：
  - `scaled_joint_trajectory_controller` vs `joint_trajectory_controller` 行为对比；
  - speed scaling 概念：`/speed_scaling_state` 话题、`/set_speed_slider` 服务；
  - teach pendant 速度限制在外部控制模式下的传导路径；
  - 控制器/驱动/MoveIt 三者分工的结构化解释。
- 不包含：
  - 真机 teach pendant 操作（fake hardware 无法模拟速度限制信号）；
  - MoveIt 实际配置与使用（阶段 3 范围）；
  - 自写控制节点（5C 范围）。

## 4. 核心概念框架（学习锚点）

### 4.1 三者分工

| 组件 | 职责 | 类比 |
|---|---|---|
| 驱动（ur_robot_driver） | 与硬件通信，暴露 hardware interface，转发速度缩放信号 | 翻译官：把 ROS 指令翻译成机器人能懂的协议 |
| 控制器（ros2_control） | 读取 hardware interface，执行轨迹插值与跟踪 | 执行官：决定每个控制周期发什么指令给关节 |
| MoveIt | 运动学规划，生成无碰撞轨迹，调用控制器 Action | 规划师：决定走哪条路，但不管怎么走每一步 |

### 4.2 speed scaling 机制

```
teach pendant 速度滑块
        ↓
ur_robot_driver 读取速度缩放因子（0.0–1.0）
        ↓
发布到 /speed_scaling_state
        ↓
scaled_joint_trajectory_controller 订阅并动态调整轨迹执行速度
        ↓
joint_trajectory_controller 不感知此信号（按原始时间戳执行）
```

### 4.3 两种控制器对比

| 维度 | joint_trajectory_controller | scaled_joint_trajectory_controller |
|---|---|---|
| speed scaling 感知 | 否 | 是 |
| 真机安全性 | 低（忽略 pendant 限速） | 高（遵守 pendant 限速） |
| fake hardware 行为 | 正常 | 正常（缩放因子默认 1.0） |
| 适用场景 | 仿真调试、无速度限制需求 | 真机部署、需遵守安全速度限制 |

## 5. learn mode 分工
- 智能体负责（低学习价值，可直接完成）：
  - 话题/服务名称查询、YAML 配置示例、文档模板。
- 人类负责（关键学习点，执行时需你亲自决策）：
  - 在 fake hardware 下切换两种控制器，观察 `/speed_scaling_state` 话题是否存在；
  - 尝试用 `ros2 service call` 调用 `/set_speed_slider`，观察响应；
  - 用自己的语言写出"三者分工"的解释，不依赖文档原文。

## 6. 实施步骤（2 天节奏）

### Day 1：控制器切换实验
1. 在 5A 的 fake hardware 环境中，查看默认激活的控制器：
   ```bash
   ros2 control list_controllers
   ```
2. 确认 `scaled_joint_trajectory_controller` 是否已加载，若未激活则手动激活：
   ```bash
   ros2 control switch_controllers \
     --activate scaled_joint_trajectory_controller \
     --deactivate joint_trajectory_controller
   ```
3. 查询 speed scaling 相关话题与服务：
   ```bash
   ros2 topic list | grep speed
   ros2 service list | grep speed
   ros2 topic echo /speed_scaling_state
   ```
4. 记录：fake hardware 模式下 speed scaling 信号的默认值是什么？

### Day 2：概念内化与文档沉淀
1. 阅读 `ur_robot_driver` 官方文档中关于 `scaled_joint_trajectory_controller` 的说明。
2. 用自己的语言回答以下问题（写入交付文档）：
   - 如果 teach pendant 速度设为 50%，`joint_trajectory_controller` 和 `scaled_joint_trajectory_controller` 各会怎么表现？
   - 为什么 MoveIt 不直接控制关节，而是通过控制器 Action？
   - fake hardware 模式下 speed scaling 为何始终是 1.0？
3. 绘制"控制器/驱动/MoveIt 三者分工"的文字结构图，写入文档。
4. 更新 `task5B_plan.md` 完成记录。

## 7. 关键命令（执行阶段使用）
```bash
# 查看所有控制器状态
ros2 control list_controllers

# 切换控制器
ros2 control switch_controllers \
  --activate scaled_joint_trajectory_controller \
  --deactivate joint_trajectory_controller

# 查询 speed scaling
ros2 topic echo /speed_scaling_state
ros2 service list | grep speed

# 查看控制器参数
ros2 param list /scaled_joint_trajectory_controller
ros2 param get /scaled_joint_trajectory_controller speed_scaling_interface_name
```

## 8. 交付物
- 文档：`notes/concepts/task5B_ur_controller_system.md`
  - 包含：三者分工结构图、两种控制器对比表、speed scaling 传导路径、三个问题的自答。

## 9. 验收标准
- 能清楚解释 `scaled_joint_trajectory_controller` 与 `joint_trajectory_controller` 的核心差异。
- 能用一段话（不超过 5 句）解释"控制器、驱动、MoveIt 三者分别负责什么"。
- 能解释 teach pendant 速度限制为何在 fake hardware 模式下不生效。

## 10. 风险与回退
- 风险 1：fake hardware 模式下 `scaled_joint_trajectory_controller` 未加载。
  - 回退：检查 `ur_control.launch.py` 的 `initial_joint_controller` 参数，尝试显式指定。
- 风险 2：`/speed_scaling_state` 话题在 fake hardware 下不存在。
  - 回退：这是预期行为，记录为"fake hardware 不模拟速度缩放信号"，作为学习结论之一。
- 风险 3：控制器切换失败（依赖冲突）。
  - 回退：先停止当前激活的控制器，再激活目标控制器；检查 hardware interface 是否被占用。

## 11. 完成记录
- 状态：`[ ] 未开始` `[#] 进行中` `[x] 已完成`
- 日期：
- 备注：
