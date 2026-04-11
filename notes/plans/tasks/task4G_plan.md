# 任务 4G 规划文档：forward_command_controller 对比（实践版）

## 1. 任务目标
- 新建独立应用包 `ur3_forward_command_controller_lab_py`，集中存放 4G 任务代码。
- 在相同场景下对比 `joint_trajectory_controller` 与 `forward_command_controller` 的行为差异。
- 输出在不同开发阶段（调试 vs 生产）的控制器选择建议，为手术机器人系统开发提供决策依据。

## 2. 当前基线（来自仓库现状）
- 4A–4F 均已完成，具备完整的轨迹控制联调基础。
- 4F 包 `ur3_joint_trajectory_controller_lab_py` 已验证 `joint_trajectory_controller` 闭环。
- 4G 需要在同等硬件仿真条件下引入 `forward_command_controller` 进行横向对比。

## 3. 任务范围（单功能约束）
- 包含：
  - 新建应用包骨架与安装配置；
  - `forward_command_controller` 的最小配置与启动链路；
  - 统一测试动作（相同关节目标位置）下的双控制器对比实验；
  - 对比维度：接口类型、指令粒度、反馈机制、实现复杂度、适用场景；
  - 4G 学习文档沉淀。
- 不包含：
  - 多控制器并发切换的复杂编排策略；
  - 真机驱动接入、实时内核优化；
  - MoveIt 集成、运动学规划。

## 4. 新建应用包设计（本任务硬性要求）
- 包名：`ur3_forward_command_controller_lab_py`
- 位置：`workspaces/ws_tutorials/src/ur3_forward_command_controller_lab_py`
- 目录骨架：
  ```
  ur3_forward_command_controller_lab_py/
  ├── package.xml
  ├── setup.py
  ├── setup.cfg
  ├── resource/
  │   └── ur3_forward_command_controller_lab_py
  ├── ur3_forward_command_controller_lab_py/
  │   ├── __init__.py
  │   └── forward_cmd_publisher.py        # 直接发布关节位置指令的节点
  ├── config/
  │   └── ur3_controllers_forward.yaml    # forward_command_controller 配置
  ├── launch/
  │   └── ur3_forward_command.launch.py   # 启动链路
  └── urdf/
      └── ur3_simplified_ros2_control.urdf.xacro  # 复用或软链接 4F 的 xacro
  ```

## 5. 核心概念对比（学习框架）

| 维度 | joint_trajectory_controller | forward_command_controller |
|---|---|---|
| 接口类型 | Action (`FollowJointTrajectory`) | Topic (`/forward_position_controller/commands`) |
| 指令粒度 | 多点轨迹 + 时间约束 | 单帧位置向量（实时覆盖） |
| 反馈机制 | goal/feedback/result 完整闭环 | 无内置反馈，需自行订阅 `/joint_states` |
| 插值 | 控制器内部插值（cubic/linear） | 无插值，直接透传到硬件接口 |
| 实现复杂度 | 高（需构造 trajectory_msgs） | 低（发布 Float64MultiArray 即可） |
| 适用场景 | 精确轨迹跟踪、生产级运动控制 | 快速调试、遥操作、实时流控制 |

## 6. learn mode 分工
- 智能体负责（低学习价值，可直接完成）：
  - 新包脚手架、依赖声明、安装规则；
  - `forward_command_controller` YAML 配置与 launch 接线；
  - 对比实验命令清单与文档模板。
- 人类负责（关键学习点，执行时需你亲自决策）：
  - 选择 `forward_command_controller` 的 command interface 类型（position / velocity / effort）；
  - 设计统一测试动作（目标关节角度、发布频率）；
  - 观察并记录两种控制器在相同输入下的实际响应差异；
  - 判断手术机器人场景中各控制器的适用边界。

## 7. 实施步骤（3 天节奏）

### Day 1：建包与 forward_command_controller 最小配置
1. 在 `ws_tutorials/src` 下新建 `ur3_forward_command_controller_lab_py` 包骨架。
2. 编写 `ur3_controllers_forward.yaml`，注册 `forward_position_controller`（类型 `forward_command_controller/ForwardCommandController`）。
3. 复用 4F 的 xacro（或软链接），确认 `ros2_control` 标签包含 position command interface。
4. 编写 launch 文件，启动 `robot_state_publisher`、`ros2_control_node`、`joint_state_broadcaster`、`forward_position_controller`。
5. 验证：`ros2 control list_controllers` 显示 `forward_position_controller` 为 `active`。

### Day 2：对比实验执行
1. 设计统一测试动作：将 `shoulder_pan_joint` 从 0 移动到 0.5 rad（其余关节保持 0）。
2. **实验 A（joint_trajectory_controller）**：使用 4F 的 action client 发送轨迹目标，记录 feedback 与 result。
3. **实验 B（forward_command_controller）**：编写 `forward_cmd_publisher.py`，以 10 Hz 发布 `Float64MultiArray` 位置指令，记录 `/joint_states` 响应。
4. 对比记录：响应延迟、超调量、到达精度、实现代码行数。

### Day 3：结论沉淀与文档完善
1. 填写 `notes/labs/task4G_controller_comparison.md` 对比表格与结论。
2. 归纳"什么场景选哪种控制器"的决策树。
3. 更新 `task4G_plan.md` 完成记录。

## 8. 实验矩阵（4G 最小可复现）

| 组别 | 控制器 | 输入 | 观测指标 | 证据 |
|---|---|---|---|---|
| A | `joint_trajectory_controller` | 单点轨迹目标（0→0.5 rad，2s） | feedback 频率、result 状态、实际到达误差 | action client 日志 + `/joint_states` echo |
| B | `forward_command_controller` | 10 Hz Float64MultiArray（目标 0.5 rad） | `/joint_states` 响应延迟、稳态误差 | topic echo + 时间戳对比 |
| C | 异常测试 | 向 forward controller 发送超出关节限位的指令 | 控制器行为（截断/报错/透传） | 日志记录 |

## 9. 关键命令（执行阶段使用）
```bash
# 构建新包
cd /home/minzi/ros2_lab/workspaces/ws_tutorials
colcon build --packages-select ur3_forward_command_controller_lab_py
source install/setup.bash

# 启动 forward_command_controller 链路
ros2 launch ur3_forward_command_controller_lab_py ur3_forward_command.launch.py

# 查看控制器状态
ros2 control list_controllers
ros2 control list_hardware_interfaces

# 发布单帧位置指令（手动测试）
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
  "data: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]" --once

# 观测关节状态
ros2 topic echo /joint_states --once

# 运行对比发布节点
ros2 run ur3_forward_command_controller_lab_py forward_cmd_publisher
```

## 10. 交付物
- 代码：`ur3_forward_command_controller_lab_py` 应用包（含 launch、config、publisher 节点）。
- 文档：`notes/labs/task4G_controller_comparison.md`（含对比表格、结论、适用场景决策树）。

## 11. 验收标准
- `forward_position_controller` 能稳定激活并响应 topic 指令。
- 能明确回答"什么场景选哪种控制器"，并给出手术机器人场景的具体建议。
- 有可复现的对比步骤与证据（日志截图或 echo 输出）。
- 不引入 4H 范围内的额外功能。

## 12. 风险与回退
- 风险 1：`forward_command_controller` 的 command interface 与 xacro 中声明的不一致。
  - 回退：先用 `ros2 control list_hardware_interfaces` 确认可用接口，再调整 YAML。
- 风险 2：发布频率过低导致控制器进入 idle 状态。
  - 回退：提高发布频率至 50 Hz，或检查 `cmd_timeout` 参数配置。
- 风险 3：测试条件不一致导致对比结论偏差。
  - 回退：固定同一输入目标与同一采样窗口（5 秒），重复实验取平均。

## 13. 完成记录
- 状态：`[ ] 未开始` `[#] 进行中` `[x] 已完成`
- 日期：`2026-04-11`
- 备注：三组实验全部完成。关键发现：forward_command_controller 不做关节限位检查，真机使用必须在上层加安全层。
