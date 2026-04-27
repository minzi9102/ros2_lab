# 任务 8B 规划文档：只读启动 ur_control 并验证状态流

## 1. 任务目标
- 在 8A preflight 通过后，启动真实机器人 `ur_control.launch.py`，但第一轮只观察状态，不发送任何轨迹。
- 验证 `/joint_states`、TF、controller manager、robot description、driver 日志和 External Control 前后状态变化。
- 建立“可连接但不可运动”的安全分界线。

## 2. 当前基线（来自仓库现状）
- 8A 将提供机器人型号、IP、网络、现场安全和系统基线。
- 阶段 2/3 已有 URSim 与 fake hardware 经验，但真实机器人状态流尚未验证。
- 当前阶段 4 尚未创建 `workspaces/ws_stage4` 或真机 bringup wrapper。

## 3. 任务范围（单功能约束）
- 包含：
  - 真实机器人 driver 启动；
  - External Control 启动前后状态观察；
  - `/joint_states`、controller、TF、driver 日志验证；
  - 只读 bringup 记录。
- 不包含：
  - 发送 FollowJointTrajectory goal；
  - MoveIt 规划执行；
  - Dashboard 恢复操作自动化；
  - 真机动作节点。

## 4. 包设计建议
- 初始可先不建包，直接用官方 launch 验证。
- 若 8B 需要沉淀 wrapper，后续放入：
  ```text
  workspaces/ws_stage4/src/ur3_real_bringup_lab/
  ├── package.xml
  ├── CMakeLists.txt
  └── launch/
      └── task8B_readonly_bringup.launch.py
  ```
- wrapper 只负责参数固定和日志清晰化，不改变官方 driver 行为。

## 5. learn mode 分工
- 智能体负责：
  - 启动命令清单；
  - 观察矩阵；
  - 日志记录模板；
  - 后续 wrapper 骨架建议。
- 人类负责：
  - 确认 8A 阻断项已清零；
  - 在示教器上操作 External Control 程序；
  - 判断现场是否允许启动 driver；
  - 全程保持急停可达和旁站观察。

## 6. 核心学习问题
1. 为什么 8B 明确禁止发送轨迹？
2. External Control 启动前后，ROS 端哪些状态应该变化？
3. `/joint_states` 连续发布说明了什么，又不能说明什么？
4. 为什么 controller active 不等于允许运动？

## 7. 实施步骤

### 练习 1：启动前复核
1. 打开 8A checklist，确认没有阻断项。
2. 确认 robot_ip、ur_type、网卡和示教器状态。
3. 确认终端不会自动运行任何运动脚本。

### 练习 2：启动官方 driver
1. 启动 `ur_control.launch.py`。
2. 第一轮建议 `launch_rviz:=false`，减少界面噪声；如需要观察模型再打开 RViz。
3. 不在同一终端运行任何轨迹测试 launch。

### 练习 3：External Control 前后对比
1. 在未运行 External Control 时记录 driver 输出和 controller 状态。
2. 由人类在示教器启动 External Control 程序。
3. 再次记录 `/joint_states`、controller、日志变化。

### 练习 4：只读状态记录
1. 记录 `/joint_states` 的频率与关节名。
2. 记录 `ros2 control list_controllers`。
3. 记录 TF 树是否包含预期 base 到 tool 链路。
4. 记录 driver 中 warning / error，不尝试自动恢复。

## 8. 关键命令（执行阶段使用）
```bash
cd /home/minzi/ros2_lab
source /opt/ros/jazzy/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=<ur3_or_ur3e> \
  robot_ip:=<robot_ip> \
  launch_rviz:=false

# 新终端，只读观察
source /opt/ros/jazzy/setup.bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 topic list
ros2 node list
```

## 9. 观察矩阵

| 阶段 | 观察项 | 预期 | 异常记录 |
|---|---|---|---|
| driver 启动 | launch 是否存活 | 无崩溃，日志可解释 | 进程退出、连接失败 |
| External Control 前 | controller 状态 | 可列出 controller，但未必可执行 | controller manager 不可用 |
| External Control 后 | `/joint_states` | 持续更新，关节名匹配 UR3 | 无消息或时间戳异常 |
| 控制器 | `scaled_joint_trajectory_controller` | 状态清楚可记录 | inactive 且原因未知 |
| TF | base 到 tool | 链路存在 | frame 缺失 |

## 10. 交付物
- 文档：`notes/labs/task8B_real_robot_readonly_bringup.md`
- 可选代码：`workspaces/ws_stage4/src/ur3_real_bringup_lab`
- 结果：一条只读 bringup 证据链。

## 11. 验收标准
- 能安全启动 driver 并保持只读观察。
- 能记录 External Control 启动前后的 ROS 状态变化。
- 能确认 `/joint_states`、controller list、TF 至少处于可观察状态。
- 能明确说明当前仍不允许发送轨迹的原因。

## 12. 风险与回退
- 风险 1：driver 连接失败。
  - 回退：回到 8A 检查 IP、路由、机器人模式、External Control 配置。
- 风险 2：只读阶段误触发运动测试。
  - 回退：关闭相关终端，保留只读命令白名单。
- 风险 3：controller 状态看似 active，误以为可以运动。
  - 回退：进入 8C 做 Dashboard 与状态门闩，不在 8B 运动。
