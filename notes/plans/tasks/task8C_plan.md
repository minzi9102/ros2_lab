# 任务 8C 规划文档：Dashboard、controller 与 External Control 状态验证

## 1. 任务目标
- 在 8B 只读 bringup 可重复后，建立执行前状态门闩。
- 查询 Dashboard、controller 和 External Control 相关状态，形成“允许运动前必须全部满足”的状态矩阵。
- 明确失败时是拒绝执行、停止观察还是交由人工恢复。

## 2. 当前基线（来自仓库现状）
- 8A 已完成真机接入前检查。
- 8B 已完成只读 driver 启动与基础状态流观察。
- 阶段 2/3 已积累 controller 与 External Control 排障经验，但尚未在真实机器人上固化状态门闩。

## 3. 任务范围（单功能约束）
- 包含：
  - Dashboard 服务查询；
  - controller 状态查询；
  - robot mode / safety mode / program state 记录；
  - 执行前状态矩阵；
  - 拒绝执行判据。
- 不包含：
  - 解锁 protective stop 的自动流程；
  - 自动 power on / brake release；
  - 发送轨迹；
  - MoveIt 规划执行。

## 4. 包设计建议
- 包名：`ur3_real_bringup_lab`
- 位置：`workspaces/ws_stage4/src/ur3_real_bringup_lab`
- 目录骨架建议：
  ```text
  ur3_real_bringup_lab/
  ├── CMakeLists.txt
  ├── package.xml
  ├── launch/
  │   └── task8C_state_check.launch.py
  └── scripts/
      └── check_real_robot_state.py
  ```
- `check_real_robot_state.py` 首轮只做查询与日志输出，不调用恢复类 Dashboard 服务。

## 5. learn mode 分工
- 智能体负责：
  - 状态查询脚本骨架；
  - Dashboard / controller 状态矩阵模板；
  - 拒绝执行判据整理。
- 人类负责：
  - 判断哪些 safety mode 可以继续，哪些必须人工介入；
  - 判断 Remote Control、External Control、program state 的现场含义；
  - 确认是否允许后续 8D 进入动作测试。

## 6. 核心学习问题
1. Dashboard 的 robot mode、safety mode、program state 分别说明什么？
2. 为什么 controller active 只是必要条件，不是充分条件？
3. 为什么 protective stop 不应该被脚本默认自动解锁？
4. 状态门闩失败时，为什么应该拒绝执行而不是自动重试？

## 7. 实施步骤

### 练习 1：列出可用 Dashboard 服务
1. 启动 8B 的 driver。
2. 使用 `ros2 service list` 找到 dashboard 相关服务。
3. 记录服务名、类型和可查询项。

### 练习 2：查询 robot / safety / program 状态
1. 查询 robot mode。
2. 查询 safety mode。
3. 查询 program state 或 program running 状态。
4. 记录 Remote Control 相关提示，必要时由人类在示教器确认。

### 练习 3：查询 controller 状态
1. 记录所有 controller。
2. 确认 `joint_state_broadcaster` 和 `scaled_joint_trajectory_controller` 等关键组件状态。
3. 如果 controller inactive，先记录原因，不自动切换。

### 练习 4：形成状态门闩
1. 将每项状态分为 `pass`、`warn`、`block`。
2. 输出 “允许进入 8D” 或 “拒绝进入 8D” 的结论。
3. 人类确认后再进入动作任务。

## 8. 关键命令（执行阶段使用）
```bash
source /opt/ros/jazzy/setup.bash

ros2 service list | grep dashboard
ros2 service type /dashboard_client/get_robot_mode
ros2 service type /dashboard_client/get_safety_mode

ros2 service call /dashboard_client/get_robot_mode ur_dashboard_msgs/srv/GetRobotMode "{}"
ros2 service call /dashboard_client/get_safety_mode ur_dashboard_msgs/srv/GetSafetyMode "{}"

ros2 control list_controllers
ros2 action list | grep follow_joint_trajectory
```

> 服务名称以现场 `ros2 service list` 为准；如果 namespace 不同，记录实际名称，不强行照抄示例。

## 9. 状态矩阵草案

| 类别 | 检查项 | pass | warn | block |
|---|---|---|---|---|
| Dashboard | robot mode | 机器人处于可控运行状态 | 模式可解释但需人工确认 | power off / disconnected / unknown |
| Dashboard | safety mode | normal | reduced / recovery 等需解释 | protective stop / safeguard stop / fault |
| Program | External Control | program running | program loaded but not running | 未加载、未运行、状态未知 |
| Controller | joint state broadcaster | active | restarting | missing / inactive |
| Controller | trajectory controller | active | inactive 但原因明确 | missing / inactive 且原因未知 |
| State | `/joint_states` | 持续更新 | 频率抖动需记录 | 无消息 |

## 10. 交付物
- 文档：`notes/labs/task8C_dashboard_controller_state.md`
- 可选代码：`workspaces/ws_stage4/src/ur3_real_bringup_lab/scripts/check_real_robot_state.py`
- 结果：一份执行前状态门闩规范。

## 11. 验收标准
- 能列出并查询 Dashboard 关键状态。
- 能解释 controller 状态与 External Control 状态的关系。
- 能输出 `pass/warn/block` 状态矩阵。
- 未通过状态门闩时，不会进入 8D 动作测试。

## 12. 风险与回退
- 风险 1：Dashboard 服务名称与示例不同。
  - 回退：以 `ros2 service list` 和服务类型为准，记录 namespace。
- 风险 2：状态查询脚本开始承担恢复职责。
  - 回退：本任务只查询、不恢复；恢复操作留给人工 runbook。
- 风险 3：状态矩阵过宽，导致风险被放行。
  - 回退：默认不确定即 block，等待人工确认。
