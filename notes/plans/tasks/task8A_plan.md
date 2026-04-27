# 任务 8A 规划文档：真机接入前安全与网络清单

## 1. 任务目标
- 在真正启动 `ur_control.launch.py` 之前，完成 UR3 真机现场、安全、网络、系统实时性和权限基线检查。
- 建立阶段 4 的 preflight checklist，明确哪些条件不满足时禁止进入 8B。
- 把“能 ping 通机器人”和“允许连接 driver”拆开判断，避免把真机接入做成临场试错。

## 2. 当前基线（来自仓库现状）
- 阶段 2 已完成 fake hardware、URSim、speed scaling 与 External Control 执行链路排障。
- 阶段 3 已完成 MoveIt 2 与 Servo fake hardware 主路径。
- 当前仓库尚未接入真实 UR3，也没有阶段 4 工作区或真机 preflight 记录。

## 3. 任务范围（单功能约束）
- 包含：
  - 机器人信息记录；
  - ROS PC 网络与系统基线；
  - 现场安全确认；
  - Remote Control / External Control / Dashboard 前置条件梳理；
  - 8B 前的禁止条件列表。
- 不包含：
  - 启动真实机器人 driver；
  - 发送任何轨迹或控制命令；
  - 创建真机运动节点；
  - 保护停止恢复自动化。

## 4. 文档设计
- 计划文档：`notes/plans/tasks/task8A_plan.md`
- 实验记录：`notes/labs/task8A_real_robot_preflight.md`
- 建议后续 runbook：`notes/runbooks/real_robot_preflight.md`

## 5. learn mode 分工
- 智能体负责：
  - checklist 模板；
  - 网络与系统信息采集命令清单；
  - 风险项分类与 8B 进入条件整理。
- 人类负责：
  - 现场确认急停、保护停止、限位、工作空间；
  - 确认真实机器人型号、控制柜、PolyScope 版本；
  - 设置或确认机器人 IP、ROS PC IP、Remote Control、External Control 程序；
  - 判断现场是否允许进入只读 bringup。

## 6. 核心学习问题
1. 为什么真机接入前要先记录现场安全状态，而不是先跑 launch？
2. 为什么固定 IP、网段、网卡选择会影响后续排障效率？
3. 为什么 lowlatency / PREEMPT_RT 是阶段 4 的系统基线问题，而不是“以后再说”的优化项？
4. 为什么急停、保护停止和软件停止不能混为一谈？

## 7. 实施步骤

### 练习 1：机器人与现场信息采集
1. 记录机器人型号：`ur3` 或 `ur3e`。
2. 记录控制柜类型、PolyScope / PolyScope X 版本。
3. 记录示教器当前模式、Remote Control 是否可用。
4. 确认 External Control URCap 是否已安装、程序是否已创建。
5. 确认现场急停按钮位置、保护停止恢复流程、可运动空间、旁站人员。

### 练习 2：网络基线采集
1. 记录机器人 IP、ROS PC IP、子网掩码、网卡名称。
2. 确认 ROS PC 与机器人在同一可达网段。
3. 使用 `ping` 做基础连通性和延迟观察。
4. 记录是否存在 VPN、无线网、Docker bridge、多个默认路由等干扰项。

### 练习 3：ROS PC 系统基线采集
1. 记录 Ubuntu、ROS 2、内核版本。
2. 记录当前是否为 generic、lowlatency 或 PREEMPT_RT 内核。
3. 记录 CPU governor、是否插电、是否有明显高负载进程。
4. 记录 `ur_robot_driver`、`ur_dashboard_msgs`、`ros2controlcli` 是否可用。

### 练习 4：8B 进入条件评审
1. 汇总所有检查项。
2. 标出阻断项、警告项、可接受项。
3. 人类现场确认后，才允许进入 8B 只读 bringup。

## 8. 关键命令（执行阶段使用）
```bash
# 网络
ip addr
ip route
ping -c 20 <robot_ip>

# 系统与实时性基线
uname -a
lsb_release -a
ros2 --version
python3 --version

# ROS 包可用性
ros2 pkg prefix ur_robot_driver
ros2 pkg prefix ur_dashboard_msgs
ros2 pkg prefix controller_manager
ros2 control --help
```

## 9. 观察矩阵

| 类别 | 检查项 | 通过条件 | 阻断条件 |
|---|---|---|---|
| 现场 | 急停与保护停止 | 人类能指出位置与恢复流程 | 不清楚如何停机或恢复 |
| 机器人 | 型号与软件版本 | 已记录 `ur_type` 与 PolyScope 版本 | 不确定型号仍准备连接 |
| 网络 | IP 与网段 | ROS PC 能稳定 ping 通机器人 | IP 冲突、丢包明显、走错网卡 |
| 权限 | Remote / External Control | 已确认现场配置路径 | 不知道是否已安装 URCap |
| 系统 | 内核与负载 | 已记录实时性基线 | 高负载或系统状态未知 |

## 10. 交付物
- 文档：`notes/labs/task8A_real_robot_preflight.md`
- 结果：一份可复用的真机接入前检查清单。

## 11. 验收标准
- 已明确 `ur_type`、`robot_ip`、ROS PC IP、网卡和现场网络拓扑。
- 已确认急停、保护停止、工作空间和人工旁站要求。
- 已记录 ROS PC 内核与实时性基线。
- 已确认 External Control / Remote Control / Dashboard 前置条件。
- 已列出“禁止进入 8B”的阻断条件。

## 12. 风险与回退
- 风险 1：网络能 ping 通，但机器人权限或 External Control 未准备好。
  - 回退：不启动 driver，先完成示教器和 URCap 配置确认。
- 风险 2：ROS PC 使用无线或复杂路由，后续连接不稳定。
  - 回退：改为固定有线连接和明确网段。
- 风险 3：现场安全职责不清晰。
  - 回退：暂停阶段 4，先明确操作者、旁站者和停机责任。
