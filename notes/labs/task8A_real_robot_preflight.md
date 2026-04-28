# Task 8A：真机接入前安全与网络清单

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[x] 已完成`

## 1. 目标
- 在启动真实 UR3 driver 前，完成现场、安全、网络、系统实时性和权限基线检查。
- 明确哪些条件不满足时禁止进入 Task 8B。
- 输出一份可复用的 preflight 记录。

## 2. 应用包与文件位置
- 骨架包：`workspaces/ws_stage4/src/ur3_real_bringup_lab`
- 配置模板：`config/task8A_preflight_checklist.yaml`
- 任务计划：`notes/plans/tasks/task8A_plan.md`

## 3. 当前准备情况
- 已准备：
  - preflight YAML 骨架；
  - 8A 任务计划；
  - 本记录模板。
- 待你完成：
  - 现场安全确认；
  - 机器人型号、IP、PolyScope 信息；
  - ROS PC 网络与内核信息；
  - 是否允许进入 8B 的人工判断。

## 4. 机器人与现场信息
- 记录日期：`2026.4.28`
- 操作者：`催眠剂`
- 旁站 / 安全确认者：`赤眉军`
- 机器人型号（`ur3` / `ur3e`）：`ur3e`
- 控制柜类型：`不清楚`
- PolyScope / PolyScope X 版本：`5.12`
- 机器人 IP：`192.168.56.101`
- External Control URCap 是否已安装：`是`
- External Control 程序名称：`external_control.urp`
- Remote Control 状态：`可用`

## 5. 现场安全检查
- 急停按钮位置：`在示教器上方`
- 保护停止恢复责任人：`采棉机`
- 机器人工作空间是否清空：`是`
- 是否设置低速 / reduced mode：`是，已在 PolyScope Safety 配置中确认存在 Reduced 限制`
- 是否有人保持急停可达：`是`
- 本轮是否允许进入只读 bringup：`是`

## 6. ROS PC 与网络基线

### 执行命令
```bash
ip addr
ip route
ping -c 20 <robot_ip>
uname -a
lsb_release -a
ros2 doctor --report
ros2 pkg prefix ur_robot_driver
ros2 pkg prefix ur_dashboard_msgs
ros2 pkg prefix controller_manager
```

### 结果填写
- ROS PC IP：`192.168.56.2`
- 网卡名称：`enp7s0`
- 子网掩码 / 网段：`255.255.255.0 / 192.168.56.0/24`
- 路由确认：`192.168.56.101 dev enp7s0 src 192.168.56.2`
- ping 丢包率：`0%（20 transmitted, 20 received）`
- ping 平均延迟：`0.200 ms`
- Ubuntu 版本：`24.04.4 LTS`
- ROS 2 版本：`jazzy`
- 内核版本：`6.17.0-20-generic`
- 实时性类型（generic / lowlatency / PREEMPT_RT）：`generic / PREEMPT_DYNAMIC，非 PREEMPT_RT`
- CPU governor / 电源状态：`powersave`
- ROS 包基线：`ur_robot_driver、ur_dashboard_msgs、controller_manager 均位于 /opt/ros/jazzy`

### 网络修复记录
- 初始问题：普通 `ping 192.168.56.101` 失败。
- 根因：Docker 网络 `ursim_net` 占用了 `192.168.56.0/24`，导致系统把机器人 IP 路由到 `br-87328a36ea46`。
- 修复操作：
  - 删除 Docker 网络 `ursim_net`；
  - 将有线连接 `enp7s0` 设置为 `192.168.56.2/24`；
  - 设置 `ipv4.never-default yes`，避免有线机器人网络接管默认路由。
- 修复后验证：`ip route get 192.168.56.101` 走 `enp7s0`，`ping -c 20 192.168.56.101` 丢包率为 `0%`。

## 7. 阻断项检查

| 检查项 | 当前结果 | 是否阻断 8B | 备注 |
|---|---|---|---|
| 急停位置明确 | `明确` | `否` | `无` |
| 工作空间清空 | `清空` | `否` | `无` |
| 机器人型号明确 | `ur3e` | `否` | `无` |
| IP 与网段明确 | `机器人 192.168.56.101；ROS PC 192.168.56.2/24；enp7s0` | `否` | `普通 ping 已稳定通过` |
| External Control 准备好 | `准备好` | `否` | `无` |
| Dashboard / Remote 条件明确 | `明确` | `否` | `无` |
| ROS PC 状态可接受 | `网络可达，ROS 包可用；内核非实时，CPU governor 为 powersave` | `否` | `允许进入 8B 只读 bringup；不允许据此发送运动指令` |

## 8. 你需要完成的判断
- 当前现场是否允许进入 8B 只读 bringup：`允许`
- 如果不允许，主要阻断原因：`无当前阻断项；原网络阻断项已修正`
- 如果允许，进入 8B 前还需要提醒自己的事项：`只读观察，不发送轨迹；保持急停可达；由人类在示教器上操作 External Control 程序`

## 9. 完成标准
- 所有关键现场、安全、网络、系统信息已记录。
- 阻断项已明确。
- 你能解释为什么 8A 通过不等于允许运动。

## 10. 完成记录
- 日期：`2026-04-28`
- 最终结论：`8A 通过。机器人网络已稳定可达，现场安全与 Reduced 配置已确认，允许进入 8B 只读 bringup；仍禁止发送任何运动指令。`
- 下一步：`进入 8B：启动真实机器人 driver 并只读观察 /joint_states、controller、TF 与 driver 日志`
