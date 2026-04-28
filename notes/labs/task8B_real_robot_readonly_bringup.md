# Task 8B：只读启动 ur_control 并验证状态流

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[x] 已完成`

## 1. 目标
- 在 8A 通过后启动真实机器人 driver。
- 第一轮只观察状态，不发送任何轨迹或控制命令。
- 验证 `/joint_states`、controller、TF、driver 日志与 External Control 前后状态变化。

## 2. 应用包与文件位置
- 骨架包：`workspaces/ws_stage4/src/ur3_real_bringup_lab`
- launch：`launch/task8B_readonly_bringup.launch.py`
- 任务计划：`notes/plans/tasks/task8B_plan.md`

## 3. 当前准备情况
- 已准备：
  - 官方 `ur_control.launch.py` 的只读 wrapper；
  - 本记录模板。
- 待你完成：
  - 确认 8A 无阻断项；
  - 在示教器上按现场要求操作 External Control；
  - 记录启动前后状态。

## 4. 启动前复核
- 8A 是否通过：`是`
- 本轮机器人型号：`ur3e`
- 本轮机器人 IP：`192.168.56.101`
- ROS PC 机器人网段 IP：`192.168.56.2`
- launch 是否保持 `launch_rviz:=false`：`是`
- 是否确认本轮不发送轨迹：`是`
- 现场急停是否可达：`是`

## 5. 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_real_bringup_lab
source install/setup.bash

ros2 launch ur3_real_bringup_lab task8B_readonly_bringup.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.56.101 \
  reverse_ip:=192.168.56.2 \
  launch_rviz:=false \
  activate_joint_controller:=false
```

只读观察命令：

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
ros2 control list_controllers
ros2 topic list
ros2 node list
```

## 6. 运行记录
- 运行日期：`2026-04-28`
- driver 是否启动成功：`是，官方 ur_control.launch.py 可连接真机；随后已将只读参数同步进 task8B wrapper`
- External Control 启动前 ROS 端状态：`program_running=false；speed_scaling=0.0；/joint_states 约 500 Hz；轨迹控制器 inactive`
- External Control 启动后 ROS 端状态：`program_running=true；speed_scaling=100.0；robot_mode=RUNNING；safety_mode=NORMAL；remote_control=false`
- `/joint_states` 是否持续更新：`是`
- `/joint_states` 频率：`约 500 Hz`
- 关节名是否匹配 UR3：`是，包含 elbow_joint、shoulder_lift_joint、shoulder_pan_joint、wrist_1_joint、wrist_2_joint、wrist_3_joint`
- controller manager 是否可用：`是`
- 关键 controller 状态摘要：`joint_state_broadcaster、io_and_status_controller、speed_scaling_state_broadcaster、force_torque_sensor_broadcaster、tcp_pose_broadcaster、ur_configuration_controller 为 active；scaled_joint_trajectory_controller、joint_trajectory_controller、forward_*、force_mode、freedrive、tool_contact、passthrough 为 inactive`
- TF / robot description 是否可观察：`是，/robot_description、/tf、/tf_static 可见；base_link -> tool0 可查询`
- 终端 warning / error：`存在 FIFO/实时调度 warning、controller manager overrun warning、机器人 calibration 与默认 kinematics 不匹配 error；关闭 driver 时 pal_statistics 出现 context invalid 日志但进程 cleanly exited`
- 中途故障记录：`一次重启失败，报 speed_slider_mask 被另一个 RTDE client 控制；清理上一轮残留 launch 进程后恢复正常`

## 7. 观察矩阵

| 观察项 | 预期 | 实际记录 | 结论 |
|---|---|---|---|
| driver 进程 | 保持运行 | `能启动并完成只读观察；观察后已主动停止` | `通过` |
| `/joint_states` | 持续更新 | `约 500 Hz，关节名匹配 UR3e 六轴` | `通过` |
| controller list | 可列出 | `状态 broadcaster active，轨迹/运动 controller inactive` | `通过` |
| External Control 前后变化 | 可解释 | `program_running false -> true；speed_scaling 0.0 -> 100.0` | `通过` |
| 无轨迹发送 | 确认未发送 | `未调用 action、MoveIt Execute 或 /commands topic` | `通过` |

## 8. 你需要完成的判断
- 当前是否只是“可连接”，而不是“可运动”：`是。8B 只证明 driver、状态流、controller manager、TF 和 External Control 状态可观察，不证明可以安全运动`
- 哪些状态还需要在 8C 进一步验证：`Dashboard/Remote Control 策略、controller 激活门闩、calibration 文件、实时调度/overrun 风险、运动前的状态检查脚本`
- 是否允许进入 8C：`允许进入 8C 状态门闩验证；仍不允许发送运动指令`

## 9. 完成标准
- 真实 driver 能启动或失败原因已清楚记录。
- `/joint_states`、controller、TF 至少完成只读验证。
- 本轮没有发送任何运动命令。

## 10. 完成记录
- 日期：`2026-04-28`
- 最终结论：`8B 通过。真实机器人 driver 可只读启动，External Control 启动后 program_running 与 speed_scaling 状态符合预期；本轮未发送任何运动指令。`
- 下一步：`进入 8C：Dashboard 与 controller 状态门闩验证，优先处理 calibration 与实时调度风险记录`
