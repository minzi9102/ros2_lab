# Task 8B：只读启动 ur_control 并验证状态流

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[ ] 待填写`

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
- 8A 是否通过：`【请填写：是 / 否】`
- 本轮机器人型号：`【请填写】`
- 本轮机器人 IP：`【请填写】`
- launch 是否保持 `launch_rviz:=false`：`【请填写】`
- 是否确认本轮不发送轨迹：`【请填写：是 / 否】`
- 现场急停是否可达：`【请填写：是 / 否】`

## 5. 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_real_bringup_lab
source install/setup.bash

ros2 launch ur3_real_bringup_lab task8B_readonly_bringup.launch.py \
  ur_type:=<ur3_or_ur3e> \
  robot_ip:=<robot_ip> \
  launch_rviz:=false
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
- 运行日期：`【请填写】`
- driver 是否启动成功：`【请填写】`
- External Control 启动前 ROS 端状态：`【请填写】`
- External Control 启动后 ROS 端状态：`【请填写】`
- `/joint_states` 是否持续更新：`【请填写】`
- `/joint_states` 频率：`【请填写】`
- 关节名是否匹配 UR3：`【请填写】`
- controller manager 是否可用：`【请填写】`
- 关键 controller 状态摘要：`【请填写】`
- TF / robot description 是否可观察：`【请填写】`
- 终端 warning / error：`【请填写】`

## 7. 观察矩阵

| 观察项 | 预期 | 实际记录 | 结论 |
|---|---|---|---|
| driver 进程 | 保持运行 | `【请填写】` | `【请填写】` |
| `/joint_states` | 持续更新 | `【请填写】` | `【请填写】` |
| controller list | 可列出 | `【请填写】` | `【请填写】` |
| External Control 前后变化 | 可解释 | `【请填写】` | `【请填写】` |
| 无轨迹发送 | 确认未发送 | `【请填写】` | `【请填写】` |

## 8. 你需要完成的判断
- 当前是否只是“可连接”，而不是“可运动”：`【请填写】`
- 哪些状态还需要在 8C 进一步验证：`【请填写】`
- 是否允许进入 8C：`【请填写：允许 / 不允许】`

## 9. 完成标准
- 真实 driver 能启动或失败原因已清楚记录。
- `/joint_states`、controller、TF 至少完成只读验证。
- 本轮没有发送任何运动命令。

## 10. 完成记录
- 日期：`【请填写】`
- 最终结论：`【请填写】`
- 下一步：`【请填写】`
