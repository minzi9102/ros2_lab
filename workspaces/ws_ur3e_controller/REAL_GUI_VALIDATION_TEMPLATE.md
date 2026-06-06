# ws_ur3e_controller 真机 GUI 验收记录模板

记录日期：`TODO(human): YYYY-MM-DD`

操作者：`TODO(human)`

旁站 / 安全确认者：`TODO(human)`

工作区：`/home/minzi/ros2_lab/workspaces/ws_ur3e_controller`

目标：在完成真机 preflight、状态门闩和 plan-only 验证后，使用 `ur3e_named_motion_gui_py` 分别触发单次 `HOME` / `READY` 执行，并记录 final-target gate 与执行后系统状态。

## 1. 安全边界

本次测试只允许：

- 使用已经现场复核并启用的 `real.home` 和 `real.ready`。
- 每次只点击一个 GUI 目标按钮。
- 每次执行前确认 8C 动作前门闩为 PASS。
- 每次执行后重新运行 8C。
- 以 service 响应和 `/joint_states` final-target gate 共同判断结果。

本次测试禁止：

- 自动解锁 protective stop。
- 自动 restart safety、brake release、power on/off。
- 失败后自动重试或自动补发另一个目标。
- 在状态不明确时继续点击 GUI。
- 同时运行其他会向机器人发送运动命令的节点。

任一 safety、controller、External Control、driver 或 joint state 异常无法解释时，终止当天运动测试。

## 2. 现场与网络确认

填写：

- 工作空间已清空：`TODO(human): 是 / 否`
- 急停可达：`TODO(human): 是 / 否`
- 示教器可操作：`TODO(human): 是 / 否`
- Reduced / 速度限制策略已确认：`TODO(human)`
- 机器人型号：`ur3e`
- 机器人 IP：`192.168.56.101`
- ROS PC IP：`192.168.56.2`
- 网络接口：`TODO(human)`
- `ping` 结果：`TODO(human)`
- External Control 程序：`/programs/external_control.urp`
- Remote Control 策略：`TODO(human): PC 管理 / 示教器人工启动`

验证命令：

```bash
ip route get 192.168.56.101
ping -c 20 192.168.56.101
```

结论：`TODO(human): PASS / BLOCK`

## 3. 启动 8B 真机 bringup

终端 A：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ur3_real_bringup_lab task8B_readonly_bringup.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.56.101 \
  reverse_ip:=192.168.56.2 \
  launch_rviz:=false \
  activate_joint_controller:=false
```

记录：

- calibration checksum：`TODO(human)`
- hardware ready gate：`TODO(human): PASS / BLOCK`
- Dashboard client：`TODO(human): available / unavailable`
- External Control：`TODO(human): running / stopped / unknown`
- `/joint_states`：`TODO(human): available / unavailable`

## 4. 激活 controller 并运行 8C

终端 B：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 control switch_controllers \
  --activate scaled_joint_trajectory_controller \
  --strict

ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
  require_trajectory_controller_active:=true
```

动作前门闩记录：

| 检查项 | 结果 |
|---|---|
| robot mode | `TODO(human): RUNNING / 其他` |
| safety mode | `TODO(human): NORMAL / 其他` |
| program running | `TODO(human): true / false` |
| remote control | `TODO(human): true / false` |
| scaled trajectory controller | `TODO(human): active / inactive` |
| `/joint_states` 频率 | `TODO(human)` |
| speed scaling | `TODO(human)` |
| 8C gate | `TODO(human): PASS / WARN / BLOCK` |

只有 8C 为 PASS 才允许继续。

## 5. 启动 MoveIt 语义层

终端 C：

```bash
source /opt/ros/jazzy/setup.bash

ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e \
  launch_rviz:=false \
  launch_servo:=false
```

记录：

- `move_group` ready：`TODO(human): 是 / 否`
- 是否出现 `robot_description_semantic` / SRDF 错误：`TODO(human): 否 / 是`

## 6. plan-only 验证

终端 D 先以禁止执行模式启动 named controller：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_ur3e_controller
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ur3e_named_motion_controller_cpp named_motion_controller.launch.py \
  runtime_mode:=real \
  execute:=false
```

另开终端依次调用：

```bash
ros2 service call /ur3e_named_motion_controller/execute_named_target \
  ur3e_controller_msgs/srv/ExecuteNamedTarget \
  "{target_name: home, execute: false, human_confirmation: ''}"
```

```bash
ros2 service call /ur3e_named_motion_controller/execute_named_target \
  ur3e_controller_msgs/srv/ExecuteNamedTarget \
  "{target_name: ready, execute: false, human_confirmation: ''}"
```

记录：

| 目标 | accepted | planned | status | message |
|---|---|---|---|---|
| home | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` |
| ready | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` |

两个目标都必须返回 `status=planned` 才允许进入 GUI execute。

## 7. 启动真实执行 controller 与 GUI

停止上一节 `execute:=false` 的 named controller，然后在终端 D 重启：

```bash
ros2 launch ur3e_named_motion_controller_cpp named_motion_controller.launch.py \
  runtime_mode:=real \
  execute:=true
```

再次运行一次 8C，确认结果仍为 PASS。

终端 E 启动真机 GUI：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_ur3e_controller
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ur3e_named_motion_gui_py real_named_motion_gui.launch.py
```

`real_named_motion_gui.launch.py` 默认携带：

```text
human_confirmation=I_CONFIRM_REAL_ROBOT_MOTION
```

确认：

- GUI 显示 `service: connected`：`TODO(human): 是 / 否`
- 当前没有其他运动命令发送者：`TODO(human): 是 / 否`
- 现场再次允许执行第一个目标：`TODO(human): 是 / 否`

## 8. 单次 HOME 验收

仅点击一次 `HOME`。

记录：

- 点击时间：`TODO(human)`
- GUI request 状态：`TODO(human)`
- `accepted`：`TODO(human)`
- `planned`：`TODO(human)`
- `executed`：`TODO(human)`
- `status`：`TODO(human)`
- `message`：`TODO(human)`
- 是否包含 `final-target gate passed`：`TODO(human): 是 / 否`
- 机械臂实际运动是否符合预期：`TODO(human)`
- 异常 / 停止操作：`TODO(human): 无 / 描述`

执行后立即重新运行 8C：

```bash
ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
  require_trajectory_controller_active:=true
```

执行后 8C：`TODO(human): PASS / WARN / BLOCK`

## 9. 单次 READY 验收

只有 HOME 验收成功、执行后 8C 为 PASS，且现场再次确认后，才允许点击一次 `READY`。

记录：

- 点击时间：`TODO(human)`
- GUI request 状态：`TODO(human)`
- `accepted`：`TODO(human)`
- `planned`：`TODO(human)`
- `executed`：`TODO(human)`
- `status`：`TODO(human)`
- `message`：`TODO(human)`
- 是否包含 `final-target gate passed`：`TODO(human): 是 / 否`
- 机械臂实际运动是否符合预期：`TODO(human)`
- 异常 / 停止操作：`TODO(human): 无 / 描述`

执行后再次运行 8C。

执行后 8C：`TODO(human): PASS / WARN / BLOCK`

## 10. 异常处理记录

如出现异常，只记录并停止，不自动恢复：

| 异常 | 是否发生 | 处理与证据 |
|---|---|---|
| `rejected_real_gate` | `TODO(human)` | `TODO(human)` |
| `rejected_delta` | `TODO(human)` | `TODO(human)` |
| `planning_failed` | `TODO(human)` | `TODO(human)` |
| `execution_failed` | `TODO(human)` | `TODO(human)` |
| final-target gate failed | `TODO(human)` | `TODO(human)` |
| controller inactive | `TODO(human)` | `TODO(human)` |
| External Control 生命周期不一致 | `TODO(human)` | `TODO(human)` |
| protective / safeguard stop | `TODO(human)` | `TODO(human)` |

若 External Control 生命周期异常，必须由现场人员确认 Remote Control 与安全状态，再按 Task 8E runbook 处理；禁止 GUI 自动执行 Dashboard 恢复命令。

## 11. 收尾

- 停止 GUI。
- 停止 named controller。
- 根据现场策略停止 External Control program。
- 停止 MoveIt。
- 停止 8B bringup。
- 保存终端日志和本记录。
- 不因测试结束自动补发 `home` 或 `ready`。

## 12. 最终结论

- HOME GUI 验收：`TODO(human): PASS / FAIL / NOT RUN`
- READY GUI 验收：`TODO(human): PASS / FAIL / NOT RUN`
- 每次执行后 8C：`TODO(human)`
- 是否出现无法解释的异常：`TODO(human): 否 / 是`
- 是否允许后续复测：`TODO(human): 是 / 否`
- 总结：`TODO(human)`
