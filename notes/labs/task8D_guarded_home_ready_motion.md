# Task 8D：低速小范围 home / ready 关节动作

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[#] 方案完善中，等待人工审批后才进入 8D 执行`

## 1. 目标
- 在 8A-8C 通过后，执行第一组低速、小范围、人工确认过的真机关节空间动作。
- 只做 home / ready 点，不做复杂规划、笛卡尔路径或 Servo。
- 每次动作都记录目标、当前状态、delta、状态门闩、人工确认和执行结果。

## 2. 应用包与文件位置
- 骨架包：`workspaces/ws_stage4/src/ur3_real_guarded_motion_lab_cpp`
- config：`config/task8D_guarded_targets.yaml`
- launch：`launch/task8D_guarded_home_ready.launch.py`
- 节点：`src/guarded_joint_motion_node.cpp`
- 任务计划：`notes/plans/tasks/task8D_plan.md`

## 3. 当前准备情况
- 已准备：
  - 受限动作 C++ 节点骨架；
  - 默认 dry-run launch；
  - home / ready 点配置模板；
  - 本记录模板。
- 8D 前置风险：
  - calibration 已提取并归档，但进入动作前必须确认已接入当前 driver / description 链路，且不再出现 calibration mismatch；
  - 实时调度 / overrun 已完成阶段性修复，但进入动作前仍需确认 driver 日志中 FIFO warning 消失、overrun 未频繁刷屏；
  - 当前 `remote_control=false`，本阶段策略是由人类在示教器上启动 External Control，ROS 端不远程 load/play 程序；
  - 当前 `scaled_joint_trajectory_controller` 在 8C 只读模式下保持 inactive，动作前门闩会因此 block，只有审批后才允许讨论显式激活。
- 当前修复 / 记录位置：
  - calibration 归档：`docs/calibration/ur3e_real_calibration.yaml`
  - calibration 运行副本：`workspaces/ws_stage4/src/ur3_real_bringup_lab/config/ur3e_real_calibration.yaml`
  - 实时调度 runbook：`notes/runbooks/real_time_overrun_mitigation.md`
- 待你完成：
  - 审批是否允许进入 8D；
  - 审核并填写 home / ready 关节目标；
  - 判断速度、加速度、delta 上限；
  - 现场确认是否允许执行。

## 3.1 当前状态记录（2026-04-28）
- calibration 文件状态：`已提取；docs/calibration 与 ws_stage4 config 副本一致；hash=calib_9781467669625414396`
- calibration 接入状态：`待 8D 审批后在 bringup/description wrapper 中确认；未确认前不进入真实动作`
- 实时内核：`6.8.0-110-lowlatency`
- realtime 权限：`minzi 在 realtime 组；ulimit -r=99；ulimit -l=unlimited；chrt -f 50 true 通过`
- 电源策略：`powerprofilesctl=performance；12 个 CPU energy_performance_preference=performance；intel_pstate active 下 scaling_governor 仍显示 powersave，按 runbook 不单独判失败`
- 机器人网络：`192.168.56.101 dev enp7s0 src 192.168.56.2`
- 8C 只读门闩：`WARN，可用于状态观察与方案评审`
- 8C 动作前门闩：`BLOCK，阻断项为 scaled_joint_trajectory_controller inactive`
- 本轮审批状态：`已审批通过`

## 3.2 当前关节状态记录（2026-04-29）
- `/joint_states` 读取结果：`已读取 1 条完整 JointState；首次 echo 提示 A message was lost，但有效消息完整`
- `/joint_states` 频率：`约 499.8-500.0 Hz，窗口 3502 条时仍稳定`
- 原始消息关节顺序：`elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint`
- 8D 配置重排顺序：`shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint`
- 重排后的当前姿态 rad：`[1.537635326385498, -1.6185537777342738, 1.408759895955221, -2.9421216450133265, -1.5928295294391077, -0.09980899492372686]`

## 4. 构建与 dry-run
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_real_guarded_motion_lab_cpp
source install/setup.bash

ros2 launch ur3_real_guarded_motion_lab_cpp task8D_guarded_home_ready.launch.py \
  execute:=false \
  target_name:=ready
```

### dry-run 记录
- 是否构建通过：`2026-04-29 通过，ur3_real_guarded_motion_lab_cpp 构建成功`
- dry-run 是否运行：`2026-04-29 通过，execute:=false target_name:=ready 与 target_name:=home 均通过`
- 是否确认未发送 goal：`已确认未发送 FollowJointTrajectory goal`
- 输出的关键日志：`ready delta_vector_rad=[0, 0, 0, 0, 0, 0.05]；home delta_vector_rad=[0, 0, 0, 0, 0, 0]；Delta gate passed；Dry-run only. No FollowJointTrajectory goal will be sent.`

### dry-run 目标读取与 delta 检查
- 目标读取：`节点已读取 home_joint_names/home_positions_rad/ready_positions_rad/reviewed_by`
- ready 检查：`wrist_3_joint delta=0.05 rad，其余关节 delta=0`
- home 检查：`所有关节 delta=0`
- delta 门闩：`max_joint_delta_rad=0.10；home 与 ready 均通过`
- 执行边界：`本轮仍不实现 FollowJointTrajectory Action Client，不发送真实动作 goal`

### 进入 8D 前必须重新执行的只读检查
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
  require_trajectory_controller_active:=true
```

预期：审批前该门闩仍可因 trajectory controller inactive 而 `BLOCK`；只有在 8D 审批允许激活 controller 后，才允许重新评估。

### 8C 动作前门闩复核（2026-04-29）
- 复核命令：`ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py require_trajectory_controller_active:=true`
- 总结论：`BLOCK`
- 通过项：
  - `robot_mode=RUNNING`
  - `safety_mode=NORMAL`
  - `External Control program_running=true`
  - `joint_state_broadcaster=active`
  - `/joint_states=503.0 Hz，收到 1520 条样本`
  - `speed_scaling=100.0`
- warning 项：`remote_control=false；本阶段仍按示教器人工启动 External Control 策略处理，ROS 端不假设远程 load/play`
- block 项：`scaled_joint_trajectory_controller=inactive；动作前要求 active`
- controller 复核：`ros2 control list_controllers` 同步显示 `scaled_joint_trajectory_controller inactive`
- calibration 复核：`当前 ros2_control_node 日志仍出现 calibration mismatch；连接机器人 checksum=calib_16756443741236045476，说明当前 bringup/description 仍未正确使用已归档 calibration`
- 实时调度复核：`lowlatency 内核；minzi 在 realtime 组；ulimit -r=99；ulimit -l=unlimited；chrt -f 50 true 通过；powerprofilesctl=performance；12 个 CPU EPP=performance`
- driver 日志复核：`Successful set up FIFO RT scheduling policy with priority 50；SCHED_FIFO OK, priority 99`
- overrun 复核：`启动期间出现一次 controller switch overrun；未在本次复核中观察到持续刷屏，但激活 trajectory controller 后仍需重新观察`
- 当前执行结论：`不得进入 execute:=true；先处理 calibration mismatch，并在明确策略后激活 scaled_joint_trajectory_controller 再重跑门闩`

### calibration 接入与 controller 激活后复核（2026-04-29）
- calibration 接入方式：`task8B_readonly_bringup.launch.py` 使用 `task8B_real_calibrated_rsp.launch.py` 作为 description launchfile，并将 `config/ur3e_real_calibration.yaml` 传给官方 `ur_rsp.launch.py` 的 `kinematics_params_file`
- 重启命令：`ros2 launch ur3_real_bringup_lab task8B_readonly_bringup.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 reverse_ip:=192.168.56.2 launch_rviz:=false activate_joint_controller:=true`
- calibration 结果：`ros2_control_node 日志显示 Calibration checksum: calib_9781467669625414396；Calibration checked successfully`
- controller 激活过程：`launch 阶段 scaled_joint_trajectory_controller 曾成功激活，但随后 controller_stopper 将其 deactive；在 External Control running 后手动执行 ros2 control switch_controllers --activate scaled_joint_trajectory_controller --strict，返回 Successfully switched controllers`
- controller 复核：`ros2 control list_controllers 显示 scaled_joint_trajectory_controller active`
- 8C 门闩复核命令：`ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py require_trajectory_controller_active:=true`
- 8C 门闩复核结论：`WARN`
- pass 项：`robot_mode=RUNNING；safety_mode=NORMAL；External Control program_running=true；joint_state_broadcaster=active；scaled_joint_trajectory_controller=active；/joint_states=503.1 Hz；speed_scaling=100.0`
- warning 项：`remote_control=false；继续按示教器人工启动 External Control 策略处理，ROS 端不远程 load/play`
- overrun 复核：`启动阶段仍出现一次 controller switch overrun；手动激活 scaled_joint_trajectory_controller 后未观察到新的持续 overrun 刷屏`
- 当前执行结论：`8C 动作前门闩已从 BLOCK 变为 WARN；真实执行前仍需单次执行前记录与人工确认`

## 5. 点位审核填写区
- home 目标来源：`2026-04-29 /joint_states 当前姿态，按 8D joint_names 顺序重排`
- home 目标关节值（rad）：`[1.537635326385498, -1.6185537777342738, 1.408759895955221, -2.9421216450133265, -1.5928295294391077, -0.09980899492372686]`
- ready 目标来源：`home 基础上仅 wrist_3_joint +0.05 rad`
- ready 目标关节值（rad）：`[1.537635326385498, -1.6185537777342738, 1.408759895955221, -2.9421216450133265, -1.5928295294391077, -0.04980899492372686]`
- 点位审核人：`用户现场确认`
- 最大单关节 delta：`0.10 rad，人工判断足够保守`
- 速度限制：`0.10，人工判断足够保守`
- 加速度限制：`0.10，人工判断足够保守`
- 最短执行时间：`5.0 s，人工判断足够保守`
- 你选择这些值的理由：`首轮真机动作只验证 home / ready 小范围关节空间动作；当前状态流稳定，home 取当前姿态，ready 仅让 wrist_3_joint 小幅移动 0.05 rad，小于 0.10 rad delta 门闩。`

### 方案审批填写区
- 是否批准进入 8D：`批准`
- 审批人：`用户现场确认`
- 审批时间：`2026-04-29`
- calibration 接入确认：`已确认`
- 实时调度确认：`沿用 2026-04-28 lowlatency / realtime / performance 记录，执行前仍需复核 driver 日志`
- Remote Control 策略确认：`示教器人工启动 External Control，ROS 端不远程 load/play`
- 是否允许显式激活 `scaled_joint_trajectory_controller`：`允许`
- 是否允许 `execute:=true`：`允许；但必须先完成具体点位落表、8C 动作前门闩和单次执行前记录`

## 6. 单次执行前记录

| 项目 | 内容 |
|---|---|
| 时间 | `2026-04-29 21:26:41 CST` |
| 操作者 | `用户现场操作；Codex 只读复核与记录` |
| 旁站确认 | `用户确认现场安全，允许实现并执行 8D 单次 ready goal` |
| 8D 审批状态 | `已审批通过；允许单次 ready goal` |
| calibration 文件 | `workspaces/ws_stage4/src/ur3_real_bringup_lab/config/ur3e_real_calibration.yaml` |
| calibration mismatch 是否消失 | `是；日志显示 Calibration checked successfully，checksum=calib_9781467669625414396` |
| 实时调度状态 | `lowlatency；SCHED_FIFO OK；powerprofilesctl=performance；CPU EPP=performance` |
| Remote Control 策略 | `remote_control=false；示教器人工启动 External Control；ROS 端不远程 load/play` |
| 目标名 | `ready` |
| 当前 joint state | `[1.5376816987991333, -1.6184712849059046, 1.4087899366961878, -2.942134996453756, -1.5928576628314417, -0.09972602525819951]` |
| 目标 joint state | `[1.537635326385498, -1.6185537777342738, 1.408759895955221, -2.9421216450133265, -1.5928295294391077, -0.04980899492372686]` |
| 每关节 delta | `配置 home->ready: [0, 0, 0, 0, 0, 0.05]；当前->ready 约 [-0.000046, -0.000082, -0.000030, 0.000013, 0.000028, 0.049917]` |
| 8C 状态门闩结果 | `WARN；唯一 warning 为 remote_control=false` |
| controller 状态 | `scaled_joint_trajectory_controller=active；joint_state_broadcaster=active；speed_scaling_state_broadcaster=active` |
| robot / safety / program 状态 | `robot_mode=RUNNING；safety_mode=NORMAL；External Control program_running=true；speed_scaling=100.0` |
| 人工确认 | `已输入确认 token；用户确认现场安全` |

## 7. 真实执行记录
> 只有 8C 通过、点位审核完成、现场人工确认后，才允许填写并执行本节。

执行命令（执行前再次确认）：

```bash
ros2 launch ur3_real_guarded_motion_lab_cpp task8D_guarded_home_ready.launch.py \
  execute:=true \
  require_confirmation:=true \
  human_confirmation:=I_CONFIRM_REAL_ROBOT_MOTION \
  target_name:=ready
```

- 是否真实执行：`是，已发送 1 次 ready goal`
- goal 是否发送：`是；节点日志显示 Sending exactly one FollowJointTrajectory point`
- goal 是否 accepted：`是；Goal accepted by controller`
- result：`Action result: status=4 error_code=0 error_string='Goal successfully reached!'`
- 最终 joint state：`[1.5375601053237915, -1.6185037098326625, 1.408731762562887, -2.9421502552428187, -1.592776123677389, -0.0997846762286585]`
- 与目标误差：`[-0.000075, 0.000050, -0.000028, -0.000029, 0.000053, -0.049976]`
- 执行中是否出现异常：`是；action result 返回成功，但 /joint_states 显示 wrist_3_joint 未到 ready 目标。controller_state 中 reference 为 ready，feedback 仍接近 home，error wrist_3_joint 约 0.050020 rad，speed_scaling_factor=0.0。`
- 异常后补充观测：`/speed_scaling_state_broadcaster/speed_scaling=100.0；/dashboard_client/program_running 返回 true；/scaled_joint_trajectory_controller/controller_state 仍显示 speed_scaling_factor=0.0。`
- 你的观察：`不把本次记录为真实到位成功；停止重试，先进入异常复盘。`

执行后补强：

- 节点新增最终位置门闩：action result 后重新读取 `/joint_states`。
- 默认 `final_position_tolerance_rad=0.02`。
- 任一关节 `final_minus_target` 超过阈值时，8D 节点返回失败。

## 8. 你需要完成的判断
- 这次动作是否足够保守：`点位与速度设置足够保守；但控制链路反馈异常，需要复盘`
- 是否允许进行第二个目标：`不允许`
- 是否需要进入 8E 复盘异常：`需要`

## 9. 完成标准
- dry-run 可运行且不发送 goal。
- 点位、delta、速度、加速度经过人工审核。
- 如真实执行，每次动作都有完整日志。
- action result 成功后必须再以 `/joint_states` 做最终到位复核。

## 10. 完成记录
- 日期：`2026-04-29`
- 最终结论：`8D 单次 ready goal 已发送，但最终位置复核未通过；本次不验收为到位成功`
- 下一步：`进入 8E，优先排查 speed_scaling_factor=0.0、External Control/控制器执行链路与 why action success does not imply motion`
