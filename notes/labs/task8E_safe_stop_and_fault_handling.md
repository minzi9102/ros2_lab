# Task 8E：异常场景与安全停机逻辑

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[#] 进行中`

## 1. 目标
- 整理并验证异常情况下的拒绝执行、cancel、停止 program 与人工恢复边界。
- 建立异常处理矩阵。
- 明确恢复类操作不进入默认自动化流程。

## 2. 应用包与文件位置
- 骨架包：`workspaces/ws_stage4/src/ur3_real_guarded_motion_lab_cpp`
- launch：`launch/task8E_fault_review.launch.py`
- 任务计划：`notes/plans/tasks/task8E_plan.md`

## 3. 当前准备情况
- 已准备：
  - dry-run 异常复盘 launch；
  - 异常矩阵模板；
  - 本记录模板。
- 待你完成：
  - 判断哪些异常可以现场验证；
  - 记录拒绝执行与人工恢复路径；
  - 明确哪些操作禁止自动化。

## 4. 执行前约束
- 是否完成 8D 最小动作：`是；ready/home/ready 最终位置门闩均通过`
- 是否有现场人工确认：`是；现场人工切换 Remote Control 并确认安全`
- 本轮是否只验证低风险异常：`是；只验证 External Control 程序生命周期、controller 状态与 8C 门闩，不主动触发保护停机`
- 是否禁止自动 unlock / restart safety：`是`

## 5. 低风险拒绝路径验证
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ur3_real_guarded_motion_lab_cpp task8E_fault_review.launch.py
```

- 是否运行 dry-run fault review：`是；task8E_fault_review.launch.py 已运行`
- 是否确认未发送 goal：`是；日志中没有 "Sending exactly one FollowJointTrajectory point"`
- 触发的拒绝原因：`target_name=out_of_range_test 不属于 home / ready`
- 日志摘要：`execute=false；Unsupported target_name=out_of_range_test. Expected home or ready.；进程 exit code 1`

补充拒绝路径：

```bash
ros2 launch ur3_real_guarded_motion_lab_cpp task8D_guarded_home_ready.launch.py \
  execute:=true \
  require_confirmation:=true \
  target_name:=ready
```

- 是否确认未发送 goal：`是；拒绝发生在 action client 前，日志中没有发送 FollowJointTrajectory point`
- 触发的拒绝原因：`缺少 human_confirmation=I_CONFIRM_REAL_ROBOT_MOTION`
- 日志摘要：`Delta gate passed 后立即 Execution rejected: human_confirmation must be I_CONFIRM_REAL_ROBOT_MOTION after现场确认.`

补充 delta 拒绝路径：

```bash
ros2 launch ur3_real_guarded_motion_lab_cpp task8D_guarded_home_ready.launch.py \
  execute:=false \
  target_name:=ready \
  max_joint_delta_rad:=0.01
```

- 是否确认未发送 goal：`是；delta gate 阶段拒绝，尚未进入 execute 分支`
- 触发的拒绝原因：`ready 的 wrist_3_joint delta=0.050000 rad，大于临时阈值 0.010000 rad`
- 日志摘要：`delta[wrist_3_joint]=0.050000 rad (block)；Delta gate failed`

补充 `/joint_states` 缺失拒绝路径：

```bash
ros2 run ur3_real_guarded_motion_lab_cpp guarded_joint_motion_node --ros-args \
  --params-file install/ur3_real_guarded_motion_lab_cpp/share/ur3_real_guarded_motion_lab_cpp/config/task8D_guarded_targets.yaml \
  -p execute:=true \
  -p require_confirmation:=true \
  -p human_confirmation:=I_CONFIRM_REAL_ROBOT_MOTION \
  -p target_name:=ready \
  -p joint_state_topic:=/joint_states_missing_task8e \
  -p joint_state_timeout_sec:=1.0
```

- 是否确认未发送 goal：`是；当前状态流门闩超时，尚未等待 action server`
- 触发的拒绝原因：`无法在 1.0s 内从 /joint_states_missing_task8e 读取完整 JointState`
- 日志摘要：`Timed out waiting for complete current joint state.；进程 exit code 1`

## 6. 异常处理矩阵

| 异常 | 检测阶段 | 本轮是否验证 | 默认动作 | 是否人工确认 | 是否允许自动恢复 | 记录 |
|---|---|---|---|---|---|---|
| 目标名非法 | 执行前 | `已验证` | 拒绝执行 | 否 | 否 | `out_of_range_test 被拒绝；未发送 goal` |
| 目标越界 | 执行前 | `未实测` | 拒绝执行 | 否 | 否 | `当前 8D 仅允许 home / ready 命名目标，任意未知目标先按目标名非法拒绝` |
| delta 过大 | 执行前 | `已验证` | 拒绝执行 | 否 | 否 | `max_joint_delta_rad 临时设为 0.01；ready wrist_3_joint delta=0.05 被 block；未发送 goal` |
| 缺少人工确认 token | 执行前 | `已验证` | 拒绝执行 | 是 | 否 | `execute=true 且 require_confirmation=true，但未传 token；delta gate 后拒绝，未发送 goal` |
| `/joint_states` 过期 / 缺失 | 执行前 | `已验证` | 拒绝执行 | 否 | 否 | `joint_state_topic 指向缺失话题；1.0s 超时；未发送 goal` |
| controller inactive | 执行前 | `已验证` | 拒绝执行 | 是 | 否 | `Local 模式下 External Control 显示 PLAYING、speed_scaling=100.0，但 scaled_joint_trajectory_controller=inactive，8C BLOCK` |
| External Control 未运行 / 生命周期残留 | 执行前 | `已验证恢复路径` | 拒绝执行，人工切 Remote Control 后由 PC stop/load/play | 是 | 否 | `Remote Control=true 后 PC 调用 stop/load_program/play 成功，8C 恢复 PASS` |
| Action 执行异常 | 执行中 | `已观察` | cancel / 停止观察 | 是 | 否 | `早期 ready goal 出现 action success 但 /joint_states 未到位；已增加 final-target gate` |
| 低速 cancel | 执行中 | `已验证` | cancel goal，记录结果和最终关节状态 | 是 | 否 | `ready->home 5s 轨迹，1s 后 cancel；action result CANCELED；最终状态仍在 reviewed home envelope 内` |
| protective stop | 任意阶段 | `【请填写】` | 停止脚本，人工处理 | 是 | 否 | `【请填写】` |
| driver 断连 | 任意阶段 | `【请填写】` | 停止脚本，记录日志 | 是 | 否 | `【请填写】` |

## 7. 人工恢复 runbook 填写区
- protective stop 后由谁判断恢复：`【请填写】`
- safeguard stop 后由谁判断恢复：`【请填写】`
- External Control program 停止后如何处理：`先停止 8D；确认示教器处于 Remote Control；PC 端依次调用 /dashboard_client/stop、/dashboard_client/load_program(filename='/programs/external_control.urp')、/dashboard_client/play；随后激活 scaled_joint_trajectory_controller 并重跑 8C`
- 哪些 Dashboard 服务只允许人工调用：`unlock_protective_stop、restart_safety、brake_release、power_on、power_off、shutdown；stop/load_program/play 也必须在现场人工确认 Remote Control 与安全后调用`
- 哪些情况必须终止当天实验：`protective stop/safeguard stop 原因不明；robot_mode 或 safety_mode 异常；/joint_states 丢失或频率异常；final-target gate 失败后无法解释；Remote Control 与 External Control 状态反复不一致`

## 7.1 External Control 生命周期复现与恢复

复现现象：

- 操作：`PC 端关闭 bringup，但示教器上的 External Control 未同步停止；随后重新启动 PC bringup`
- 观测：`示教器不一定报错；/dashboard_client/program_running=true；/speed_scaling_state_broadcaster/speed_scaling=100.0`
- 阻塞点：`scaled_joint_trajectory_controller=inactive；8C require_trajectory_controller_active:=true 返回 BLOCK`
- 解释：`External Control 程序状态与 PC 端 controller 生命周期脱节；不能只凭 program_running=true 或 speed_scaling=100.0 判断可运动`

Remote Control 恢复步骤：

```bash
ros2 service call /dashboard_client/is_in_remote_control \
  ur_dashboard_msgs/srv/IsInRemoteControl {}

ros2 service call /dashboard_client/stop std_srvs/srv/Trigger {}

ros2 service call /dashboard_client/load_program ur_dashboard_msgs/srv/Load \
  "{filename: '/programs/external_control.urp'}"

ros2 service call /dashboard_client/play std_srvs/srv/Trigger {}

ros2 control switch_controllers \
  --activate scaled_joint_trajectory_controller \
  --strict

ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
  require_trajectory_controller_active:=true
```

本轮结果：

- `remote_control=True`
- `/dashboard_client/stop`：`success=True, message='Stopped'`
- `/dashboard_client/load_program`：`success=True, answer='Loading program: /programs/external_control.urp, /programs/default.installation'`
- `/dashboard_client/play`：`success=True, message='Starting program'`
- controller 激活：`scaled_joint_trajectory_controller is already active`
- 8C：`robot_mode=RUNNING；safety_mode=NORMAL；program_running=true；remote_control=True；scaled_joint_trajectory_controller=active；/joint_states=502.9 Hz；speed_scaling=100.0；Task 8C gate result: PASS`
- 复核：`2026-04-29 22:16 CST 再次运行 8C，/joint_states=503.2 Hz，Task 8C gate result: PASS`

## 7.2 低速 cancel 路径

执行前状态：

- 8C：`PASS；robot_mode=RUNNING；safety_mode=NORMAL；program_running=true；remote_control=True；scaled_joint_trajectory_controller=active；/joint_states=503.0 Hz；speed_scaling=100.0`
- 当前姿态：`接近 ready；wrist_3_joint=-0.049824062977926076`
- cancel 测试策略：`target_name=home；min_trajectory_duration_sec=5.0；cancel_after_sec=1.0；max_joint_delta_rad=0.10`
- 安全边界：`取消后不要求到达 home，只要求最终姿态仍在 reviewed home envelope 内`

执行命令：

```bash
ros2 launch ur3_real_guarded_motion_lab_cpp task8D_guarded_home_ready.launch.py \
  execute:=true \
  require_confirmation:=true \
  human_confirmation:=I_CONFIRM_REAL_ROBOT_MOTION \
  target_name:=home \
  cancel_after_sec:=1.0
```

结果摘要：

- 起点读取：`current_positions_rad=[1.537635, -1.618500, 1.408689, -2.942183, -1.592838, -0.049828]`
- 起点门闩：`Current-home gate passed；wrist_3_joint current_minus_home=0.049981 rad`
- goal：`Goal accepted by controller`
- cancel：`Cancel test armed；Cancel response: return_code=0 goals_canceling=1`
- action result：`status=5；error_code=0；error_string=''`
- 最终读取：`current_positions_rad=[1.537575, -1.618468, 1.408661, -2.942152, -1.592877, -0.054685]`
- 最终 envelope：`Current-home gate passed；wrist_3_joint current_minus_home=0.045124 rad`
- 节点结论：`Task 8E cancel path finished with final state inside the reviewed home envelope.`

执行后复核：

- `/joint_states`：`[1.5376189947128296, -1.6185304127135218, 1.4087231794940394, -2.9421001873412074, -1.592809025441305, -0.05467397371401006]`
- 8C：`PASS；/joint_states=502.9 Hz；scaled_joint_trajectory_controller=active；speed_scaling=100.0`
- 处理原则：`cancel 后不自动补发 home/ready；是否回 home 由现场人工另行确认`

## 8. 你需要完成的判断
- 本轮异常验证是否足够：`对目标名非法、缺少人工确认 token、delta 过大、/joint_states 缺失、External Control 生命周期残留、controller inactive 与低速 cancel 路径足够；不覆盖保护停机类异常`
- 哪些异常不适合实测，只能写 runbook：`protective stop、safeguard stop、brake release、power cycle、restart safety`
- 8F 收口前还缺哪些证据：`规范化关机顺序；必要时补充 controller_state speed_scaling_factor 与 speed_scaling topic 的差异说明`

## 9. 完成标准
- 至少一个执行前拒绝路径有证据。
- 保护停止等高风险恢复不会被默认自动化。
- 每类异常都有处理动作和人工确认边界。

## 10. 完成记录
- 日期：`2026-04-29`
- 最终结论：`已复现 PC bringup 与示教器 External Control 生命周期脱节导致的 8C BLOCK；Remote Control 模式下由 PC stop/load/play External Control 可恢复到 8C PASS；低速 cancel 路径可解释且最终姿态在 reviewed home envelope 内`
- 下一步：`固化 8D/8F 的标准启动与关机顺序：Remote Control 模式优先由 PC 管理 External Control 生命周期；关闭 bringup 前同步 stop program；cancel 后是否回 home 必须另行人工确认`
