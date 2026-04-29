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

- 是否运行 dry-run fault review：`【请填写】`
- 是否确认未发送 goal：`【请填写】`
- 触发的拒绝原因：`【请填写】`
- 日志摘要：`【请填写】`

## 6. 异常处理矩阵

| 异常 | 检测阶段 | 本轮是否验证 | 默认动作 | 是否人工确认 | 是否允许自动恢复 | 记录 |
|---|---|---|---|---|---|---|
| 目标越界 | 执行前 | `【请填写】` | 拒绝执行 | 否 | 否 | `【请填写】` |
| delta 过大 | 执行前 | `【请填写】` | 拒绝执行 | 否 | 否 | `【请填写】` |
| `/joint_states` 过期 | 执行前 | `【请填写】` | 拒绝执行 | 否 | 否 | `【请填写】` |
| controller inactive | 执行前 | `已验证` | 拒绝执行 | 是 | 否 | `Local 模式下 External Control 显示 PLAYING、speed_scaling=100.0，但 scaled_joint_trajectory_controller=inactive，8C BLOCK` |
| External Control 未运行 / 生命周期残留 | 执行前 | `已验证恢复路径` | 拒绝执行，人工切 Remote Control 后由 PC stop/load/play | 是 | 否 | `Remote Control=true 后 PC 调用 stop/load_program/play 成功，8C 恢复 PASS` |
| Action 执行异常 | 执行中 | `已观察` | cancel / 停止观察 | 是 | 否 | `早期 ready goal 出现 action success 但 /joint_states 未到位；已增加 final-target gate` |
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

## 8. 你需要完成的判断
- 本轮异常验证是否足够：`对 External Control 生命周期残留与 controller inactive 路径足够；不覆盖保护停机类异常`
- 哪些异常不适合实测，只能写 runbook：`protective stop、safeguard stop、brake release、power cycle、restart safety`
- 8F 收口前还缺哪些证据：`规范化关机顺序；Remote Control PC 端启动顺序；8D 最终到位日志归档；必要时补充 controller_state speed_scaling_factor 与 speed_scaling topic 的差异说明`

## 9. 完成标准
- 至少一个执行前拒绝路径有证据。
- 保护停止等高风险恢复不会被默认自动化。
- 每类异常都有处理动作和人工确认边界。

## 10. 完成记录
- 日期：`2026-04-29`
- 最终结论：`已复现 PC bringup 与示教器 External Control 生命周期脱节导致的 8C BLOCK；Remote Control 模式下由 PC stop/load/play External Control 可恢复到 8C PASS`
- 下一步：`固化 8D/8F 的标准启动与关机顺序：Remote Control 模式优先由 PC 管理 External Control 生命周期；关闭 bringup 前同步 stop program`
