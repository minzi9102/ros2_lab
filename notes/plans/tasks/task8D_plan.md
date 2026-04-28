# 任务 8D 规划文档：低速小范围 home / ready 关节动作

## 1. 任务目标
- 在 8A-8C 全部通过后，执行第一组低速、小范围、人工确认过的 UR3 真机关节空间动作。
- 只做 home / ready 点，不做复杂规划、不做笛卡尔路径、不做 Servo。
- 为每次动作记录当前状态、目标、关节 delta、速度限制、执行结果和人工确认。

## 2. 当前基线（来自仓库现状）
- 阶段 2 已完成 FollowJointTrajectory 和 URSim speed scaling。
- 阶段 3 已完成 MoveIt 规划执行，但真机首轮不直接复用复杂规划。
- 8C 已提供 Dashboard、External Control、controller 与状态流门闩：
  - 只读状态门闩为 `WARN`，主要 warning 为 `remote_control=false` 与轨迹控制器 inactive；
  - 动作前门闩为 `BLOCK`，阻断项为 `scaled_joint_trajectory_controller` inactive。
- 真机 calibration 已提取并归档：
  - 归档路径：`docs/calibration/ur3e_real_calibration.yaml`
  - 运行副本：`workspaces/ws_stage4/src/ur3_real_bringup_lab/config/ur3e_real_calibration.yaml`
  - 两份文件当前一致，hash 为 `calib_9781467669625414396`。
- 实时调度与电源策略已完成阶段性修复：
  - 当前内核：`6.8.0-110-lowlatency`
  - `minzi` 已在 `realtime` 组，`ulimit -r = 99`，`ulimit -l = unlimited`
  - `chrt -f 50 true` 通过
  - `powerprofilesctl get = performance`，12 个 CPU 的 `energy_performance_preference = performance`
  - `scaling_governor` 在 `intel_pstate active` 下仍显示 `powersave`，按 runbook 不单独判定为失败。
- 当前用户要求：本轮只完善 8D 方案文档，等待人工审批后才进入 8D 执行。

## 3. 任务范围（单功能约束）
- 包含：
  - 受限 home / ready 关节目标；
  - joint range 与 delta range 检查；
  - 速度、加速度、持续时间保守约束；
  - 人工确认后发送单次 FollowJointTrajectory；
  - 执行结果日志。
- 不包含：
  - MoveIt pose goal；
  - Cartesian Path；
  - Servo / Twist 控制；
  - 循环执行；
  - 自动恢复 protective stop；
  - 非人工确认的无人值守动作。

## 3.1 本轮审批边界
- 本轮允许：
  - 完善 8D 方案；
  - 记录 calibration 与实时调度当前状态；
  - 明确进入 8D 前需要审批的检查项；
  - 保留 dry-run / 状态门闩命令作为后续入口。
- 本轮不允许：
  - 启动真实机器人 driver；
  - 激活 `scaled_joint_trajectory_controller`；
  - 发送 FollowJointTrajectory goal；
  - 修改或执行任何真实动作逻辑；
  - 将“状态可观察”解释为“可以运动”。

## 4. 包设计建议
- 包名：`ur3_real_guarded_motion_lab_cpp`
- 位置：`workspaces/ws_stage4/src/ur3_real_guarded_motion_lab_cpp`
- 目录骨架建议：
  ```text
  ur3_real_guarded_motion_lab_cpp/
  ├── CMakeLists.txt
  ├── package.xml
  ├── config/
  │   └── task8D_guarded_targets.yaml
  ├── launch/
  │   └── task8D_guarded_home_ready.launch.py
  └── src/
      └── guarded_joint_motion_node.cpp
  ```
- 首轮节点默认 dry-run 或 require-confirm，必须显式参数打开执行。

## 5. learn mode 分工
- 智能体负责：
  - 包骨架；
  - 参数读取、日志结构、Action Client 外壳；
  - joint range / delta range 检查脚手架；
  - 默认拒绝执行路径。
- 人类负责：
  - 选择 home / ready 点；
  - 判断目标是否处于现场安全工作空间；
  - 确认速度、加速度、持续时间是否足够保守；
  - 每次动作前进行现场确认。

## 6. 核心学习问题
1. 为什么真机第一组动作应选择关节空间 home / ready，而不是 pose goal？
2. 为什么目标范围检查还不够，必须检查从当前状态到目标的 delta？
3. 为什么默认 dry-run / require-confirm 是真机实验入口的底线？
4. 为什么 Action result 成功也不等于这段逻辑已经是控制系统？

## 7. 实施步骤

### 练习 0：审批前复核，不进入动作
1. 确认 8A-8C 记录均已完成。
2. 确认 calibration 文件已接入后续 bringup / description wrapper；如果尚未接入，只允许继续方案设计或 dry-run。
3. 确认实时调度状态：
   - lowlatency 内核；
   - `SCHED_FIFO` 可用；
   - power profile / EPP 为 performance；
   - driver 启动时 FIFO warning 消失，overrun 不频繁刷屏。
4. 确认 `remote_control=false` 的现场策略：本阶段采用“人类在示教器上启动 External Control，ROS 端不远程 load/play 程序”。
5. 确认动作前门闩仍会在 trajectory controller inactive 时 block。

### 练习 1：准备目标点但不执行
1. 人类给出 home / ready 候选关节值。
2. 记录每个关节目标、单位、来源、现场审核人。
3. 先运行 dry-run，输出目标与当前状态差异。

### 练习 2：实现执行前检查
1. 检查 8C 状态门闩是否通过。
2. 检查真机 calibration 是否已在当前 driver/description 链路生效，不再出现 calibration mismatch。
3. 检查当前 joint state 是否新鲜、关节名是否匹配。
4. 检查目标是否在允许范围内。
5. 检查每个关节 delta 是否小于本任务上限。
6. 检查速度、加速度、执行时长是否保守。

建议动作前最小门闩：
- `robot_mode=RUNNING`
- `safety_mode=NORMAL` 或人工解释过的 `REDUCED`
- `program_running=true`
- `speed_scaling > 0`
- `/joint_states` 新鲜且频率稳定
- `joint_state_broadcaster=active`
- `scaled_joint_trajectory_controller=active`
- calibration mismatch 已消除
- FIFO warning 已消除，overrun 未频繁刷屏
- `remote_control=false` 的情况下，External Control 由示教器人工启动并由现场确认

### 练习 3：人工确认后执行单次小动作
1. 只发送一个目标。
2. 保持现场急停可达。
3. 观察执行中状态与 driver 日志。
4. 执行结束后记录 result、最终 joint state、误差。

### 练习 4：复盘并决定是否允许第二个目标
1. 如果第一目标异常，停止任务并进入 8E。
2. 如果第一目标正常，人工确认是否继续第二个 ready 点。
3. 不进行循环动作。

## 8. 关键命令（执行阶段使用）
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_real_guarded_motion_lab_cpp
source install/setup.bash

# 进入 8D 前再次确认状态门闩；本命令只检查，不会激活 controller
ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
  require_trajectory_controller_active:=true

# dry-run：只检查，不发送 goal
ros2 launch ur3_real_guarded_motion_lab_cpp task8D_guarded_home_ready.launch.py \
  execute:=false \
  target_name:=ready

# 真实执行：必须经过人类现场确认后才允许
ros2 launch ur3_real_guarded_motion_lab_cpp task8D_guarded_home_ready.launch.py \
  execute:=true \
  require_confirmation:=true \
  human_confirmation:=I_CONFIRM_REAL_ROBOT_MOTION \
  target_name:=ready
```

## 9. 执行前记录模板

| 项目 | 内容 |
|---|---|
| 时间 | |
| 操作者 | |
| 旁站确认 | |
| 8D 审批人 | |
| calibration 文件 | `docs/calibration/ur3e_real_calibration.yaml` / 运行副本 |
| calibration mismatch 是否消失 | yes / no |
| 实时调度状态 | lowlatency / SCHED_FIFO / performance |
| Remote Control 策略 | 示教器人工启动 / Dashboard 远程 |
| 目标名 | home / ready |
| 当前 joint state | |
| 目标 joint state | |
| 每关节 delta | |
| controller 状态 | |
| robot / safety / program 状态 | |
| 速度 / 加速度 / 时长 | |
| 人工确认 | yes / no |

## 10. 交付物
- 代码：`workspaces/ws_stage4/src/ur3_real_guarded_motion_lab_cpp`
- 文档：`notes/labs/task8D_guarded_home_ready_motion.md`
- 结果：一次或少量几次可追踪的低速小范围真机动作。

## 11. 验收标准
- dry-run 能输出当前状态、目标、delta 和检查结果。
- 任何检查失败时默认拒绝执行。
- 真实执行前有人类确认记录。
- 真机能以低速完成一个 home / ready 点动作。
- 每次执行都有目标、状态、结果和异常记录。

## 12. 风险与回退
- 风险 1：目标点未经现场审核。
  - 回退：只保留 dry-run，不允许 `execute:=true`。
- 风险 2：当前状态离目标太远。
  - 回退：拒绝执行，重新定义更近的中间点。
- 风险 3：执行中出现 protective stop 或 controller error。
  - 回退：停止发送目标，进入 8E 异常处理记录。
