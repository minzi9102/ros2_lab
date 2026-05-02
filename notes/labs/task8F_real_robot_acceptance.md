# Task 8F：执行日志、操作规程与阶段验收

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[x] 已完成`

## 1. 目标
- 收口阶段 4 的真机接入学习。
- 整理日志规范、操作规程、验收证据和边界声明。
- 明确当前代码仍是学习实验入口，不是生产控制系统。

## 2. 相关文件
- 阶段计划：`notes/plans/archive/stage4_learning_plan.md`
- 子任务计划：
  - `notes/plans/tasks/task8A_plan.md`
  - `notes/plans/tasks/task8B_plan.md`
  - `notes/plans/tasks/task8C_plan.md`
  - `notes/plans/tasks/task8D_plan.md`
  - `notes/plans/tasks/task8E_plan.md`
  - `notes/plans/tasks/task8F_plan.md`
- 骨架包：
  - `workspaces/ws_stage4/src/ur3_real_bringup_lab`
  - `workspaces/ws_stage4/src/ur3_real_guarded_motion_lab_cpp`

## 3. 当前准备情况
- 已准备：
  - 阶段 4 两个包骨架；
  - 8A-8F 记录模板；
  - 阶段验收清单。
- 已完成：
  - 汇总 8A-8E 证据；
  - 写出操作规程；
  - 判断阶段 4 首轮最小真机接入验收通过。

## 4. 日志目录规范填写区
- 本阶段日志根目录：`notes/labs/task8A_real_robot_preflight.md` 到 `notes/labs/task8F_real_robot_acceptance.md`；代码与运行入口位于 `workspaces/ws_stage4`
- preflight 日志目录：`notes/labs/task8A_real_robot_preflight.md`
- readonly bringup 日志目录：`notes/labs/task8B_real_robot_readonly_bringup.md`
- guarded motion 日志目录：`notes/labs/task8D_guarded_home_ready_motion.md`
- fault review 日志目录：`notes/labs/task8E_safe_stop_and_fault_handling.md`
- 命名规则是否足够清晰：`足够；按 Task 8A-8F 分文件记录，关键运行证据写入对应任务文档；后续若增加 rosbag 或终端原始日志，应继续使用 task编号_日期_场景 命名`

## 5. 单次动作日志字段复核

| 字段 | 是否已记录 | 证据位置 |
|---|---|---|
| timestamp | `已记录` | `8A 记录日期 2026.4.28；8D 单次执行前记录 2026-04-29 21:26:41 CST；8E 完成记录 2026-04-29` |
| operator / observer | `已记录` | `8A 操作者=催眠剂，旁站/安全确认者=赤眉军；保护停止恢复责任人=采棉机；8D 记录为用户现场操作、Codex 只读复核与记录` |
| robot_id / robot_ip | `已记录` | `8A/8B：ur3e，robot_ip=192.168.56.101，ROS PC=192.168.56.2/24` |
| target_name | `已记录` | `8D：ready/home/ready；8E：out_of_range_test、ready、home cancel` |
| current_joint_state | `已记录` | `8D 当前姿态与单次执行前记录；8E cancel 起点读取` |
| target_joint_state | `已记录` | `8D home/ready 点位审核填写区` |
| joint_delta | `已记录` | `8D dry-run 与单次执行前记录；ready 仅 wrist_3_joint +0.05 rad，max_joint_delta_rad=0.10` |
| controller_state | `已记录` | `8B controller 列表；8C 状态门闩矩阵；8D/8E scaled_joint_trajectory_controller active/inactive 复核` |
| dashboard_state | `已记录` | `8C Dashboard 状态记录；8E Remote Control 恢复结果` |
| precheck_result | `已记录` | `8A 阻断项检查；8C pass/warn/block；8D 动作前门闩从 BLOCK 到 WARN/PASS 的记录` |
| execution_result | `已记录` | `8D ready/home/ready 最终位置门闩通过；8E 低风险拒绝路径与低速 cancel 结果` |
| final_joint_state | `已记录` | `8D 追加手动验证 final-target gate；8E cancel 后最终 joint state 与 reviewed home envelope` |

## 6. 操作规程草案填写区
- 启动前流程：`确认工作空间清空、急停可达、Reduced 配置存在、操作者/旁站/恢复责任人明确；确认机器人 ur3e、robot_ip=192.168.56.101、ROS PC=192.168.56.2/24、ping 0% 丢包；使用 lowlatency/realtime/performance 基线；确认 calibration 文件接入并无 mismatch`
- 执行前流程：`启动 task8B wrapper；确认 External Control 运行；确认 robot_mode=RUNNING、safety_mode=NORMAL、program_running=true、/joint_states 约 500 Hz、scaled_joint_trajectory_controller=active；运行 8C 动作前门闩；执行 8D dry-run；核对 target_name、当前关节、目标关节、delta、速度/加速度/最短时间；最后输入人工确认 token`
- 执行中观察项：`观察 /joint_states 持续更新、controller_state reference/feedback/error、speed_scaling topic 与 Dashboard program_running；只执行一个 FollowJointTrajectory goal；出现 action result 后仍以 /joint_states final-target gate 判定真实到位`
- 停止条件：`8C BLOCK；缺少人工确认 token；target_name 非 home/ready；delta 超限；/joint_states 缺失或超时；controller inactive；robot_mode/safety_mode 异常；action success 但 final-target gate 失败；External Control 与 controller 生命周期不一致且无法解释`
- 异常后流程：`不自动补发 goal，不自动 unlock_protective_stop/restart_safety/brake_release/power_on；先停止脚本并记录日志；External Control 生命周期残留时，必须在现场确认 Remote Control 与安全后由 PC stop/load_program/play，再激活 controller 并重跑 8C；protective stop/safeguard stop/driver 断连只按 8E runbook 人工恢复`
- 收尾流程：`cancel 后不自动回 home/ready；是否回 home 由现场人工另行确认；关闭 bringup 前优先同步 stop External Control program；保存 8C/8D/8E 关键日志；若出现无法解释的 safety/controller/joint state 异常，终止当天运动实验`

## 7. 阶段 4 验收清单

| 验收项 | 证据 | 状态 |
|---|---|---|
| 安全连接 UR3 真机 | `8A：现场安全、Reduced 配置、急停可达、网络路由修复后 ping 0% 丢包；允许进入只读 bringup` | `通过` |
| 只读状态流正常 | `8B：真实 driver 可连接；/joint_states 约 500 Hz；controller manager、TF、External Control 状态可观察；未发送运动命令` | `通过` |
| Dashboard / controller 门闩明确 | `8C：robot_mode=RUNNING，safety_mode=NORMAL，program_running=true，/joint_states 约 465-503 Hz；trajectory controller inactive 时动作前门闩 BLOCK；脚本不执行恢复类 Dashboard 操作` | `通过` |
| 执行低速 home / ready 点 | `8D：calibration 接入后 checksum 通过；ready/home/ready 在 Remote Control + PC 管理 External Control 生命周期后 final-target gate 均通过` | `通过` |
| 异常时拒绝执行或停机 | `8E：目标名非法、缺少人工确认 token、delta 过大、/joint_states 缺失、controller inactive 均拒绝且未发送 goal；低速 cancel 后最终状态仍在 reviewed home envelope 内` | `通过` |
| 操作规程可复用 | `8F 已汇总启动前、执行前、执行中、停止、异常后、收尾流程；8E 已记录 Remote Control 恢复 runbook` | `通过` |
| 边界声明清楚 | `8E/8F 明确 protective stop、safeguard stop、brake release、power cycle、restart safety 不主动实测且不自动恢复；当前代码是实验入口` | `通过` |

## 8. 边界声明
- 当前已经验证的能力：`安全网络接入 UR3e；真实 driver 只读状态流；Dashboard/controller/External Control 门闩；calibration 接入；低速小范围 home/ready 关节空间动作；执行前拒绝路径；External Control 生命周期恢复；低速 cancel 记录`
- 当前仍未验证的能力：`protective stop/safeguard stop 的真实触发与恢复；driver 物理断连恢复；长时间稳定性；复杂轨迹、笛卡尔路径、MoveIt 真机规划执行、Servo 真机控制；多目标连续任务`
- 当前明确不支持的能力：`自动 unlock_protective_stop、restart_safety、brake_release、power_on/power_off/shutdown；无人值守执行；自动从异常中继续补发 goal；超出 home/ready 的任意目标执行；把 action success 当作唯一到位判据`
- 哪些代码只是实验入口：`task8B_readonly_bringup.launch.py、task8C_state_check.launch.py、task8D_guarded_home_ready.launch.py、task8E_fault_review.launch.py、guarded_joint_motion_node.cpp 均服务于阶段 4 学习实验，不是生产控制系统`
- 若要成为控制系统，还缺哪些工程能力：`正式风险评估；硬件级安全链路和权限模型；可审计日志/rosbag/事件存档；系统级状态机；异常恢复审批流；独立 watchdog；更完整的测试矩阵；参数版本管理；长期稳定性与回归验证`

## 9. 经验总结
- 本阶段最关键的安全经验：`真机动作不能只看 action result；必须把 Dashboard、controller、/joint_states、calibration、人工确认和 final-target gate 串成门闩链`
- 本阶段最容易误判的现象：`program_running=true、speed_scaling=100.0 或 action success 都不单独等价于“机器人真实可动且已到位”；External Control 程序生命周期可能和 PC 端 controller 生命周期脱节`
- 对阶段 2/3 仿真经验的修正：`仿真里 controller/action 成功通常足够解释结果；真机上还要处理 Remote Control、External Control、calibration checksum、实时调度、最终关节反馈和现场恢复责任`
- 后续最值得单开的任务：`整理阶段 4 原始日志/rosbag 规范；单开 Remote Control 生命周期管理；单开真机 MoveIt/Servo 前置风险评审；单开生产级安全状态机设计`

## 10. 完成标准
- 8A-8E 的核心证据已汇总。
- 阶段验收有明确通过 / 未通过结论。
- 文档清楚区分实验脚本与控制系统。
- 后续任务边界明确。

## 11. 完成记录
- 日期：`2026-05-02`
- 阶段 4 是否通过验收：`通过；仅限首轮最小真机接入与 guarded home/ready 验收`
- 最终结论：`阶段 4 已完成从现场 preflight、只读 bringup、状态门闩、低速 home/ready 动作到异常拒绝/cancel/runbook 的闭环；当前实验入口足以支撑后续学习任务，但不能作为生产控制系统使用`
- 下一步：`先归档原始终端日志或 rosbag 命名规范；随后单开阶段 5 前置任务，评审是否进入 MoveIt 真机规划、Servo 真机控制或更严格的安全状态机实现`
