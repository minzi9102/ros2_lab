# 阶段 4 学习计划：接入真机 UR3

## 1. 计划目标
- 在 `2-4` 周内把阶段 2/3 的 fake hardware、URSim、MoveIt 2 经验迁移到真实 UR3。
- 阶段重点不是“让机器人动起来”，而是“在可解释、可回退、可记录的约束下安全地动起来”。
- 完成真机通信、只读状态验证、低速小范围动作、安全停机逻辑、执行日志规范这几条主线。
- 对齐阶段验收目标：
  - 能安全连接 UR3 真机；
  - 能执行简单 home / ready 点运动；
  - 能在异常时进入明确的停机或拒绝执行路径；
  - 不把一次性实验脚本误当成可复用控制系统。

## 2. 阶段启动基线（来自当前仓库现状）
- 已完成阶段 1：ROS 2 topic、service、action、tf2、QoS、URDF/xacro 与 `ros2_control` 最小链路。
- 已完成阶段 2：
  - `ur_robot_driver` fake hardware；
  - `FollowJointTrajectory` Python / C++ 最小闭环；
  - URSim speed scaling、External Control、`scaled_joint_trajectory_controller` 执行链路排障。
- 已完成阶段 3：
  - MoveIt 2 bringup；
  - MoveGroupInterface C++ 规划与执行；
  - Planning Scene 与简单碰撞约束；
  - RViz 点击目标位姿自动规划；
  - MoveIt Servo fake hardware 最小闭环。
- 当前边界：
  - 尚未接入真实 UR3；
  - Task 7E 的 Servo 仅完成 fake hardware 验收，真机 Servo 不进入阶段 4 首轮目标；
  - 阶段 4 先做低速、短距离、人工可监控的关节空间动作。

## 3. 官方约束与阶段策略

### 3.1 官方约束
- 官方 ROS 2 driver 推荐用 `ur_control.launch.py` 启动真实机器人：
  ```bash
  ros2 launch ur_robot_driver ur_control.launch.py ur_type:=<UR_TYPE> robot_ip:=<IP_OF_THE_ROBOT> launch_rviz:=true
  ```
- 真实机器人启动后，需要在示教器上运行 External Control 程序，ROS 端才会进入可接收控制命令的状态。
- e-Series 部分 Dashboard / script 操作需要机器人处于 Remote Control 模式。
- 官方文档建议为 `universal_robot_driver` 准备具备实时能力的 Ubuntu 系统；低延迟内核在很多场景可用，PREEMPT_RT 是更强的实时方案。

参考：
- `ur_robot_driver` usage: <https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/rolling/doc/ur_robot_driver/ur_robot_driver/doc/usage/toc.html>
- Dashboard client: <https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_robot_driver/ur_robot_driver/doc/dashboard_client.html>
- Real-time scheduling: <https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/real_time.html>

### 3.2 阶段策略
- 主线顺序：`网络/权限/安全检查 -> 只读连接 -> 控制器与 Dashboard 验证 -> 低速小范围动作 -> 异常停机 -> 日志规范`
- 运动策略：
  - 先 `plan_only`，后 `plan_and_execute`；
  - 先关节空间 home / ready 点，后再考虑笛卡尔目标；
  - 先小幅度单次动作，后再做重复验证；
  - 不在阶段 4 首轮做 Servo、遥操作、视觉闭环或自动化长流程。
- 安全策略：
  - 所有会让真机运动的命令都必须有人工确认点；
  - 默认低速、短距离、低加速度；
  - 每次执行前检查当前 joint state、controller 状态、robot mode、safety mode、program state；
  - 任何状态不满足预期时拒绝执行，而不是尝试自动恢复。

## 4. 任务拆分与进度跟踪

### 4.1 进度统计
- 已准备骨架：`0 / 6`
- 已完成实验：`0 / 6`
- 当前状态：阶段 4 计划已创建，尚未进入真机联调。

### 4.2 可勾选任务清单
- [ ] 任务 8A：真机接入前安全与网络清单
- [ ] 任务 8B：只读启动 `ur_control.launch.py` 并验证状态流
- [ ] 任务 8C：Dashboard、controller 与 External Control 状态验证
- [ ] 任务 8D：低速小范围 home / ready 关节动作
- [ ] 任务 8E：异常场景与安全停机逻辑
- [ ] 任务 8F：执行日志、操作规程与阶段验收

## 5. 每个任务的目标范围

### 5.1 任务 8A：真机接入前安全与网络清单
- 目标：
  - 明确 UR3 型号、控制柜版本、PolyScope 版本、机器人 IP、ROS PC IP、网段、网卡。
  - 确认真机周围工作空间、急停按钮、保护停止、示教器权限、Remote Control 设置。
  - 记录当前 ROS PC 内核、网络延迟、CPU 电源策略、是否具备 lowlatency / PREEMPT_RT 条件。
- 交付：
  - 一份真机 preflight checklist；
  - 一份网络与系统基线记录；
  - 明确“哪些条件不满足时禁止进入 8B”。

### 5.2 任务 8B：只读启动 `ur_control.launch.py` 并验证状态流
- 目标：
  - 启动真实机器人 driver，但第一轮只观察，不发送轨迹。
  - 验证 `/joint_states`、TF、controller manager、robot description、driver 日志。
  - 确认 External Control 程序启动前后的 ROS 端状态变化。
- 交付：
  - 只读 bringup 记录；
  - 正常状态与异常状态的日志样例；
  - 一条“可连接但不可运动”的安全分界线。

### 5.3 任务 8C：Dashboard、controller 与 External Control 状态验证
- 目标：
  - 查询 Dashboard 服务：robot mode、safety mode、program state、program running、remote control。
  - 验证 `scaled_joint_trajectory_controller` 等关键控制器状态。
  - 建立执行前状态门闩：只有所有状态满足预期，后续动作任务才允许继续。
- 交付：
  - 状态检查脚本或 runbook；
  - Dashboard / controller / External Control 状态矩阵；
  - 失败时拒绝执行的判据。

### 5.4 任务 8D：低速小范围 home / ready 关节动作
- 目标：
  - 只执行人工确认过的 home / ready 点。
  - 每个目标在发送前做 joint range、delta range、速度、加速度、当前状态检查。
  - 初始动作应低速、短距离、单次执行，执行后记录实际终点与误差。
- 交付：
  - 一个受限的真机关节动作入口；
  - 一组保守 home / ready 目标；
  - 执行前检查、执行结果、失败原因的日志记录。

### 5.5 任务 8E：异常场景与安全停机逻辑
- 目标：
  - 验证常见异常：driver 断开、External Control 停止、controller inactive、protective stop、目标越界、当前状态超出预期。
  - 明确每类异常是“拒绝执行”“停止 program”“取消 goal”还是“人工处理后恢复”。
  - 不把 `unlock_protective_stop`、`restart_safety` 这类恢复操作做成默认自动化流程。
- 交付：
  - 异常处理矩阵；
  - 安全停机或拒绝执行路径；
  - 保护停止后的人工恢复 runbook。

### 5.6 任务 8F：执行日志、操作规程与阶段验收
- 目标：
  - 为每一次真机动作记录目标、当前状态、规划结果、执行结果、Dashboard 状态、操作者确认。
  - 整理阶段 4 操作规程：启动前、执行前、执行中、异常后、收尾。
  - 用最小动作完成阶段验收，不扩大到 Servo、复杂规划或自动化任务。
- 交付：
  - 真机运行日志目录规范；
  - 阶段 4 验收记录；
  - 阶段总结：哪些逻辑可复用，哪些仍只是实验入口。

## 6. 每个任务的规划文档目录
1. `8A` -> `notes/plans/tasks/task8A_plan.md`
2. `8B` -> `notes/plans/tasks/task8B_plan.md`
3. `8C` -> `notes/plans/tasks/task8C_plan.md`
4. `8D` -> `notes/plans/tasks/task8D_plan.md`
5. `8E` -> `notes/plans/tasks/task8E_plan.md`
6. `8F` -> `notes/plans/tasks/task8F_plan.md`

## 7. 每个任务的实验记录文档目录
1. `8A` -> `notes/labs/task8A_real_robot_preflight.md`
2. `8B` -> `notes/labs/task8B_real_robot_readonly_bringup.md`
3. `8C` -> `notes/labs/task8C_dashboard_controller_state.md`
4. `8D` -> `notes/labs/task8D_guarded_home_ready_motion.md`
5. `8E` -> `notes/labs/task8E_safe_stop_and_fault_handling.md`
6. `8F` -> `notes/labs/task8F_real_robot_acceptance.md`

## 8. 工作区与包目录建议
- 工作区：`workspaces/ws_stage4`
- 初始包建议：
  - `ur3_real_bringup_lab`：真机 bringup wrapper、只读状态检查、Dashboard 查询入口。
  - `ur3_real_guarded_motion_lab_cpp`：受限 home / ready 动作、范围检查、执行日志、安全拒绝路径。
- 是否复用 `ws_stage3`：
  - 可以复用阶段 3 的 MoveIt 经验；
  - 不建议把真机逻辑直接塞回 `ws_stage3`，避免 fake hardware / URSim / real robot 的安全边界混在一起。

## 9. 推荐节奏（2-4 周）
- 第 1 周：
  - 完成 `8A` preflight checklist；
  - 完成 `8B` 只读 bringup；
  - 不执行任何真机运动。
- 第 2 周：
  - 完成 `8C` 状态矩阵；
  - 建立执行前状态门闩；
  - 准备 `8D` 的 home / ready 点，但先做人工 review。
- 第 3 周：
  - 完成 `8D` 低速小范围动作；
  - 每次动作后复盘日志、误差、状态变化；
  - 开始整理异常处理矩阵。
- 第 4 周：
  - 完成 `8E` 和 `8F`；
  - 只做必要的异常验证，不故意制造高风险状态；
  - 收口阶段验收与后续边界。

## 10. learn mode 分工原则
- 智能体负责：
  - 文档模板、checklist、runbook；
  - ROS 2 包骨架、launch wrapper、状态查询脚本；
  - 范围检查、日志结构、执行前状态门闩的脚手架；
  - 集成检查与实验记录整理。
- 人类负责：
  - 真机现场安全确认；
  - IP、示教器、Remote Control、External Control 程序设置；
  - home / ready 点位选择；
  - 是否允许执行某一次真机动作；
  - 急停、保护停止、恢复操作的现场判断。

## 11. 执行约束
- 每次只推进一个任务，完成并记录后再进入下一项。
- 未完成 `8A` 和 `8B` 前，不写任何会发送轨迹的入口。
- 未完成 `8C` 状态门闩前，不允许 `8D` 执行动作。
- 每个动作入口必须默认拒绝执行，只有显式确认与状态检查通过后才发送 goal。
- 任何轨迹执行前必须记录：
  - 当前 joint state；
  - 目标 joint state；
  - 每个关节的 delta；
  - controller 状态；
  - robot mode / safety mode / program state；
  - 操作者确认。
- 任何失败都先进入“停止、记录、人工判断”，不做自动重试。
- 不在阶段 4 首轮做：
  - MoveIt Servo 真机控制；
  - 遥操作；
  - 视觉闭环；
  - 长时间循环运动；
  - 自动解锁 protective stop；
  - 大范围笛卡尔运动。

## 12. 阶段验收标准
- 能从 ROS PC 安全连接 UR3 真机，并解释网络、External Control、Dashboard、controller 的角色。
- 能只读验证 `/joint_states`、controller 状态、Dashboard 状态和 External Control 状态。
- 能在低速模式下执行一个简单 home / ready 点运动。
- 能在目标越界、状态不满足、controller 不可用、program 未运行等异常下拒绝执行或进入停机路径。
- 能为每一次目标、规划结果、执行结果、异常原因生成可追踪日志。
- 能清楚说明当前代码哪些是学习实验入口，哪些条件尚不足以成为真实控制系统。

## 13. 当前开放问题
- 真实机器人型号是 `ur3` 还是 `ur3e`？
- 控制柜 PolyScope / PolyScope X 版本是多少？
- ROS PC 当前是否使用 lowlatency 或 PREEMPT_RT 内核？
- 现场网络拓扑与固定 IP 如何配置？
- home / ready 点应由谁审核，是否需要现场安全负责人确认？
- 是否允许阶段 4 末尾接入 MoveIt 规划执行，还是只保留关节空间 ready 点？
