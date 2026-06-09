# ws_ur3e_controller 真机 GUI 验收记录

记录日期：`2026-06-09`

工作区：`/home/minzi/ros2_lab/workspaces/ws_ur3e_controller`

目标：在阶段 4 真机 bringup、状态门闩和 MoveIt 语义层均已就绪的前提下，使用 `ur3e_named_motion_gui_py` 的真机 launch 入口完成 `HOME` / `READY` 两个命名目标的单次 GUI 验收，并记录本轮现场修正与最终结果。

## 1. 背景

本轮真机 GUI 验收并不是首次尝试。

在此前的真机调试中，`real.home` 曾以较早的一版现场姿态为基线，后续又把 `real.ready` 调整为和仿真一致的较大幅度差值。这样做以后，真机当前姿态已经不再靠近旧的 `real.home` / `real.ready`，导致 plan-only 阶段就被 joint delta gate 拒绝。

本轮验收前，现场重新读取了真实 `/joint_states`，并把真机 catalog 基线重录为当前人工确认姿态。对应提交为：

```text
eefdafa feat(ur3e-controller): 按当前真机姿态重录 home ready 基线
```

重录后的策略是：

- `real.home` 直接采用 2026-06-09 现场读到的当前真机姿态。
- `real.ready` 保持和仿真一致的差值模式：
  - `shoulder_pan_joint` `+0.5236 rad`
  - `elbow_joint` `-0.5236 rad`
  - 其余关节不变
- `real.max_joint_delta_rad` 保持 `0.60 rad`，允许从新基线进入 `ready`。

## 2. 本轮前置修正

验收前的关键问题是：旧的真机目标基线已经和现场真实姿态脱节。

现场读取到的当前姿态为：

```text
shoulder_pan_joint   =  0.5235722064971924
shoulder_lift_joint  = -1.5707042559409565
elbow_joint          =  1.0472167173968714
wrist_1_joint        = -1.5708304844298304
wrist_2_joint        = -1.5708978811847132
wrist_3_joint        = -0.00009566942323857575
```

据此更新后的真机目标为：

```text
real.home:
[0.5235722064971924, -1.5707042559409565, 1.0472167173968714,
 -1.5708304844298304, -1.5708978811847132, -0.00009566942323857575]

real.ready:
[1.0471722064971923, -1.5707042559409565, 0.5236167173968715,
 -1.5708304844298304, -1.5708978811847132, -0.00009566942323857575]
```

更新后已重新构建并在隔离 `ROS_DOMAIN_ID` 下通过 `ur3e_named_motion_controller_cpp` 现有测试，避免和现场真机会话串台。

## 3. 验收路径

本轮现场沿用 [REAL_GUI_VALIDATION_TEMPLATE.md](REAL_GUI_VALIDATION_TEMPLATE.md) 的顺序执行：

1. 启动 `task8B_readonly_bringup.launch.py`，确认真机 driver、Dashboard client、External Control 和 `/joint_states` 正常。
2. 激活 `scaled_joint_trajectory_controller`，运行 8C 动作前门闩，确认进入 PASS。
3. 启动 `ur_moveit_config`，确保 `MoveGroupInterface` 所需语义层在线。
4. 先以 `runtime_mode:=real execute:=false` 启动 named motion controller，分别对 `home` 和 `ready` 做 plan-only 验证。
5. 关闭 plan-only 节点，改为 `runtime_mode:=real execute:=true` 重启 named motion controller。
6. 启动：

```bash
ros2 launch ur3e_named_motion_gui_py real_named_motion_gui.launch.py
```

7. 在 GUI 中逐次执行单次 `HOME`、单次 `READY`，每次执行后都重新复核 8C 状态。

其中 `real_named_motion_gui.launch.py` 默认携带：

```text
human_confirmation=I_CONFIRM_REAL_ROBOT_MOTION
```

因此 GUI 本身不负责绕过真机门闩，而是通过显式 token 进入后端 controller 的真实执行路径。

## 4. 结果

本轮真机 GUI 验收最终通过。现场确认结果如下：

- plan-only 阶段已不再触发 `rejected_delta`，说明新 `real.home` / `real.ready` 基线与现场姿态一致。
- GUI 能稳定连接 `/ur3e_named_motion_controller/execute_named_target` service。
- `HOME` 单次 GUI 执行通过。
- `READY` 单次 GUI 执行通过。
- 执行后系统仍可继续通过 8C 复核，说明 Dashboard / controller / External Control / `/joint_states` 链路在本轮执行后保持可解释状态。

本轮最重要的结论是：真机 GUI 并没有暴露新的控制链路问题，真正挡住执行的是 catalog 基线和现场姿态脱节。把基线重录到现场确认姿态后，GUI、controller、MoveIt 和真机 driver 之间的最小闭环可以成立。

## 5. 本轮经验

- 真机 `home` / `ready` 目标不是“写过一次就永远稳定”的常量，任何一次现场复核后都要重新检查它们是否仍和真实起始姿态邻近。
- 如果 `plan-only` 返回 `rejected_delta`，优先比较 `/joint_states` 和 catalog 目标差值，而不是先怀疑 GUI 或 MoveIt。
- GUI 只是薄客户端，真机安全边界仍由：
  - 8B bringup
  - 8C 状态门闩
  - named controller 的 real gate / delta gate / final-target gate
  共同承担。
- `real_named_motion_gui.launch.py` 适合作为单次、低速、有人旁站的真机验收入口；它不是生产控制面板，也不承担自动恢复或异常清除职责。

## 6. 当前边界

本轮通过后，可以确认的能力是：

- `ws_ur3e_controller` 已完成 fake hardware GUI、URSim GUI 和真机 GUI 的首轮最小闭环验收。
- 真机 GUI 可在 `HOME` / `READY` 两个经过现场复核的目标上执行单次动作。

本轮仍未扩大到的范围包括：

- 保护停止 / safeguard stop 的真实触发与恢复
- 更复杂的连续多目标任务
- 笛卡尔目标、任意命名目标编辑、自动重试
- 无人值守执行
- 生产级安全状态机与权限模型

## 7. 关联文档

- [DEBUG_RECORD_2026-05-06_REAL_HOME_EXECUTION.md](DEBUG_RECORD_2026-05-06_REAL_HOME_EXECUTION.md)
- [REAL_GUI_VALIDATION_TEMPLATE.md](REAL_GUI_VALIDATION_TEMPLATE.md)
- [URSIM_GUI_VALIDATION_2026-06-05.md](URSIM_GUI_VALIDATION_2026-06-05.md)
- [REAL_HOME_READY_GUI_PLAN.md](REAL_HOME_READY_GUI_PLAN.md)
