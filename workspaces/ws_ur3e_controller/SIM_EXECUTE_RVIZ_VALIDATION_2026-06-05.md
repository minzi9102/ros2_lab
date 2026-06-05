# ws_ur3e_controller 仿真 execute + RViz 验收记录

记录日期：2026-06-05

工作区：`/home/minzi/ros2_lab/workspaces/ws_ur3e_controller`

目标：在完整 MoveIt fake hardware 仿真中启动 RViz，观察机械臂执行 `ready` 和 `home` 两个命名目标，并确认 execute 路径通过 final-target gate。

## 1. 启动方式

启动完整仿真、MoveIt、RViz，并允许执行：

```bash
ros2 launch ur3e_named_motion_controller_cpp sim_named_motion_bringup.launch.py \
  execute:=true \
  launch_rviz:=true
```

启动日志确认：

- `ur3e_named_motion_controller` 以 `runtime_mode=sim allow_execution=true` 启动。
- `move_group` 输出 `You can start planning now!`。
- RViz 输出 `Ready to take commands for planning group ur_manipulator`。

## 2. ready execute 验收

调用：

```bash
ros2 service call /ur3e_named_motion_controller/execute_named_target \
  ur3e_controller_msgs/srv/ExecuteNamedTarget \
  "{target_name: ready, execute: true, human_confirmation: ''}"
```

结果：

```text
accepted=True
planned=True
executed=True
status='executed'
message="target 'ready' executed and final-target gate passed"
```

结论：`ready` 在仿真中执行成功，并通过 final-target gate。

## 3. home execute 验收

调用：

```bash
ros2 service call /ur3e_named_motion_controller/execute_named_target \
  ur3e_controller_msgs/srv/ExecuteNamedTarget \
  "{target_name: home, execute: true, human_confirmation: ''}"
```

结果：

```text
accepted=True
planned=True
executed=True
status='executed'
message="target 'home' executed and final-target gate passed"
```

结论：`home` 在仿真中执行成功，并通过 final-target gate。

## 4. home 最终状态检查

执行 `home` 后读取 `/joint_states --once`，关键关节位置如下：

```text
elbow_joint: 1.57072912306902
shoulder_pan_joint: -0.00006791850998997688
```

与 `home` 目标值对比：

- `elbow_joint` 目标 `1.5708 rad`，实际约 `1.570729 rad`
- `shoulder_pan_joint` 目标 `0.0 rad`，实际约 `-0.000068 rad`

结论：关键关节实际位置与 `home` 目标值一致，误差明显小于仿真 final-target gate 容差 `0.03 rad`。

## 5. RViz 观察结论

本次执行顺序为 `ready -> home`：

- 执行 `ready` 时，RViz 中机械臂从 home 姿态运动到 ready 姿态。
- 执行 `home` 时，RViz 中机械臂从 ready 姿态返回 home 姿态。
- 两次请求都没有使用人工确认 token。

## 6. 总结

本次验收结果为 PASS：

- `ready` execute 通过。
- `home` execute 通过。
- 两者均通过 final-target gate。
- 完整仿真链路支持通过 RViz 观察实际运动过程。
