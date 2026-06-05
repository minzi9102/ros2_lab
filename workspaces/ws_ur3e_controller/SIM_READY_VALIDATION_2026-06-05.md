# ws_ur3e_controller sim.ready 仿真验收记录

记录日期：2026-06-05

工作区：`/home/minzi/ros2_lab/workspaces/ws_ur3e_controller`

目标：验证调整后的 `sim.ready` 命名目标在完整 MoveIt 仿真中能够通过 plan-only 和 execute，并在执行后通过 final-target gate。

## 1. 目标定义

`sim.ready` 相对 `sim.home` 的设计位移：

- `shoulder_pan_joint`：`+30deg`，目标值约 `0.5236 rad`
- `elbow_joint`：`-30deg`，目标值约 `1.0472 rad`
- 其余关节保持 `home` 姿态

## 2. Plan-Only 验收

调用：

```bash
ros2 service call /ur3e_named_motion_controller/execute_named_target \
  ur3e_controller_msgs/srv/ExecuteNamedTarget \
  "{target_name: ready, execute: false, human_confirmation: ''}"
```

结果：

```text
accepted=True
planned=True
executed=False
status='planned'
message='plan-only request passed catalog, joint-state, delta, and MoveGroup planning gates'
```

结论：`ready` 通过 catalog、joint-state、delta 和 MoveGroup planning gate。

## 3. Execute 验收

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

结论：`ready` 在仿真中完成执行，并通过 final-target gate。

## 4. Joint State 到位检查

执行后读取 `/joint_states --once`，关键关节位置如下：

```text
elbow_joint: 1.0472238897749688
shoulder_pan_joint: 0.5235859430485871
```

与目标值对比：

- `shoulder_pan_joint` 目标 `0.5236 rad`，实际约 `0.523586 rad`
- `elbow_joint` 目标 `1.0472 rad`，实际约 `1.047224 rad`

结论：关键关节实际位置与 `sim.ready` 目标值一致，误差明显小于仿真 final-target gate 容差 `0.03 rad`。

## 5. 总结

本次验收结果为 PASS：

- `ready` plan-only 通过。
- `ready` execute 通过。
- final-target gate 通过。
- `/joint_states` 中第一关节和第三关节到位值符合设计。

下一步进入 launch + service 级自动测试设计与实现。
