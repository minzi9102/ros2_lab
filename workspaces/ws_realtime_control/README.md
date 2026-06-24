# ws_realtime_control

`ws_realtime_control` 是 UR3e 实时遥操作实验工作区，用于验证键盘、evdev 键盘和 Xbox 手柄到 MoveIt Servo 的连续速度控制链路。

当前稳定入口是 fake hardware 仿真；URSim 和真机路径已有键盘验证记录，但手柄真机控制尚未开放。

## 环境

```bash
cd ~/ros2_lab/workspaces/ws_realtime_control
source /opt/ros/jazzy/setup.bash
source ../ws_stage3/install/setup.bash
source install/setup.bash
```

构建与测试：

```bash
colcon build
colcon test --packages-select ur3e_keyboard_servo_py --event-handlers console_direct+
colcon test-result --verbose
```

## Fake Hardware：Xbox 手柄

当前已验证设备：

```text
Xbox Series X Controller
/dev/input/js0
```

启动：

```bash
ros2 launch ur3e_keyboard_servo_py sim_keyboard_servo.launch.py \
  use_mock_hardware:=true \
  launch_rviz:=true \
  input_backend:=joy \
  command_frame:=base_link \
  linear_speed_mps:=0.20
```

控制语义：

| 输入 | 行为 |
| --- | --- |
| 左摇杆上 / 下 | 参考坐标系 `+X` / `-X` |
| 左摇杆左 / 右 | 参考坐标系 `+Y` / `-Y` |
| 左摇杆对角线 | x/y 合成，合速度不超过 `linear_speed_mps` |
| 松开左摇杆 | 平滑停止 |
| A | 立即停止 |
| B | 停止并退出 |

手柄输入由 ROS 2 标准 `joy_node` 发布到 `/joy`，本包只负责将 `/joy` 转换为 `/servo_node/delta_twist_cmds`。

## 其他输入后端

终端键盘：

```bash
ros2 launch ur3e_keyboard_servo_py sim_keyboard_servo.launch.py \
  use_mock_hardware:=true \
  launch_rviz:=true \
  input_backend:=terminal
```

evdev 键盘：

```bash
ros2 launch ur3e_keyboard_servo_py sim_keyboard_servo.launch.py \
  use_mock_hardware:=true \
  launch_rviz:=true \
  input_backend:=evdev \
  input_device:=/dev/input/by-id/usb-ITE_Tech._Inc._ITE_Device_8910_-event-kbd \
  command_frame:=base_link \
  linear_speed_mps:=0.20
```

`command_frame` 支持：

```text
base_link
tool0
```

## 安全边界

- fake hardware 手柄控制已完成 RViz 人工验收。
- 手柄真机控制未实现、未验收，不能直接用于真实机器人。
- 真机实时控制必须另走安全 launch、人工确认、低速限幅和只读状态门闩。
- 出现 MoveIt Servo singularity、collision 或 emergency stop 日志时，应立即松开输入或按停止键，并记录现象。

## 记录

- 总计划：[REALTIME_CONTROL_PLAN.md](REALTIME_CONTROL_PLAN.md)
- fake hardware 验证：[notes/sim_fake_hardware_validation.md](notes/sim_fake_hardware_validation.md)
- URSim 验证：[notes/sim_ursim_validation.md](notes/sim_ursim_validation.md)
- 真机验证：[notes/real_robot_validation.md](notes/real_robot_validation.md)
