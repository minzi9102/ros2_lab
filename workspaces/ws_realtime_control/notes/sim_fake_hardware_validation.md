# Fake Hardware Smoke Test 记录

日期：2026-06-15

## 目标

验证 `ur3e_keyboard_servo_py` 的 fake hardware 仿真启动链路能跑到 MoveIt Servo 和键盘控制节点。

本轮先做启动链路 smoke test，随后补做 RViz 中的键盘输入与机械臂运动人工验收。

## 命令

预检：

```bash
cd ~/ros2_lab/workspaces/ws_realtime_control
source /opt/ros/jazzy/setup.bash
source ../ws_stage3/install/setup.bash
source install/setup.bash

ros2 pkg prefix ur3e_keyboard_servo_py
ros2 pkg prefix ur3_moveit_servo_lab_cpp
ros2 pkg prefix ur_robot_driver
ros2 pkg prefix ur_moveit_config
```

调试运行：

```bash
ros2 launch ur3e_keyboard_servo_py sim_keyboard_servo.launch.py \
  use_mock_hardware:=true \
  launch_rviz:=false
```

最终一次日志：

```text
workspaces/ws_realtime_control/logs/sim_keyboard_servo_console_20260615-171417.log
workspaces/ws_realtime_control/logs/sim_keyboard_servo/20260615-171418/
```

## 结果

通过的启动节点与门闩：

- `ros2_control_node` 使用 fake hardware 启动。
- `/joint_states` 出现。
- `joint_state_broadcaster` 与 `forward_position_controller` active。
- `move_group` 完成初始化并输出 `You can start planning now!`。
- `servo_node` 完成初始化并发布 `/servo_node/status`。
- `keyboard_servo_node` 启动。
- `keyboard_servo_node` 请求并完成 MoveIt Servo TWIST command mode 切换。
- `launch_rviz:=true` 能启动 RViz，并进入 MoveIt MotionPlanning 面板。
- 启动 launch 的终端获得焦点后，`keyboard_servo_node` 能收到键盘输入并生成 move command。
- 用户确认 RViz 中可以看到机械臂响应键盘输入并运动。

关键日志：

```text
Keyboard Servo node started. command_topic=/servo_node/delta_twist_cmds command_type_service=/servo_node/switch_command_type frame_id=base_link rate=30.0Hz speed=0.0200m/s timeout=0.20s
Requested MoveIt Servo TWIST command mode.
MoveIt Servo accepted TWIST command mode.
```

人工键盘输入验收记录：

```text
[keyboard_servo_node-13] [INFO] [1781531634.643721568] [ur3e_keyboard_servo]: Key command received: action=move x=-1.0 y=0.0
[keyboard_servo_node-13] [INFO] [1781531634.676777907] [ur3e_keyboard_servo]: Key command received: action=move x=-1.0 y=0.0
[keyboard_servo_node-13] [INFO] [1781531634.743417258] [ur3e_keyboard_servo]: Key command received: action=move x=-1.0 y=0.0
[keyboard_servo_node-13] [INFO] [1781531634.777014191] [ur3e_keyboard_servo]: Key command received: action=move x=-1.0 y=0.0
[keyboard_servo_node-13] [INFO] [1781531634.810306771] [ur3e_keyboard_servo]: Key command received: action=move x=-1.0 y=0.0
```

结论：

- fake hardware + RViz 启动链路通过。
- MoveIt Servo TWIST mode 握手通过。
- 键盘输入到 `keyboard_servo_node` 的读取链路通过。
- `keyboard_servo_node` 能按键盘输入生成 x/y 平面速度命令。
- fake hardware 下键盘输入到 RViz 机械臂运动的闭环通过。

## 本轮发现并修复的问题

1. `keyboard_servo_node` 启动时崩溃。
   - 原因：`rclpy` logger 不支持 Python logging 风格的多参数格式化。
   - 修复：改为预格式化字符串。

2. 非交互终端运行 smoke test 时，终端按键读取可能无法进入 cbreak 模式。
   - 修复：`TerminalKeyReader` 在非 TTY 环境下不启用 termios，并返回无按键输入。

3. MoveIt Servo 未设置 command type 时拒绝 Twist 输入。
   - 修复：`keyboard_servo_node` 启动后调用 `/servo_node/switch_command_type`，请求 TWIST mode，成功后再发布 Twist。

4. 启动期日志不够完整。
   - 修复：仿真 launch 将本包节点输出改为 `both`，并设置 ROS 日志相关环境变量。

5. `launch_rviz:=true` 时没有看到 RViz 窗口。
   - 原因：顶层 `launch_rviz` 与内层 `ur_control.launch.py` / `ur_moveit.launch.py` 使用同名 launch configuration；内层 include 传入的 `launch_rviz=false` 污染了顶层 RViz 条件判断。
   - 历史参考：Task 7E 的 `b0d106e fix(task7E): restore rviz launch in nested moveit flow` 也修过同类问题。
   - 修复：在顶层 launch 中将用户输入缓存到 `realtime_launch_rviz`，RViz 节点条件读取该私有配置。
   - 验证：`logs/sim_keyboard_servo_console_20260615-172046_rviz.log` 中出现 `rviz2-7: process started`，且 RViz 日志出现 `Ready to take commands for planning group ur_manipulator.`。

6. RViz 能启动，但按 `w/a/s/d` 和方向键没有运动。
   - 原因：`ros2 launch` 启动的子进程 `stdin` 不一定是交互 TTY，原 `TerminalKeyReader` 会进入非交互模式并持续返回无按键。
   - 修复：当 `stdin` 不是 TTY 时，`TerminalKeyReader` 尝试打开控制终端 `/dev/tty` 读取按键；同时启动日志会显示键盘输入来源。
   - 操作要求：按键时焦点必须在启动 launch 的终端窗口；如果焦点在 RViz，按键会被 RViz 接收而不是键盘节点接收。
   - 验证方式：终端或日志中应出现 `Keyboard input attached to ...`，按有效键时应出现 `Key command received: action=move ...`。

## 剩余验收

RViz 已确认能启动，键盘输入已确认能进入节点，机械臂已确认会响应按键运动。后续可继续补充完整方向矩阵记录：

```bash
ros2 launch ur3e_keyboard_servo_py sim_keyboard_servo.launch.py \
  use_mock_hardware:=true \
  launch_rviz:=true
```

待确认：

- ↑：末端沿 +x 小幅移动。
- ↓：末端沿 -x 小幅移动。
- ←：末端沿 +y 小幅移动。
- →：末端沿 -y 小幅移动。
- 松键自动停止。
- 空格立即停止。
- q 退出并停止。

## 高速流畅 evdev 切片

日期：2026-06-23

已实现但尚未完成人工 RViz 验收：

```text
evdev 真实 key-down / key-up
自动重复事件忽略
100 Hz TwistStamped 发布
0.20 m/s 目标速度
0.50 m/s² 加速度
0.80 m/s² 减速度
base_link / tool0 参考坐标系选择
多方向冲突归零
方向反转先减速到零
空格和 q 立即归零
```

系统准备：

```bash
sudo apt install python3-evdev
sudo usermod -aG input minzi
```

注销并重新登录后检查：

```bash
id
python3 -c "import evdev"
test -r /dev/input/by-id/usb-ITE_Tech._Inc._ITE_Device_8910_-event-kbd
```

计划中的 fake hardware 验收命令：

```bash
ros2 launch ur3e_keyboard_servo_py sim_keyboard_servo.launch.py \
  use_mock_hardware:=true \
  launch_rviz:=true \
  input_backend:=evdev \
  input_device:=/dev/input/by-id/usb-ITE_Tech._Inc._ITE_Device_8910_-event-kbd \
  command_frame:=base_link \
  linear_speed_mps:=0.20
```

当前状态：

```text
pytest 与 colcon test 均为 28 passed，colcon build 通过。
terminal fake hardware 回归 smoke test 已到达 MoveIt Servo TWIST mode 握手。
非法坐标系、缺失 evdev 设备、非正速度均在 driver 启动前拒绝。
当前系统尚未安装 python3-evdev，minzi 尚未加入 input 组；ITE 设备存在但不可读。
人工 RViz 验收未执行，不记录为通过。
加入 input 组后，用户可以读取系统输入设备事件。
```
