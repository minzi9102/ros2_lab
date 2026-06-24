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
垂直方向组合为归一化对角线，总速度保持 0.20 m/s
同轴相反按键按轴抵消
单轴与对角线之间平滑转向
180 度方向反转先减速到零
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
terminal fake hardware 回归 smoke test 已到达 MoveIt Servo TWIST mode 握手。
非法坐标系、缺失 evdev 设备、非正速度均在 driver 启动前拒绝。
python3-evdev 已安装，minzi 已加入 input 组，ITE 键盘设备可读。
0.20 m/s 单轴连续运动已由用户在 RViz 中确认成功。
持续运动最终接近奇异位形，MoveIt Servo 正常触发 emergency stop。
pytest 与 colcon test 均为 37 passed，colcon build 通过。
对角线采样结果为 x=y=0.141421356 m/s，合速度为 0.200000000 m/s。
对角线 RViz 人工验收未执行，不记录为通过。
加入 input 组后，用户可以读取系统输入设备事件。
```

对角线人工验收矩阵：

```text
W+A -> +X/+Y，每轴约 +0.1414 m/s
W+D -> +X/-Y
S+A -> -X/+Y
S+D -> -X/-Y
W+S+A -> X 抵消，只沿 +Y
保持 W 后按 A -> 从 +X 平滑转向 +X/+Y
W+A 运动时松开 A -> 平滑回到 +X
空格 -> 立即停止
```

## Xbox 手柄 fake hardware 切片

日期：2026-06-24

本轮新增，并已完成人工 RViz 验收：

```text
input_backend:=joy
标准 ROS 2 joy_node 读取 USB Xbox 手柄
/joy -> x/y 平面 TwistStamped
左摇杆上/下 -> +X/-X
左摇杆左/右 -> +Y/-Y
对角线输入按向量归一化，合速度不超过 linear_speed_mps
A 按钮 -> 立即停止
B 按钮 -> 立即停止并退出
真机手柄控制不在本切片范围内
```

系统识别结果：

```text
ros2 run joy joy_enumerate_devices
ID 0: Xbox Series X Controller

/dev/input/js0 exists=True readable=True
/dev/input/by-id/usb-Microsoft_Controller_30394F4730303638343936333038-joystick exists=True readable=True
```

计划中的 fake hardware 验收命令：

```bash
ros2 launch ur3e_keyboard_servo_py sim_keyboard_servo.launch.py \
  use_mock_hardware:=true \
  launch_rviz:=true \
  input_backend:=joy \
  command_frame:=base_link \
  linear_speed_mps:=0.20
```

人工验收结果：

```text
用户已使用 Xbox 手柄在 fake hardware + RViz 环境下完成控制闭环验收。
MoveIt Servo 启动、joy_node 手柄输入、/joy 到 TwistStamped 转换、机械臂响应均确认成功。
本结论仅覆盖 fake hardware，不覆盖 URSim 或真机。
```

人工验收矩阵：

```text
左摇杆轻推 -> 低速连续移动
左摇杆推到底 -> 明显连续移动
左摇杆对角线 -> 合速度不暴涨
松开左摇杆 -> 平滑停止
A -> 立即停止
B -> 停止并退出
出现 singularity warning -> 立即松杆或按 A，记录为 MoveIt Servo 安全保护
```
