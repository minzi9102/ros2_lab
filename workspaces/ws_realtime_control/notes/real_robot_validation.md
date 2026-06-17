# Real Robot Keyboard Servo 验证记录

日期：2026-06-17

## 当前阶段

已完成真机前置状态检查：

```text
robot mode RUNNING
safety mode NORMAL
External Control running
remote_control=True
joint_state_broadcaster active
/joint_states 约 501 Hz
speed scaling 100.0
trajectory controllers inactive
```

`trajectory controllers inactive` 是只读 bringup 阶段的预期状态。进入键盘 Servo 真机验证前，需要使用 `real_keyboard_servo.launch.py` 由 launch 显式激活 `forward_position_controller`。

## 安全入口

真机键盘 Servo 入口：

```bash
ros2 launch ur3e_keyboard_servo_py real_keyboard_servo.launch.py \
  ur_type:=ur3e \
  robot_ip:=<ROBOT_IP> \
  reverse_ip:=<PC_ROBOT_NET_IP> \
  launch_rviz:=false \
  human_confirmation:=I_CONFIRM_REAL_ROBOT_MOTION
```

真机默认安全参数：

```text
linear_speed_mps=0.005
key_timeout_sec=0.15
enable_z=false
enable_rotation=false
max_session_duration_sec=30.0
```

## 待验证矩阵

| 操作 | 预期 | 结果 |
| --- | --- | --- |
| 不按键 | 机械臂静止 | 待验证 |
| w / ↑ 短按 | 末端沿 +x 微动 | 待验证 |
| s / ↓ 短按 | 末端沿 -x 微动 | 待验证 |
| a / ← 短按 | 末端沿 +y 微动 | 待验证 |
| d / → 短按 | 末端沿 -y 微动 | 待验证 |
| 松键 | 0.15 秒内停止 | 待验证 |
| 空格 | 立即停止 | 待验证 |
| q | 退出前发布零速度 | 待验证 |
| 30 秒会话上限 | 超时自动停止并退出 | 待验证 |

## 禁止事项

- 不使用 `sim_keyboard_servo.launch.py` 连接真机。
- 不提高 `linear_speed_mps`。
- 不启用 z 轴或旋转。
- 不在安全模式异常、External Control 未运行、speed scaling 为 0 时启动。
