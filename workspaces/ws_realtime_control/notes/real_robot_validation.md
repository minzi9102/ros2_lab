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
linear_speed_mps=0.010
key_timeout_sec=0.25
enable_z=false
enable_rotation=false
max_session_duration_sec=45.0
```

`real_keyboard_servo.launch.py` 允许临时覆盖低速验证参数，但带有硬上限：

```text
linear_speed_mps <= 0.100
key_timeout_sec <= 0.50
max_session_duration_sec <= 90.0
```

启动顺序要求：

```text
1. 启动真实 UR driver，暂不启动 dashboard client。
2. 等待 hardware ready gate：controller manager 可用、joint_state_broadcaster active、/joint_states 完整。
3. hardware ready 后再启动 dashboard client 和 External Control manager。
4. 等待 forward_position_controller active。
5. 启动 MoveIt Servo。
6. 等待 /servo_node/status。
7. 启动 keyboard_servo_node。
```

历史问题：

```text
2026-06-17 首版 real_keyboard_servo.launch.py 曾同时启动 driver 和 External Control manager。
结果 UR driver 在 configure 阶段报错：Could not get configuration package within timeout。
修正：复用 Task 8B 时序，先 hardware ready，再 dashboard / External Control。

2026-06-17 真机 launch 修正后，键盘节点已能进入 TWIST mode 并读取终端按键。
短按 `w` 时日志连续输出 `Key command received: action=move x=1.0 y=0.0`。
随后触发 30 秒会话上限并自动发布 stop 退出，这是安全参数导致的预期行为。
机械臂运动情况不明显，判断与原默认速度 `0.005 m/s` 过低和会话窗口较短有关。
修正：将真机 launch 改为可覆盖 `linear_speed_mps`、`key_timeout_sec`、`max_session_duration_sec`，默认保持低速但提升到更易观察的 `0.010 m/s`。

2026-06-17 将 `linear_speed_mps` 可覆盖硬上限提升到 `0.050 m/s` 后，用户报告实机键盘 Servo 验证成功。
本轮未粘贴完整启动日志和实际使用速度，记录为“调速后人工观察成功”；四方向逐项矩阵仍待单独补齐。

2026-06-17 按用户要求，将当前 `linear_speed_mps` 可覆盖硬上限继续提升到 `0.100 m/s`。
默认值仍保持 `0.010 m/s`，只有显式传参时才使用更高速度。

2026-06-17 用户报告 `linear_speed_mps:=0.100` 实机试验成功，机械臂可以响应键盘 Servo 输入并产生可观察运动。
同时用户反馈 0.100 m/s 下运动有点卡顿；该现象应作为后续平滑性调查输入，不作为继续升速依据。
```

## 待验证矩阵

| 操作 | 预期 | 结果 |
| --- | --- | --- |
| 不按键 | 机械臂静止 | 真机 launch 链路成功，未见异常自发运动报告 |
| w / ↑ 短按 | 末端沿 +x 微动 | 成功，0.100 m/s 下可观察运动，但用户反馈有卡顿 |
| s / ↓ 短按 | 末端沿 -x 微动 | 待验证 |
| a / ← 短按 | 末端沿 +y 微动 | 待验证 |
| d / → 短按 | 末端沿 -y 微动 | 待验证 |
| 松键 | 0.25 秒内停止 | 待验证 |
| 空格 | 立即停止 | 待验证 |
| q | 退出前发布零速度 | 待验证 |
| 45 秒会话上限 | 超时自动停止并退出 | 先前 30 秒上限已验证会自动 stop；45 秒版本待复测 |

## 禁止事项

- 不使用 `sim_keyboard_servo.launch.py` 连接真机。
- 不超过 `linear_speed_mps:=0.100`。
- 不启用 z 轴或旋转。
- 不在安全模式异常、External Control 未运行、speed scaling 为 0 时启动。
