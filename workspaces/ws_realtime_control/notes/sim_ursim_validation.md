# URSim Smoke Test 记录

日期：2026-06-15

## 目标

验证 `ur3e_keyboard_servo_py` 在 URSim 路径下能通过 MoveIt Servo 驱动 URSim 中的 UR3 机械臂运动。

本轮不打开 RViz，使用 URSim 网页监视器观察机械臂运动。

## 预检

环境：

```bash
cd ~/ros2_lab/workspaces/ws_realtime_control
source /opt/ros/jazzy/setup.bash
source ../ws_stage3/install/setup.bash
source install/setup.bash
```

包解析结果：

```text
ur3e_keyboard_servo_py -> /home/minzi/ros2_lab/workspaces/ws_realtime_control/install/ur3e_keyboard_servo_py
ur3_moveit_servo_lab_cpp -> /home/minzi/ros2_lab/workspaces/ws_stage3/install/ur3_moveit_servo_lab_cpp
ur_robot_driver -> /opt/ros/jazzy
ur_moveit_config -> /opt/ros/jazzy
```

URSim 网络预检：

```bash
ping -c 3 172.17.0.2
nc -vz 172.17.0.2 29999
nc -vz 172.17.0.2 30001
nc -vz 172.17.0.2 30002
nc -vz 172.17.0.2 30004
```

结果：

```text
ping 3/3 received, 0% packet loss
29999 succeeded
30001 succeeded
30002 succeeded
30004 succeeded
```

## 运行命令

```bash
ros2 launch ur3e_keyboard_servo_py sim_keyboard_servo.launch.py \
  use_mock_hardware:=false \
  robot_ip:=172.17.0.2 \
  launch_rviz:=false
```

## 结果

通过项：

- URSim 网络和 Dashboard / primary / secondary / RTDE 端口可达。
- UR driver 能连接 `172.17.0.2`。
- 本轮未启动 RViz。
- 用户通过 URSim 网页监视器确认机械臂响应键盘 Servo 输入并运动。

结论：

```text
URSim 路径下，键盘输入 -> MoveIt Servo -> UR driver -> URSim 机械臂运动链路已完成 smoke test。
```

## 已知观察

前序失败排查中出现过两类现象：

1. `servo_node` 被 `SIGKILL`，随后 `realtime_servo_status_gate` 等待 `/servo_node/status` 超时。
   - 当时系统内存压力较高，URSim Java 进程占用约 5.4GB，swap 已使用约 1.5GB。
   - 未在内核日志中找到明确 OOM killer 记录。

2. 轻量复跑时 `forward_position_controller` 曾先激活成功，随后被切为 inactive，导致 joint state gate 超时。
   - `robot_state_helper` 显示 URSim 当时为 `safety mode NORMAL`、`robot mode RUNNING`。
   - 推测与 URSim External Control 程序运行状态或控制器切换时序有关。

这些现象没有阻止本轮 URSim 网页监视器下的运动 smoke test 通过，但后续做完整 URSim 验收前仍应关注。

## 剩余验收

后续建议补充完整方向矩阵：

| 操作 | 待确认标准 |
| --- | --- |
| ↑ / w | 末端沿 +x 小幅移动 |
| ↓ / s | 末端沿 -x 小幅移动 |
| ← / a | 末端沿 +y 小幅移动 |
| → / d | 末端沿 -y 小幅移动 |
| 松键 | 自动停止 |
| 空格 | 立即停止 |
| q | 退出并停止 |

后续如果需要 RViz 同时观察，建议先关闭不必要的大内存程序，再使用 `launch_rviz:=true`。
