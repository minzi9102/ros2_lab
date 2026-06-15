# URSim Docker runbook

## 目标

在本机通过 Docker 启动 URSim，并为 `ur_robot_driver`、MoveIt、MoveIt Servo 等 ROS 2 实验提供可连接的 UR3 仿真环境。

本 runbook 汇总仓库历史经验，尤其是容器网络、Docker context、External Control URCap 和控制器状态相关的坑。

## 适用范围

- URSim e-Series Docker 镜像。
- UR3 / UR3e 学习实验。
- 需要 `ur_robot_driver` 通过真实网络接口连接 URSim 的任务。
- 不适用于验证真实动力学、真实碰撞、真实力矩和真实安全距离。

URSim 能验证 RTDE、Dashboard、External Control、controller、speed scaling 和 MoveIt 执行链路；不能替代真机安全验证。

## 快速启动

优先使用系统 Docker Engine 的 `default` context：

```bash
docker context ls
docker --context default image ls | grep -i ursim
```

启动可复用容器：

```bash
docker --context default run -d \
  --name ursim_ur3 \
  -p 5900:5900 \
  -p 6080:6080 \
  -e ROBOT_MODEL=UR3 \
  -v /home/minzi/.ursim/e-series/urcaps:/urcaps:ro \
  -v /home/minzi/.ursim/programs:/ursim/programs \
  universalrobots/ursim_e-series:latest
```

如果容器已经存在：

```bash
docker --context default start ursim_ur3
```

查看容器 IP：

```bash
docker --context default inspect -f '{{range.NetworkSettings.Networks}}{{.IPAddress}}{{end}}' ursim_ur3
```

当前历史记录中常见结果：

```text
URSim container IP: 172.17.0.2
Docker bridge host IP: 172.17.0.1
```

打开浏览器访问：

```text
http://localhost:6080/vnc.html
```

或使用 VNC：

```text
localhost:5900
```

## ROS 侧连接

ROS 侧 `robot_ip` 使用 URSim 容器 IP，而不是宿主机 bridge IP：

```bash
robot_ip:=172.17.0.2
```

网络预检：

```bash
ping -c 3 172.17.0.2
nc -vz 172.17.0.2 29999
nc -vz 172.17.0.2 30001
nc -vz 172.17.0.2 30002
nc -vz 172.17.0.2 30004
```

端口含义：

| 端口 | 用途 |
| --- | --- |
| 29999 | Dashboard |
| 30001 | Primary interface |
| 30002 | Secondary interface |
| 30004 | RTDE |
| 50002 | External Control reverse interface，通常由 ROS driver 在宿主机监听 |

示例：实时键盘 Servo URSim 路径：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_realtime_control
source /opt/ros/jazzy/setup.bash
source ../ws_stage3/install/setup.bash
source install/setup.bash

ros2 launch ur3e_keyboard_servo_py sim_keyboard_servo.launch.py \
  use_mock_hardware:=false \
  robot_ip:=172.17.0.2 \
  launch_rviz:=false
```

## External Control 配置

URSim 中的 External Control Host 必须填写容器能访问到的宿主机地址。

在默认 Docker bridge 下：

```text
External Control Host IP: 172.17.0.1
External Control Port: 50002
```

注意：

- `robot_ip` 是 ROS 连接 URSim 的地址，常见是 `172.17.0.2`。
- External Control Host 是 URSim 回连 ROS driver 的地址，常见是 `172.17.0.1`。
- 这两个地址不是同一个东西。

配置步骤：

1. 在 URSim PolyScope 中进入 Installation / URCaps，确认 External Control URCap 已安装。
2. 在 Program 中添加 External Control 节点。
3. 将 Host 配成 `172.17.0.1`，Port 配成 `50002`。
4. 启动 ROS driver / launch。
5. 在 PolyScope 中运行 External Control 程序。

## 安装 External Control URCap

容器化 URSim 默认不一定已经安装 External Control URCap。

历史稳定做法：

```bash
docker --context default exec ursim_ur3 sh -lc 'ls -lah /urcaps; find /urcaps -maxdepth 2 -type f | sort'
```

确认 `externalcontrol-1.0.5.urcap` 能在容器内看到后，在 PolyScope GUI 中手动安装：

1. 打开 URCaps 页面。
2. 选择挂载目录中的 External Control URCap。
3. 安装后按提示重启 PolyScope。
4. PolyScope 重启可能导致容器退出。
5. 重新启动容器：

```bash
docker --context default start ursim_ur3
```

安装成功后，还必须在 Program 页面插入并运行 External Control。只安装 URCap 不等于执行链路已就绪。

## 关键坑

### 1. Docker context 可能不是同一个 daemon

本机历史上同时出现过：

```text
default
desktop-linux
```

它们可能是两套独立 Docker daemon。镜像、容器、网络互相不可见。

排查：

```bash
docker context ls
docker --context default ps -a
docker --context desktop-linux ps -a
docker --context default image ls | grep -i ursim
docker --context desktop-linux image ls | grep -i ursim
```

建议：本仓库 URSim 实验优先统一使用：

```bash
docker --context default ...
```

### 2. External Control Host 不能沿用旧 IP

不要把历史实验里的 `192.168.56.1`、`192.168.56.101` 直接套到 Docker URSim。

每次先确认容器网络：

```bash
docker --context default inspect ursim_ur3 --format '{{json .NetworkSettings.Networks}}'
```

默认 bridge 下通常是：

```text
URSim container: 172.17.0.2
Host gateway: 172.17.0.1
```

External Control 里填 `172.17.0.1:50002`。

### 3. Plan 成功不代表 Execute 成功

如果 RViz / MoveIt 能 plan，但 execute 不动，优先查执行链路：

```bash
ros2 topic echo --once /speed_scaling_state_broadcaster/speed_scaling
ros2 control list_controllers
```

常见失败信号：

```text
speed_scaling = 0.0
scaled_joint_trajectory_controller inactive
forward_position_controller inactive
```

优先回到 URSim 页面确认 External Control 程序是否真的在运行，而不是继续改 MoveIt waypoint。

### 4. controller 先 active 后 inactive

历史上观察到 `forward_position_controller` 先激活成功，随后很快被切为 inactive。

常见解释：

- External Control 程序没有持续运行。
- URSim 程序状态与 ROS controller 生命周期不同步。
- controller stopper 根据 UR 程序状态停止了运动 controller。

处理：

1. 在 URSim 中停止当前程序。
2. 重新运行 External Control 程序。
3. 重新启动 ROS launch。
4. 观察 `speed_scaling` 是否非零，目标 controller 是否 active。

### 5. URSim 很吃内存

URSim Java 进程可能占用数 GB 内存。与 RViz、MoveIt、浏览器、VS Code 同时运行时，可能出现进程被杀或启动不稳定。

建议：

- 先用 `launch_rviz:=false` 跑通 driver / Servo 链路。
- 再按需要打开 RViz。
- 出现 `servo_node exit code -9` 时，先检查系统内存和 swap。

```bash
free -h
swapon --show
ps -eo pid,ppid,stat,rss,comm,args --sort=-rss | head -25
journalctl -k --since '10 min ago' --no-pager | grep -Ei 'killed process|oom|out of memory'
```

## 成功信号

UR driver 连接成功：

```text
Connected: Universal Robots Dashboard Server
Negotiated RTDE protocol version to 2
Received URControl version ...
```

External Control / 执行链路可用：

```text
speed_scaling > 0
目标 trajectory / forward controller active
/joint_states 正常发布
URSim 中机械臂响应执行或 Servo 输入
```

MoveIt Servo 可用：

```text
Servo initialized successfully
Received ServoStatus on /servo_node/status
MoveIt Servo accepted TWIST command mode
```

## 参考记录

- `notes/labs/task5B_ur_controller_system.md`
- `notes/plans/tasks/task6B_plan.md`
- `notes/labs/task7C_planning_scene_collision.md`
- `workspaces/ws_ur3e_controller/URSIM_GUI_VALIDATION_2026-06-05.md`
- `workspaces/ws_realtime_control/notes/sim_ursim_validation.md`
- `experience/sessions/20260416-122035--task7c-ursim-smoke.json`
