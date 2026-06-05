# ws_ur3e_controller URSim GUI 验收记录

记录日期：2026-06-05

工作区：`/home/minzi/ros2_lab/workspaces/ws_ur3e_controller`

目标：在 `URSim + ur_robot_driver + MoveIt/RViz` 环境中，使用 `ur3e_named_motion_gui_py` GUI 触发 `READY -> HOME` 两个命名目标执行，并确认链路可用。

## 1. 环境与地址

URSim 使用 Docker 容器启动：

```bash
docker run --rm -it \
  --name ursim_ur3 \
  -p 5900:5900 \
  -p 6080:6080 \
  -e ROBOT_MODEL=UR3 \
  -v /home/minzi/.ursim/e-series/urcaps:/urcaps:ro \
  universalrobots/ursim_e-series:latest
```

本次确认到的 URSim 容器 IP：

```text
172.17.0.2
```

Docker bridge 中 ROS 主机地址：

```text
172.17.0.1
```

因此本次使用：

- `robot_ip:=172.17.0.2`
- Polyscope External Control Host IP：`172.17.0.1`
- Polyscope External Control Port：`50002`

## 2. URCap 与 External Control 恢复

初始现象：

- Polyscope 的 `Installation -> System -> URCaps` 中没有可用的 External Control。
- `Active URCaps` 为空。

参考历史记录后确认，这和之前 Task 7C URSim 冒烟时的问题一致：

- URSim 初始镜像没有 External Control。
- 需要挂载并安装 `externalcontrol-1.0.5` URCap。
- Docker bridge 下 External Control 的远程主机地址应使用宿主机网关 `172.17.0.1`，而不是旧桥接网段地址。

恢复方式：

1. 停止旧 `ursim_ur3` 容器。
2. 使用 `-v /home/minzi/.ursim/e-series/urcaps:/urcaps:ro` 重新启动 URSim。
3. 在 Polyscope 中确认 External Control URCap 可用。
4. 在程序中添加 External Control 节点。
5. 将 External Control Host 配置为 `172.17.0.1:50002`。
6. 启动 ROS driver 链路后，在 Polyscope 中运行 External Control 程序。

## 3. ROS 启动方式

启动 URSim 版 named motion bringup：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_ur3e_controller
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ur3e_named_motion_controller_cpp sim_named_motion_bringup.launch.py \
  runtime_mode:=sim \
  execute:=true \
  use_mock_hardware:=false \
  robot_ip:=172.17.0.2 \
  launch_rviz:=true
```

另开终端启动 GUI：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_ur3e_controller
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ur3e_named_motion_gui_py sim_named_motion_gui.launch.py
```

## 4. 中间故障：delta gate 拒绝

GUI 首次点击 `READY` 后，service 已连接，但控制器拒绝执行：

```text
FAILED: accepted=False planned=False executed=False
status=rejected_delta
message=delta gate failed at elbow_joint: delta=3.250200 rad exceeds max_joint_delta_rad=3.200000
```

当时读取到的关键 URSim 关节状态：

```text
elbow_joint = -2.2030 rad
```

`sim.ready` 中 `elbow_joint` 目标约为：

```text
1.0472 rad
```

因此差值为：

```text
3.2502 rad
```

该值超过仿真 catalog 的：

```text
max_joint_delta_rad = 3.2000 rad
```

处理方式：

1. 在 Polyscope 中停止 External Control 程序。
2. 进入 Move 页面。
3. 将 `elbow_joint` 从约 `-126.2 deg` 调整到约 `-120 deg`，使当前姿态落回 delta gate 允许范围。
4. 重新运行 External Control 程序。
5. 在 GUI 中再次点击 `READY`。

## 5. 最终验收结果

用户现场确认：

- GUI service 状态为 connected。
- `READY` 点击后执行通过。
- `HOME` 点击后执行通过。
- 本轮 `READY -> HOME` 在 URSim 环境下完成。

本轮结论：

- `ur3e_named_motion_gui_py` GUI 能连接 URSim 版 named motion service。
- URSim External Control URCap 挂载与配置恢复有效。
- `robot_ip:=172.17.0.2` 与 External Control Host `172.17.0.1:50002` 的组合可用。
- 初始姿态可能触发 delta gate；需要先将 URSim 当前关节姿态调整到 catalog 目标允许范围内。
- 通过 delta gate 后，GUI 可完成 `READY -> HOME` 执行闭环。

## 6. 复现提醒

- URSim 容器启动时必须挂载 External Control URCap 目录。
- Docker default bridge 下，Polyscope External Control Host 通常填 `172.17.0.1`。
- ROS driver 的 `robot_ip` 填 URSim 容器 IP，本次为 `172.17.0.2`。
- 如果 GUI 返回 `rejected_delta`，先检查 `/joint_states` 与 catalog 目标差值，而不是优先怀疑 GUI。
- 如果 ROS 侧已停止，`/speed_scaling_state_broadcaster/speed_scaling` 和 `/controller_manager/list_controllers` 查询会不可用，这是正常现象。
