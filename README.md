# ros2_lab

ROS 2 Jazzy 机械臂开发学习仓库，以 UR3 为目标平台，按阶段推进 ROS 2 基础通信、`ros2_control`、UR driver、URSim、MoveIt 2 与 MoveIt Servo。

本仓库默认采用 learn mode：智能体负责计划、脚手架、上下文整理、集成与校验；关键设计和有学习价值的实现优先交给人类完成。协作细则见 [AGENTS.md](AGENTS.md)。

## 当前状态

| 阶段 | 内容 | 状态 |
|---|---|---|
| 阶段 1 | ROS 2 基础能力与控制链路入门（Task 4A-4H） | 已完成 |
| 阶段 2 | UR3 fake hardware / URSim 控制实践（Task 5A-6B） | 已完成 |
| 阶段 3 | UR3 + MoveIt 2 运动规划与 Servo（Task 7A-7E） | 已完成 |
| 阶段 4 | UR3e 真机接入、安全预检与只读状态门闩（Task 8A-8C） | 进行中 |

阶段 3 已于 2026-04-27 收口。阶段 4 当前已完成 8A 网络与现场预检、8B 真机只读 bringup、8C Dashboard/controller 状态门闩；尚未进入真实运动执行。

## 仓库地图

```text
ros2_lab/
├── AGENTS.md                 # learn mode 协作规则
├── CLAUDE.md                 # Claude/Codex 协作补充说明
├── README.md
├── archive/                  # 归档内容
├── docs/                     # 外部资料或补充文档入口
│   └── calibration/          # 真机标定参数归档副本
├── downloads/                # 临时下载目录
├── experience/               # 可复用仓库经验索引与 session 记录
├── logs/                     # 运行验收日志，例如 Task 7E harness
├── notes/
│   ├── concepts/             # 概念笔记
│   ├── labs/                 # 每个任务的实验记录
│   ├── plans/                # 阶段计划与任务拆解
│   ├── reports/              # 阶段复盘
│   └── runbooks/             # 排障手册
├── scripts/                  # 仓库级辅助脚本
└── workspaces/
    ├── ws_tutorials/         # 阶段 1：ROS 2 基础与最小控制链路
    ├── ws_stage2/            # 阶段 2：UR driver、fake hardware、URSim
    ├── ws_stage3/            # 阶段 3：MoveIt 2 与 MoveIt Servo
    └── ws_stage4/            # 阶段 4：UR3e 真机接入与状态门闩
```

`build/`、`install/`、`log/`、`logs/` 多为 colcon 构建或实验运行产物，可能随本机实验状态变化。阅读源码时优先看各工作区的 `src/` 与 `notes/`。

## 工作区与包

### `workspaces/ws_tutorials`

阶段 1 主工作区，覆盖 ROS 2 package、topic、service、action、tf2、QoS、URDF/xacro 与 `ros2_control` 最小链路。

主要包：
- `ur3_joint_state_publisher_py`
- `ur3_tf_lookup_py`
- `ur3_mode_service_py`
- `ur3_qos_lab_py`
- `ur3_ros2_control_lab_py`
- `ur3_forward_command_controller_lab_py`
- `ur3_joint_trajectory_controller_lab_py`
- `ur3_follow_joint_trajectory_client_py`
- `ur3_stage1_review_py`
- `ur3_stage1_review_cpp`
- `ur3_state_monitor_cpp`

### `workspaces/ws_stage2`

阶段 2 工作区，聚焦 UR3 控制链路：fake hardware、`FollowJointTrajectory`、C++ action client、URSim speed scaling。

主要包：
- `ur3_minimal_control_lab_py`
- `ur3_minimal_control_lab_cpp`
- `ur3_ursim_speed_scaling_lab_py`

### `workspaces/ws_stage3`

阶段 3 工作区，复用官方 `ur_moveit_config`，按单功能包拆解 MoveIt 2 学习目标。

主要包：
- `ur3_moveit_bringup_lab`：Task 7A，MoveIt bringup 与 RViz Quickstart
- `ur3_moveit_move_group_lab_cpp`：Task 7B，MoveGroupInterface C++ 最小规划节点
- `ur3_moveit_scene_lab_cpp`：Task 7C，Planning Scene、碰撞体与 Cartesian Path
- `ur3_moveit_goal_pose_lab_cpp`：Task 7D，RViz 点击目标位姿后自动规划执行
- `ur3_moveit_servo_lab_cpp`：Task 7E，MoveIt Servo 连续速度控制最小闭环

### `workspaces/ws_stage4`

阶段 4 工作区，面向真实 UR3e 接入。当前只允许安全预检、只读 bringup、Dashboard/controller 状态门闩与文档化记录；真实运动任务尚未放行。

主要包：
- `ur3_real_bringup_lab`：Task 8A-8C，真机网络/安全预检、只读 driver bringup、状态门闩检查。

真机 calibration 文件：
- 主用文件：`workspaces/ws_stage4/src/ur3_real_bringup_lab/config/ur3e_real_calibration.yaml`
- 归档副本：`docs/calibration/ur3e_real_calibration.yaml`
- 当前 hash：`calib_9781467669625414396`

该文件来自当前真实 UR3e 控制柜，用于替代 `ur_description` 的默认 kinematics 参数，避免 `calibration mismatch` 导致的 TCP、TF、MoveIt 位姿误差。包内 `config/` 文件是后续 bringup/description wrapper 应使用的运行入口；`docs/calibration/` 文件只作为仓库级归档副本。

后续接入 driver 时，需要让 robot description 使用该文件作为 `kinematics_params_file`。例如在 description launch 或自定义 wrapper 中传入：

```bash
kinematics_params_file:=/home/minzi/ros2_lab/workspaces/ws_stage4/src/ur3_real_bringup_lab/config/ur3e_real_calibration.yaml
```

注意：当前官方 `ur_control.launch.py` 在本机版本中不直接透传 `kinematics_params_file`；进入 8D 前应通过自定义 description launch/wrapper 接入该文件，而不是继续依赖默认 kinematics。

## 文档入口

阶段计划：
- [阶段 1 计划](notes/plans/archive/stage1_learning_plan.md)
- [阶段 2 计划](notes/plans/archive/stage2_learning_plan.md)
- [阶段 3 计划](notes/plans/archive/stage3_learning_plan.md)
- [阶段 4 计划](notes/plans/archive/stage4_learning_plan.md)

阶段 3 实验记录：
- [Task 7A：MoveIt Quickstart](notes/labs/task7A_moveit_quickstart.md)
- [Task 7B：MoveGroupInterface C++](notes/labs/task7B_move_group_interface_cpp.md)
- [Task 7C：Planning Scene 与碰撞约束](notes/labs/task7C_planning_scene_collision.md)
- [Task 7D：目标位姿自动规划](notes/labs/task7D_goal_pose_auto_plan.md)
- [Task 7E：MoveIt Servo](notes/labs/task7E_moveit_servo.md)

阶段 4 实验记录：
- [Task 8A：真机接入前安全与网络清单](notes/labs/task8A_real_robot_preflight.md)
- [Task 8B：只读启动 ur_control 并验证状态流](notes/labs/task8B_real_robot_readonly_bringup.md)
- [Task 8C：Dashboard、controller 与 External Control 状态验证](notes/labs/task8C_dashboard_controller_state.md)

经验索引：
- [experience/index.json](experience/index.json)

## 常用命令

构建某个工作区：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

只构建 Task 7E：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_servo_lab_cpp
source install/setup.bash
```

运行 Task 7E fake hardware 验收 harness：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run ur3_moveit_servo_lab_cpp run_task7e_full_test.py \
  --workspace-root . \
  --max-attempts 1 \
  --timeout-sec 60 \
  --trigger-count 8 \
  --use-mock-hardware true \
  --launch-rviz true
```

手动触发一轮 Servo 速度命令：

```bash
ros2 service call /ur3_servo_twist_commander/start_motion std_srvs/srv/Trigger "{}"
```

构建阶段 4 真机 bringup 包：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_real_bringup_lab
source install/setup.bash
```

运行 Task 8C 只读状态门闩：

```bash
ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py
```

运行 Task 8C 动作前门闩演示（只检查，不激活 controller）：

```bash
ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
  require_trajectory_controller_active:=true
```

## 当前边界

- 阶段 3 的主路径已完成，但 Task 7E 仅验收 fake hardware + RViz；URSim、真机、遥操作输入设备和视觉伺服留作后续独立任务。
- 阶段 4 已进入真实 UR3e，但目前只完成 8A-8C：网络/安全预检、只读状态流和状态门闩。尚未允许真实运动。
- 真机 calibration 文件已提取并归档，但进入 8D 前还需要接入 bringup/description wrapper，确认 `calibration mismatch` 消失后再讨论运动。
- Task 7B 的 `ros2 run` 直跑会绕过 launch 注入，可能复现本地 kinematics warning；当前推荐使用对应 launch 入口。
- 本仓库是学习仓库，不是生产机器人控制系统。进入 URSim 或真机前，应重新审查速度、停止策略、控制器状态、网络与安全边界。
