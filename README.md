# ros2_lab

ROS 2 Jazzy 机械臂开发学习仓库，以 UR3 为目标平台，按阶段推进 ROS 2 基础通信、`ros2_control`、UR driver、URSim、MoveIt 2 与 MoveIt Servo。

本仓库默认采用 learn mode：智能体负责计划、脚手架、上下文整理、集成与校验；关键设计和有学习价值的实现优先交给人类完成。协作细则见 [AGENTS.md](AGENTS.md)。

## 当前状态

| 阶段 | 内容 | 状态 |
|---|---|---|
| 阶段 1 | ROS 2 基础能力与控制链路入门（Task 4A-4H） | 已完成 |
| 阶段 2 | UR3 fake hardware / URSim 控制实践（Task 5A-6B） | 已完成 |
| 阶段 3 | UR3 + MoveIt 2 运动规划与 Servo（Task 7A-7E） | 已完成 |

阶段 3 已于 2026-04-27 收口。最近的主线历史集中在 Task 7E：MoveIt Servo fake hardware 最小闭环、启动门闩、静止态 joint state relay、重复 Trigger harness 与验收经验沉淀。

## 仓库地图

```text
ros2_lab/
├── AGENTS.md                 # learn mode 协作规则
├── CLAUDE.md                 # Claude/Codex 协作补充说明
├── README.md
├── archive/                  # 归档内容
├── docs/                     # 外部资料或补充文档入口
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
    └── ws_stage3/            # 阶段 3：MoveIt 2 与 MoveIt Servo
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

## 文档入口

阶段计划：
- [阶段 1 计划](notes/plans/archive/stage1_learning_plan.md)
- [阶段 2 计划](notes/plans/archive/stage2_learning_plan.md)
- [阶段 3 计划](notes/plans/archive/stage3_learning_plan.md)

阶段 3 实验记录：
- [Task 7A：MoveIt Quickstart](notes/labs/task7A_moveit_quickstart.md)
- [Task 7B：MoveGroupInterface C++](notes/labs/task7B_move_group_interface_cpp.md)
- [Task 7C：Planning Scene 与碰撞约束](notes/labs/task7C_planning_scene_collision.md)
- [Task 7D：目标位姿自动规划](notes/labs/task7D_goal_pose_auto_plan.md)
- [Task 7E：MoveIt Servo](notes/labs/task7E_moveit_servo.md)

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

## 当前边界

- 阶段 3 的主路径已完成，但 Task 7E 仅验收 fake hardware + RViz；URSim、真机、遥操作输入设备和视觉伺服留作后续独立任务。
- Task 7B 的 `ros2 run` 直跑会绕过 launch 注入，可能复现本地 kinematics warning；当前推荐使用对应 launch 入口。
- 本仓库是学习仓库，不是生产机器人控制系统。进入 URSim 或真机前，应重新审查速度、停止策略、控制器状态、网络与安全边界。
