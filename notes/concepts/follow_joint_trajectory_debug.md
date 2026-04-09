# Task4F joint_trajectory_controller 联调学习记录（填写模板）

## 1. 本次学习目标（用自己的话）
- 目标清单（完成后勾选）：
  - [X] 我能解释 `joint_trajectory_controller` 与 `action interface` 的职责边界： `joint_trajectory_controller` 负责硬件接口读和写，它实现了 `FollowJointTrajectory` action server，能够接收目标轨迹中离散的轨迹点并插值成为一条平滑的轨迹曲线（包括位置、速度、力矩），从硬件接口读取当前状态，计算误差并写入控制指令；`FollowJointTrajectory` action 是通信协议，定义了 goal/feedback/result 的数据结构，让外部客户端能够发送轨迹目标、监听执行进度、获取最终结果。
  - [X] 我能说清 `FollowJointTrajectory` action 的 goal/feedback/result 结构：GOAL中包括TRAJECTORY（一系列有序离散的轨迹点，每个点包括关节的位置，速度，加速度以及到达该点的时间戳），MULTI_DOF_TRAJECTORY（不能用关节表示的机器人，如无人机等，可以用这个字段来填入位置，方向等类型的轨迹点）；TOLERANCE（包括在路径中运行时的容差，在终点的容差，以及时间上的容差）FEEDBACK中包括HEADER（当前的时间戳和轨迹的参考坐标系），JOINT_NAMES关节名称列表，DESIRED（预期当前机器人应该在的位置），ACTUAL（实际状态），ERROR（误差状态）；RESULT中包括ERROR_CODE（状态码，0代表成功），ERROR_STRING(错误的具体描述)
  - [X] 我能独立完成 action 接口对齐与关节名校验
  - [X] 我能发送单点与多点轨迹目标并观测 feedback/result
  - [X] 我能复现 1 个常见失败并定位到根因

## 2. 实验环境

| 项目 | 内容 |
|---|---|
| 日期 | `2026.4.8` |
| 系统 | `ubuntu24` |
| ROS 2 版本 | `jazzy` |
| 工作区 | `/home/minzi/ros2_lab/workspaces/ws_tutorials` |
| 任务包 | `ur3_joint_trajectory_controller_lab_py` |
| 启动文件 | `launch/ur3_joint_trajectory_controller.launch.py` |

## 3. 控制链路总览（先写结构，再写数据流）
- 启动链路（建议按下面格式补全）：
  - `launch -> xacro -> robot_description -> robot_state_publisher`
  - `launch -> xacro -> robot_description + controllers.yaml -> ros2_control_node(controller_manager)`
  - `spawner -> /controller_manager/* services -> joint_state_broadcaster (active)`
  - `spawner -> /controller_manager/* services -> joint_trajectory_controller (active)`
- 运行时数据流（建议按下面格式补全）：
  - `hardware state_interface -> joint_state_broadcaster -> /joint_states`
  - `/joint_states + robot_description -> robot_state_publisher -> /tf + /tf_static`
  - `action client -> /joint_trajectory_controller/follow_joint_trajectory (action) -> joint_trajectory_controller -> hardware command_interface`

## 4. 文件结构与作用梳理（4F 相关）
- 应用包目录：`workspaces/ws_tutorials/src/ur3_joint_trajectory_controller_lab_py`
- 文件清单（补充你自己的理解）：

| 文件 | 作用 | 你需要特别关注的点 |
|---|---|---|
| `urdf/ur3_simplified_ros2_control.urdf.xacro` | `读取机械臂的几何结构，以及添加了专为ros2_control服务的硬件接口定义` | `文件引用了外部的xacro文件` |
| `config/ur3_controllers_with_trajectory.yaml` | `定义了joint_state_broadcaster和joint_trajectory_controller控制器` | `关节顺序必须与URDF一致` |
| `launch/ur3_joint_trajectory_controller.launch.py` | `自动配置启动参数，搭建robot_description，然后启动robot_state_publisher和ros2_control_node` | `无` |
| `scripts/test_trajectory_client.py` | `最小轨迹联调脚本，发送3点轨迹目标` | `action接口名必须与控制器配置一致` |
| `package.xml` | `配置程序包的依赖项` | `由ai来填写` |
| `setup.py` | `配置程序包的可运行项目` | `确保 data_files 覆盖 launch/config/urdf，entry_points 覆盖可执行脚本` |

## 5. 启动与验证步骤（按执行顺序填写）

### 5.1 构建与启动
```bash
cd /home/minzi/ros2_lab/workspaces/ws_tutorials
colcon build --packages-select ur3_joint_trajectory_controller_lab_py
source install/setup.bash
ros2 launch ur3_joint_trajectory_controller_lab_py ur3_joint_trajectory_controller.launch.py
```

### 5.2 关键核验命令
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 action list
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{trajectory: {header: {frame_id: base_link}, joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], points: [{positions: [0, 0, 0, 0, 0, 0], time_from_start: {sec: 1, nanosec: 0}}]}}"
```

### 5.3 你本次实际执行记录
- 启动是否成功：`成功`
- 两个控制器状态是否均为 active：`是`
- action 接口是否可见：`是，/joint_trajectory_controller/follow_joint_trajectory`
- 单点轨迹是否成功：`是`
- 多点轨迹是否成功：`是`

## 6. A/B/C/D 实验矩阵（最小可复现）

| 组别 | 输入条件 | 预期现象 | 实际现象 | 结论 |
|---|---|---|---|---|
| A 基线 | 启动 manager，激活两个控制器 | 两个控制器均为 active | `joint_trajectory_controller joint_trajectory_controller/JointTrajectoryController  active joint_state_broadcaster     joint_state_broadcaster/JointStateBroadcaster          active` | `成功启动joint_state_broadcaster和joint_trajectory_controller两个controller` |
| B 单点 | 发送单点轨迹目标 | goal 被接受，result 返回成功 | `日志提示goal accepted且finished` | `可以成功执行` |
| C 多点 | 发送 3 点平滑轨迹 | goal 被接受，feedback 持续更新，result 返回成功 | `goal 被接受，feedback 持续更新，result 返回成功` | `可成功执行` |
| D 异常 | 人为改错关节名或时间参数 | action 返回失败或超时 | `ur3_joint_trajectory_controller.launch.py不能启动，what():  Joint 'shoulder_pen_joint' not found in URDF` | `无法启动控制器` |

## 7. 故障排查记录（至少写 1 条）

| 现象 | 关键报错 | 根因判断 | 修复动作 | 复验结果 |
|---|---|---|---|---|
| `不能启动ur3_joint_trajectory_controller.launch.py` | `Failed to activate controller : joint_trajectory_controller` | `YAML 里设置了 use_sim_time: true，use_sim_time: true 时，ROS2 节点会等待 /clock 话题提供时间。没有 Gazebo 或其他仿真器发布时钟，控制器的激活流程就会卡住直到超时。` | `use_sim_time改为false` | `修复成功` |
| `发送action goal后没有动静` | `Waiting for an action server to become available...` | `action 接口名不对。刚才 ros2 action list 显示的是：/joint_trajectory_controller/follow_joint_trajectory但命令里用的是 /follow_joint_trajectory（缺少前缀），所以一直在等待一个不存在的 server。` | `正确的接口名重发` | `修复成功` |

排查顺序建议（可删改）：
1. 先看 `ros2_control_node` 首个 `ERROR/what()`。
2. 再用 `ros2 action list` 确认 action 接口是否可见。
3. 核对 `joint_trajectory_controller` YAML 中的关节名与 URDF 是否逐字一致。
4. 最后用 `ros2 action send_goal` 发送单点目标验证接口连通。

## 8. 关键命令与日志证据（贴你自己的）
- 命令片段：
```bash
ros2 control list_controllers
ros2 action list
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], points: [{positions: [0.5, -0.5, 0.5, 0.0, 0.0, 0.0], velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2, nanosec: 0}}]}}"
```
- 关键日志片段（C组）：
```text
Waiting for an action server to become available...
Sending goal:
     trajectory:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  joint_names:
  - shoulder_pan_joint
  - shoulder_lift_joint
  - elbow_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint
  points:
  - positions:
    - 0.5
    - -0.5
    - 0.5
    - 0.0
    - 0.0
    - 0.0
    velocities:
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    - 0.0
    accelerations: []
    effort: []
    time_from_start:
      sec: 2
      nanosec: 0
multi_dof_trajectory:
  header:
    stamp:
      sec: 0
      nanosec: 0
    frame_id: ''
  joint_names: []
  points: []
path_tolerance: []
component_path_tolerance: []
goal_tolerance: []
component_goal_tolerance: []
goal_time_tolerance:
  sec: 0
  nanosec: 0

Goal accepted with ID: 4cffe19f69ab4e0db82b5fbe80e8db77

Result:
    error_code: 0
error_string: Goal successfully reached!

Goal finished with status: SUCCEEDED
```

## 9. 结论（面向 4F 验收）
- 本次联调是否通过：`通过`
- 我能解释的职责边界（3-5 句）：`JTC负责接收TRAJECTORY,即一系列离散的轨迹点，并能够将他们插值成为一条平滑的轨迹，每个轨迹点包含包括关节位置，速度，加速度等信息，读取硬件接口，接收机器人当前状态，并根据机器人当前状态和目标状态计算运动控制指令，写入硬件控制接口，指示机器人进行运动`
- 本次最大的收获：`了解了ACTION从发出到执行的流程`
- 当前仍存在的风险：`不知道ACTION信息内部具体的内容`

## 10. 下一步（衔接 4G）
- 进入 4G 前需要准备：
  - [ ] 明确 `forward_command_controller` 与 `joint_trajectory_controller` 的差异
  - [ ] 准备对比实验矩阵
  - [ ] 确认两个控制器的并行加载策略

## 11. 完成记录
- 状态：`[x] 已完成`
- 日期：2026.4.9
- 备注：无
