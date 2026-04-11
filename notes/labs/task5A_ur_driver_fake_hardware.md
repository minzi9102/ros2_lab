# Task 5A：跑通 UR ROS 2 Driver（fake hardware）

## 1. 目标
- 在 fake hardware 模式下完整启动 UR3 控制链路
- 验证三项指标：joint states、controller 状态、TF 树

## 2. 环境信息
- ROS 2 版本：Jazzy
- `ur_robot_driver` 版本：3.7.0
- launch 文件路径：`/opt/ros/jazzy/share/ur_robot_driver/launch/ur_control.launch.py`

## 3. 启动步骤

### Step 1：source 环境
```bash
source /opt/ros/jazzy/setup.bash
```

### Step 2：以 mock hardware 模式启动 UR3
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true \
  launch_rviz:=true
```

> ⚠️ 参数名陷阱（v3.7.0 变更）：旧文档/网络资料中常见的 `use_fake_hardware:=true` 在 v3.7.0 **已失效**，正确参数为 `use_mock_hardware:=true`。
> `robot_ip` 在 mock 模式下不会实际连接，但必须填合法 IP 格式（`yyy.yyy.yyy.yyy` 无法被 DNS 解析会导致 crash）。

## 4. 验证三项指标

### 4.1 验证 joint states
```bash
ros2 topic echo /joint_states --once
```
**预期输出**：6 个关节的 name / position / velocity / effort

**实际输出**：
```yaml
header:
  frame_id: base_link
name: [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
position: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]   # 单位：弧度
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort:   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

**关键认知**：
- `shoulder_lift_joint` 和 `wrist_1_joint` 初始值为 `-1.57 rad`（≈ -90°），而非全 0
- 这是 UR3 的**初始上电姿态**：肩部向后倾 90°、腕 1 弯曲 90°，使机械臂保持竖直向上的"自然站立"形态，而非平躺
- 若全关节为 0，机械臂会水平展开（奇异位型），实际使用中几乎不以此为起点

### 4.2 验证 controller 状态
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```
**预期**：`joint_state_broadcaster` 和 `scaled_joint_trajectory_controller`（或类似）均为 `active`

**实际输出（mock hardware 模式下 active 的 controllers）**：

| controller | 职责 |
|---|---|
| `scaled_joint_trajectory_controller` | **核心运动控制器**。继承自标准 `joint_trajectory_controller`，新增速度缩放（Speed Scaling）功能，根据示教器速度滑块实时调整运动节奏，是 UR 机器人最关键的控制器 |
| `joint_state_broadcaster` | **状态发布桥梁**。从硬件接口读取 6 个关节的位置/速度/力矩，发布到 `/joint_states`。没有它 RViz 模型不会动，TF 树也会断裂 |
| `io_and_status_controller` | **IO 与状态管理**。处理通用 GPIO、工具端 IO 及机器人状态（上电、急停等） |
| `speed_scaling_state_broadcaster` | **速度比例发布**。将示教器速度滑块当前比例（0~100%）发布出来，供 `scaled_joint_trajectory_controller` 消费 |
| `force_torque_sensor_broadcaster` | **力矩传感器**。读取末端法兰内置 F/T 传感器数据并发布 |
| `ur_configuration_controller` | **配置管理接口**。通过 Service 动态修改驱动内部设置（Payload、TCP 偏移等），无需重启驱动 |

**关键发现**：`scaled_joint_trajectory_controller` 与 `speed_scaling_state_broadcaster` 是一对协同关系——后者发布速度比例，前者根据该值动态调整轨迹执行速度，这是 UR 驱动区别于通用 `joint_trajectory_controller` 的核心差异。

### 4.3 验证 TF 树
```bash
ros2 run tf2_tools view_frames
```
或在 RViz 中添加 TF display。

**预期**：`base_link → shoulder_link → ... → tool0` 链路完整，无断链警告

**实际结果**：
- 根节点：`world`
- 总 frame 数：13 个
- `tool0` 位于最底层，父节点为 `flange`
- 完整链路：`world → base → base_link → shoulder_link → upper_arm_link → forearm_link → wrist_1_link → wrist_2_link → wrist_3_link → flange → tool0`（含 `base_link_inertia`、`robotiq_85_*` 等辅助 frame）

**关键认知**：
- 根节点是 `world` 而非 `base_link`——UR 官方 URDF 在 `base_link` 之上还有一层 `world` 固定 frame，自建 xacro 通常以 `base_link` 为根
- `flange → tool0`：`flange` 是机械法兰接口（物理安装面），`tool0` 是 ROS 控制坐标系（默认与 flange 重合，安装末端工具后可偏移）

## 5. 关键参数说明

| 参数 | 含义 | mock hardware 下的值 |
|---|---|---|
| `ur_type` | 机器人型号 | `ur3` |
| `robot_ip` | 机器人 IP | 任意合法 IP（如 `192.168.56.101`），不实际连接 |
| `use_mock_hardware` | 启用软件模拟硬件接口（v3.7.0+） | `true` |
| `launch_rviz` | 同时启动 RViz | `true` |

> ⚠️ `use_fake_hardware` 是旧参数名（v2.x 及更早），v3.7.0 已统一改为 `use_mock_hardware`。

## 6. 控制链路层次理解

| 层次 | 组件 | 职责 |
|---|---|---|
| 驱动层 | `ur_robot_driver` | 与硬件/仿真通信，暴露 hardware interface |
| 控制器层 | `ros2_control_node` + controllers | 读取 hardware interface，执行控制逻辑 |
| 状态发布层 | `joint_state_broadcaster` | 将 hardware interface 状态发布为 `/joint_states` |
| 描述层 | `robot_state_publisher` | 将 URDF + joint states 转换为 TF 树 |

fake hardware 模式：驱动层用软件模拟硬件接口，其余层与真机完全相同。

## 7. 发现与疑问

**发现**：
- v3.7.0 将 `use_fake_hardware` 参数重命名为 `use_mock_hardware`，旧参数名不被识别
- 即使在 mock 模式，`robot_ip` 仍须填合法 IP 格式，`yyy.yyy.yyy.yyy` 会触发 DNS 解析失败导致 crash
- mock 模式下仍加载 `URPositionHardwareInterface`（内部实现了 mock 逻辑），而非 `mock_components/GenericSystem`

**疑问**：
- 无

## 8. 与阶段 1 自建方案的对比

| 对比项 | 阶段 1 自建 xacro | 5A UR 官方驱动 |
|---|---|---|
| URDF 来源 | 手写 xacro | `ur_description` 包提供 |
| hardware interface | `mock_components/GenericSystem` | `ur_robot_driver/URPositionHardwareInterface` |
| 控制器配置 | 手写 yaml | 官方预配置 |
| 启动方式 | 自写 launch | `ur_control.launch.py` |

## 9. 验收状态
- [x] `ros2 topic echo /joint_states` 输出 6 个关节数据（position 非全零，shoulder_lift/wrist_1 初始为 -1.57 rad）
- [x] `ros2 control list_controllers` 显示 6 个 active controller（含 scaled_joint_trajectory_controller、joint_state_broadcaster 等）
- [x] TF 树根节点为 `world`，共 13 个 frame，`tool0` 位于最底层，父节点为 `flange`，链路完整无断链
- [x] 能口头解释 mock hardware 与真机模式的本质区别：驱动层用 URPositionHardwareInterface 内部 mock 逻辑替代真实 RTDE 通信，其余控制器层/状态发布层/描述层与真机完全相同

## 10. 完成记录
- 状态：`[x] 已完成`
- 日期：2026-04-11
- 备注：v3.7.0 参数名变更（use_fake_hardware → use_mock_hardware）为本次最关键的排错经验；三项验证指标全部通过
