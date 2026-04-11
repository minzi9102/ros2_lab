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

### Step 2：以 fake hardware 模式启动 UR3
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true \
  launch_rviz:=true
```

> 注意：fake hardware 模式下 `robot_ip` 不会真正连接，填任意 IP 即可。

## 4. 验证三项指标

### 4.1 验证 joint states
```bash
ros2 topic echo /joint_states --once
```
**预期输出**：6 个关节的 name / position / velocity / effort

**实际输出**：
```
（待填写）
```

### 4.2 验证 controller 状态
```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```
**预期**：`joint_state_broadcaster` 和 `scaled_joint_trajectory_controller`（或类似）均为 `active`

**实际输出**：
```
（待填写）
```

### 4.3 验证 TF 树
```bash
ros2 run tf2_tools view_frames
```
或在 RViz 中添加 TF display。

**预期**：`base_link → shoulder_link → ... → tool0` 链路完整，无断链警告

**实际结果**：
```
（待填写）
```

## 5. 关键参数说明

| 参数 | 含义 | fake hardware 下的值 |
|---|---|---|
| `ur_type` | 机器人型号 | `ur3` |
| `robot_ip` | 机器人 IP | 任意（不实际连接） |
| `use_fake_hardware` | 启用软件模拟硬件接口 | `true` |
| `launch_rviz` | 同时启动 RViz | `true` |

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
- （待填写）

**疑问**：
- （待填写）

## 8. 与阶段 1 自建方案的对比

| 对比项 | 阶段 1 自建 xacro | 5A UR 官方驱动 |
|---|---|---|
| URDF 来源 | 手写 xacro | `ur_description` 包提供 |
| hardware interface | `mock_components/GenericSystem` | `ur_robot_driver/URPositionHardwareInterface` |
| 控制器配置 | 手写 yaml | 官方预配置 |
| 启动方式 | 自写 launch | `ur_control.launch.py` |

## 9. 验收状态
- [ ] `ros2 topic echo /joint_states` 输出 6 个关节数据
- [ ] `ros2 control list_controllers` 显示至少 2 个 active controller
- [ ] RViz TF 树 `base_link → tool0` 链路完整
- [ ] 能口头解释 fake hardware 与真机模式的本质区别

## 10. 完成记录
- 状态：`[#] 进行中`
- 日期：2026-04-11
- 备注：工作区 `ws_stage2` 已建立，驱动已确认安装，等待执行验证步骤
