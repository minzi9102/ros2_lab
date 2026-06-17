# ws_realtime_control 实时控制工作区计划

本文档定义新建独立工作区 `ws_realtime_control` 的实时控制实验方案。

核心决策是：不把实时控制代码放进 `ws_ur3e_controller`，而是新建一个专门工作区，作为后续键盘控制、手柄控制、脚踏控制、视觉伺服等实时控制实验的入口。

当前实现状态：

```text
核心包 v1 已进入实现：创建 ur3e_keyboard_servo_py，覆盖键盘映射、安全限幅、终端按键读取、TwistStamped 发布节点和单元测试。
仿真 launch 切片已进入实现：新增 sim_keyboard_servo.launch.py 和 sim_keyboard_servo.yaml，复用 Stage 3 Task 7E 的 MoveIt Servo 启动门闩。
fake hardware smoke test 已跑到 Servo + keyboard 节点，并完成 TWIST mode 握手；RViz 中已确认机械臂响应键盘输入并运动，四方向逐项运动矩阵仍可继续补充。
`launch_rviz:=true` 的顶层 RViz 启动已修复并验证能启动到 MoveIt MotionPlanning 面板。
键盘输入已补充 `/dev/tty` fallback；下一次方向验收时需要保持启动终端获得焦点，并观察 `Key command received` 日志。
fake hardware 下键盘输入与运动人工验收已成功记录：终端聚焦后可连续收到 `Key command received: action=move ...`，并在 RViz 中看到机械臂运动。
URSim smoke test 已通过：不启动 RViz，通过 URSim 网页监视器确认机械臂响应键盘 Servo 输入并运动。
真机前置只读检查已通过：RUNNING/NORMAL、External Control running、remote_control=True、/joint_states 约 501 Hz、speed scaling 100.0。
真机安全 launch 已进入实现：新增 real_keyboard_servo.launch.py 和 real_keyboard_servo.yaml，强制确认口令、低速、短超时、30 秒会话上限。
真机 launch 启动顺序已调整为复用 Task 8B：driver hardware ready 后再启动 dashboard / External Control manager，再等待 forward_position_controller 后启动 Servo 和键盘节点。
真机键盘输入链路已确认：短按按键时终端连续输出 `Key command received`，但 5 mm/s 默认速度下肉眼运动不明显；已计划通过 launch 参数进行保守调参。
仿真完整方向矩阵和真机短按验证仍保留为后续独立任务。
```

当前第一版只规划：

```text
键盘方向键 -> x/y 平面速度命令 -> MoveIt Servo -> UR3e 机械臂运动
```

MoveIt Servo 是 MoveIt 2 中用于连续速度控制的模块，适合“按住按键就持续运动，松开就停止”的控制方式。

## 1. 新工作区定位

建议新建：

```text
workspaces/ws_realtime_control
```

定位：

```text
ws_realtime_control = 实时机械臂控制实验工作区
```

不命名为 `ws_keyboard_control`，因为键盘只是第一种输入方式；但第一版也不做复杂架构。工作区名称保持通用，内部第一阶段只放一个键盘伺服控制包。

## 2. 为什么要单独建工作区

原仓库已经按阶段分得很清楚：

- `ws_tutorials` 做基础学习。
- `ws_stage2` 做 UR3 控制链路。
- `ws_stage3` 做 MoveIt 2 和 Servo。
- `ws_stage4` 做真实 UR3e 的安全接入与低速动作。
- `ws_ur3e_controller` 做命名目标控制器和图形界面闭环。

新建工作区的好处：

| 目的 | 说明 |
| --- | --- |
| 不污染旧阶段 | 阶段 3、阶段 4 已经是学习任务和验收记录，不应继续塞新功能 |
| 便于真机隔离 | 实时控制比 HOME/READY 更危险，需要独立启动入口 |
| 便于后续扩展 | 以后可以逐步加入手柄、脚踏、视觉输入，但第一版不做 |
| 便于安全审查 | 所有实时控制代码集中在一个工作区，真机前更容易检查 |

仓库 README 已说明，阶段 3 的 Servo 主路径已经完成，但“遥操作输入设备”仍是后续独立任务；进入 URSim 或真机前，仍要重新审查速度、停止策略、控制器状态、网络与安全边界。

## 3. 工作区目录设计

后续实现阶段建议目录结构：

```text
workspaces/ws_realtime_control/
├── README.md
├── REALTIME_CONTROL_PLAN.md
├── src/
│   └── ur3e_keyboard_servo_py/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       │   └── ur3e_keyboard_servo_py
│       ├── ur3e_keyboard_servo_py/
│       │   ├── __init__.py
│       │   ├── keyboard_servo_node.py
│       │   ├── key_mapping.py
│       │   ├── safety_limiter.py
│       │   └── terminal_key_reader.py
│       ├── config/
│       │   ├── sim_keyboard_servo.yaml
│       │   └── real_keyboard_servo.yaml
│       ├── launch/
│       │   ├── sim_keyboard_servo.launch.py
│       │   └── real_keyboard_servo.launch.py
│       └── test/
│           ├── test_key_mapping.py
│           └── test_safety_limiter.py
├── notes/
│   ├── sim_fake_hardware_validation.md
│   ├── sim_ursim_validation.md
│   └── real_robot_validation.md
└── scripts/
    ├── build_ws.sh
    ├── run_sim_fake.sh
    ├── run_sim_ursim.sh
    └── run_real.sh
```

第一版只做这一个软件包：

```text
ur3e_keyboard_servo_py
```

使用 Python 写节点。键盘读取和速度发布逻辑简单，用 Python 更符合 KISS 原则；真正的底层运动控制继续交给 MoveIt Servo 和机器人驱动。

## 4. 工作区边界

这个新工作区只负责：

```text
输入设备 -> 速度命令生成 -> 安全限幅 -> 发布给 MoveIt Servo
```

不负责：

```text
机器人驱动
MoveIt 配置
URDF 模型
真机标定
Dashboard 自动恢复
Protective stop 自动解锁
复杂路径规划
```

它是一个“实时输入控制层”，不是完整机器人控制系统。

## 5. 第一版功能范围

只做：

| 功能 | 说明 |
| --- | --- |
| ↑ | 末端沿 +x 方向移动 |
| ↓ | 末端沿 -x 方向移动 |
| ← | 末端沿 +y 方向移动 |
| → | 末端沿 -y 方向移动 |
| 空格 | 立即停止 |
| q | 退出并停止 |
| 松键停止 | 超过短时间没有按键输入，就自动发布零速度 |
| 速度限制 | 仿真稍快，真机极低速 |

第一版不做：

```text
z 轴
姿态旋转
速度档位
组合键
图形界面
手柄
脚踏
视觉伺服
自动恢复
轨迹录制
多机器人
复杂日志系统
```

这样更符合 YAGNI 和 KISS：只做当前明确需要的能力，并保持方案简单、清晰、可验证。

## 6. 核心节点设计

### 6.1 `keyboard_servo_node.py`

职责：

```text
读取键盘 -> 查询当前按键状态 -> 生成 x/y 速度 -> 发布速度消息
```

发布的话题：

```text
/servo_node/delta_twist_cmds
```

发布的消息类型：

```text
TwistStamped
```

第一版只使用：

```text
linear.x
linear.y
```

其余字段全部保持 0。

### 6.2 `key_mapping.py`

只做按键映射：

```text
↑  -> x 正方向
↓  -> x 负方向
←  -> y 正方向
→  -> y 负方向
空格 -> 停止
q  -> 退出
```

单独拆出来是为了方便测试，不需要启动机器人也能验证方向是否正确。

### 6.3 `safety_limiter.py`

只做最小安全限制：

```text
限制最大线速度
禁用 z 轴
禁用旋转
松键超时归零
退出前归零
```

第一版不做复杂碰撞检测。MoveIt Servo 已承担一部分运动约束检查，新节点只做输入侧最小限幅。

### 6.4 `terminal_key_reader.py`

只负责终端按键读取。

第一版不做图形界面。`ws_ur3e_controller` 的图形界面方案已经明确采用薄客户端思路：界面只发送请求，不复制后端安全逻辑、不自动重试、不自动恢复。键盘控制第一版也应保持这个原则。

## 7. 仿真阶段方案

目标：

```text
键盘 -> 速度命令 -> MoveIt Servo -> 假硬件 / URSim -> RViz 观察末端运动
```

### 7.1 仿真启动文件

文件：

```text
launch/sim_keyboard_servo.launch.py
```

建议支持两个模式：

```text
use_mock_hardware:=true   # 假硬件
use_mock_hardware:=false  # URSim
```

第一版一个仿真启动文件即可，不拆成很多启动入口。

### 7.2 仿真参数

文件：

```text
config/sim_keyboard_servo.yaml
```

建议：

```yaml
command_topic: /servo_node/delta_twist_cmds
frame_id: base_link
publish_rate_hz: 30.0
key_timeout_sec: 0.20
linear_speed_mps: 0.02
enable_z: false
enable_rotation: false
```

### 7.3 启动顺序

仿真启动文件应复用阶段 3 Task 7E 的经验。Task 7E 不是一上来就启动 commander，而是先启动驱动和 MoveIt，再确认 `/joint_states` 和控制器，再启动 Servo，最后才启动控制节点。

新工作区也按这个顺序：

```text
1. 启动机器人驱动或假硬件
2. 启动 MoveIt
3. 等待 /joint_states
4. 等待必要控制器 active
5. 启动 MoveIt Servo
6. 等待 /servo_node/status
7. 启动 keyboard_servo_node
```

### 7.4 仿真验收步骤

假硬件：

```bash
cd ~/ros2_lab/workspaces/ws_realtime_control
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash

ros2 launch ur3e_keyboard_servo_py sim_keyboard_servo.launch.py \
  use_mock_hardware:=true \
  launch_rviz:=true
```

验收：

| 操作 | 通过标准 |
| --- | --- |
| 不按键 | 机械臂不动 |
| ↑ | 末端沿 +x 小幅移动 |
| ↓ | 末端沿 -x 小幅移动 |
| ← | 末端沿 +y 小幅移动 |
| → | 末端沿 -y 小幅移动 |
| 松开按键 | 自动停止 |
| 空格 | 立即停止 |
| q | 退出并停止 |

URSim：

```bash
ros2 launch ur3e_keyboard_servo_py sim_keyboard_servo.launch.py \
  use_mock_hardware:=false \
  robot_ip:=<URSim_IP> \
  launch_rviz:=true
```

验收：

| 项目 | 标准 |
| --- | --- |
| External Control | 程序运行 |
| speed scaling | 非零 |
| RViz | 运动方向正确 |
| URSim | 运动方向正确 |
| 停止 | 松键、空格、退出均停住 |

## 8. 真机阶段方案

目标：

```text
键盘方向键 -> 真实机械臂末端在 x/y 平面微小移动
```

真机第一版不是“自由遥操作”，而是“低速、短时、人工旁站、随时急停”的验证实验。

### 8.1 真机启动文件

文件：

```text
launch/real_keyboard_servo.launch.py
```

真机启动必须默认不允许运动。

参数建议：

```yaml
command_topic: /servo_node/delta_twist_cmds
frame_id: base_link
publish_rate_hz: 30.0
key_timeout_sec: 0.25
linear_speed_mps: 0.010
enable_z: false
enable_rotation: false
require_confirmation: true
human_confirmation: ""
max_session_duration_sec: 45.0
```

必须显式传入确认口令才允许运动：

```bash
ros2 launch ur3e_keyboard_servo_py real_keyboard_servo.launch.py \
  human_confirmation:=I_CONFIRM_REAL_ROBOT_MOTION
```

这个设计参考阶段 4 guarded motion 的做法：真机执行需要 `human_confirmation`，并且对动作范围和最终状态有门闩参数。

真机 launch 允许临时覆盖低速验证参数，但必须通过硬上限：

```text
linear_speed_mps <= 0.050
key_timeout_sec <= 0.50
max_session_duration_sec <= 90.0
```

超过硬上限或传入非正数时，launch 必须拒绝启动。

### 8.2 真机前置检查

真机前先运行阶段 4 的只读状态门闩：

```bash
ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
  require_trajectory_controller_active:=true
```

检查器本身只读，不发送运动命令，适合作为实时控制前的安全检查。

必须通过：

```text
robot mode 正常
safety mode 正常
External Control 正在运行
joint_state_broadcaster active
Servo 所需控制器 active
/joint_states 正常
speed scaling 非零
没有标定不匹配
```

如果出现 `BLOCK`，不得启动键盘控制。

### 8.3 真机启动顺序

```text
1. 人工检查机械臂周围空间
2. 运行 Task 8C 状态门闩
3. 启动 real_keyboard_servo.launch.py
4. 不按键，确认机械臂静止
5. 短按 ↑，观察 +x 方向
6. 空格停止
7. 短按 ↓、←、→
8. q 退出
9. 记录 notes/real_robot_validation.md
```

### 8.4 真机验收标准

| 项目 | 标准 |
| --- | --- |
| 默认状态 | 不传确认口令时不能运动 |
| 不按键 | 不发布非零速度 |
| 松键 | 0.15 秒内归零 |
| 空格 | 立即归零 |
| q | 退出前归零 |
| ↑ ↓ ← → | 方向正确 |
| 速度 | 默认 0.010 m/s，最高只允许覆盖到 0.050 m/s |
| z 轴 | 永远为 0 |
| 旋转 | 永远为 0 |
| 会话时长 | 默认 45 秒，最高只允许覆盖到 90 秒 |
| 异常恢复 | 第一版不做自动恢复 |

## 9. 新工作区与旧工作区的关系

新工作区不复制：

```text
ur_robot_driver
ur_moveit_config
ur3_real_bringup_lab
ur3_real_guarded_motion_lab_cpp
ur3e_named_motion_controller_cpp
```

而是通过启动文件引用已有包。

推荐使用方式：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_lab/workspaces/ws_stage4/install/setup.bash
source ~/ros2_lab/workspaces/ws_ur3e_controller/install/setup.bash
source ~/ros2_lab/workspaces/ws_realtime_control/install/setup.bash
```

如果某些包已经来自系统安装，就不需要重复 source 对应工作区。

## 10. 开发任务拆分

### 任务 1：创建工作区骨架

产出：

```text
workspaces/ws_realtime_control
workspaces/ws_realtime_control/README.md
workspaces/ws_realtime_control/src/ur3e_keyboard_servo_py
```

验收：

```bash
cd workspaces/ws_realtime_control
colcon build
```

能空构建通过。

### 任务 2：实现键盘到速度映射

产出：

```text
key_mapping.py
test_key_mapping.py
```

验收：

```text
↑ -> linear.x > 0
↓ -> linear.x < 0
← -> linear.y > 0
→ -> linear.y < 0
空格 -> 全部为 0
q -> 请求退出
```

### 任务 3：实现安全限幅

产出：

```text
safety_limiter.py
test_safety_limiter.py
```

验收：

```text
x/y 速度不超过配置值
z 永远为 0
所有旋转速度永远为 0
超时后自动归零
退出前自动归零
```

### 任务 4：实现键盘节点

产出：

```text
keyboard_servo_node.py
```

验收：

```bash
ros2 run ur3e_keyboard_servo_py keyboard_servo_node
```

能在不启动机器人时打印当前状态；按键后生成对应速度；退出前发布零速度。

### 任务 5：仿真启动文件

产出：

```text
sim_keyboard_servo.launch.py
sim_keyboard_servo.yaml
```

验收：

```bash
ros2 launch ur3e_keyboard_servo_py sim_keyboard_servo.launch.py \
  use_mock_hardware:=true \
  launch_rviz:=true
```

假硬件中方向正确。

状态：

```text
已完成 fake hardware smoke test：启动链路、RViz 启动、Servo TWIST mode 握手、键盘输入进入节点、RViz 机械臂响应运动均已通过。
仍可继续补充 ↑ ↓ ← → 四方向的逐项运动方向记录。
```

### 任务 6：URSim 验收

产出：

```text
notes/sim_ursim_validation.md
```

验收：

```text
URSim 中 x/y 方向正确
松键停止
空格停止
q 退出
无持续异常
```

状态：

```text
已完成 URSim smoke test：launch_rviz:=false，通过 URSim 网页监视器确认机械臂响应键盘 Servo 输入并运动。
仍可继续补充 URSim ↑ ↓ ← →、松键、空格、q 的完整验收矩阵。
```

### 任务 7：真机启动文件

产出：

```text
real_keyboard_servo.launch.py
real_keyboard_servo.yaml
```

验收：

```text
未传 human_confirmation 时拒绝真实运动
传入确认口令后才启动键盘控制
默认速度 0.005 m/s
会话 30 秒自动停止
```

状态：

```text
已实现 real_keyboard_servo.launch.py 和 real_keyboard_servo.yaml。
真机入口要求 human_confirmation:=I_CONFIRM_REAL_ROBOT_MOTION，默认速度 0.005 m/s，会话 30 秒自动停止。
首版并发启动 External Control manager 过早会导致 driver configure 时序风险；已改为先通过 hardware ready gate，再启动 dashboard / External Control。
```

### 任务 8：真机低速验证

产出：

```text
notes/real_robot_validation.md
```

验收：

```text
↑ ↓ ← → 四方向短按通过
松键停止通过
空格停止通过
q 退出通过
无 protective stop
无异常恢复动作
```

## 11. 建议最终命令

创建工作区：

```bash
cd ~/ros2_lab/workspaces
mkdir -p ws_realtime_control/src
cd ws_realtime_control
```

构建：

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_lab/workspaces/ws_stage4/install/setup.bash
source ~/ros2_lab/workspaces/ws_ur3e_controller/install/setup.bash

colcon build
source install/setup.bash
```

仿真运行：

```bash
ros2 launch ur3e_keyboard_servo_py sim_keyboard_servo.launch.py \
  use_mock_hardware:=true \
  launch_rviz:=true
```

真机前检查：

```bash
ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
  require_trajectory_controller_active:=true
```

真机运行：

```bash
ros2 launch ur3e_keyboard_servo_py real_keyboard_servo.launch.py \
  human_confirmation:=I_CONFIRM_REAL_ROBOT_MOTION
```

## 12. 最终建议

新方案可以概括为：

```text
新建 ws_realtime_control
只放实时控制相关代码
第一版只做键盘 x/y 伺服控制
仿真先过 fake hardware 和 URSim
真机必须先过 Task 8C 门闩
真机默认不动，必须人工确认
速度极低，松键即停，退出即停
```

这个方案比把代码塞进 `ws_ur3e_controller` 更清晰，也更符合原仓库“分阶段、可验收、真机安全优先”的风格。
