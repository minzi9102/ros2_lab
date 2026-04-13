# Task 7E：MoveIt Servo 连续控制入门

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[#] 已创建 Servo bringup 与 Twist 命令骨架，速度参数待你亲自校准`

## 1. 目标
- 在 `workspaces/ws_stage3/src/ur3_moveit_servo_lab_cpp` 中跑通最小 MoveIt Servo 链路。
- 使用：
  - `/servo_node/delta_twist_cmds`
  - `/servo_node/status`
- 理解 Servo 为什么属于“连续速度控制”，而不是“离线路径规划”。

## 2. 应用包位置
- 包路径：`/home/minzi/ros2_lab/workspaces/ws_stage3/src/ur3_moveit_servo_lab_cpp`
- 关键文件：
  - `launch/task7E_moveit_servo.launch.py`
  - `src/servo_twist_commander_node.cpp`

## 3. 当前准备情况
- 已完成：
  - Servo wrapper launch；
  - `forward_position_controller` 对齐；
  - `TwistStamped` 周期发布节点；
  - `/servo_node/status` 订阅；
  - 7E 实验记录模板。
- 待你完成：
  - 判断线速度 / 角速度上限；
  - 判断运行持续时间；
  - 判断停止策略是否足够保守；
  - 在 fake hardware 下完成最小验收。

## 4. 练习 1：构建 7E C++ 包

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_servo_lab_cpp
source install/setup.bash
```

### 结果记录
- 是否构建通过：
- 若失败，错误集中在哪个 `ServoStatus` 或 `TwistStamped` 依赖：

## 5. 练习 2：审查速度与停机参数

### 背景
这里真正有学习价值的，不是“如何发一个 TwistStamped”，而是“连续速度控制应该多快、发多久、怎么安全停”。这些判断直接影响你以后做主从操作和人机协同的安全边界。

### 位置
- 文件：`workspaces/ws_stage3/src/ur3_moveit_servo_lab_cpp/src/servo_twist_commander_node.cpp`
- 重点参数：
  - `publish_rate_hz`
  - `run_duration_sec`
  - `halt_publish_count`
  - `linear_x / y / z`
  - `angular_x / y / z`

### 已准备内容
- Servo bringup wrapper；
- `forward_position_controller` 启动链路；
- `TwistStamped` 周期发送；
- 零速度 halt 消息；
- Servo status 日志。

### 你需要完成的内容
- 判断速度是不是过快。
- 判断运行时间是不是过长。
- 判断当前 halt 消息数量是否足以可靠停下。

### 权衡点
- 速度太小可能几乎看不到响应，太大又会让连续控制失去保守性。
- 运行时间太短不利于观察，太长会放大风险和误差。
- halt 消息太少可能停不稳，太多又会让实验日志冗长。

### 完成标准
- fake hardware 下能跑完一段短时 Servo 命令。
- 停止发命令后能看到机械臂停下。
- 你能解释为什么 Servo 不是 MoveGroup 的替代品。

### 恢复方式
- 你改完参数或命令策略后告诉我“请 review task7E 实现”。
- 我会继续帮你：
  - review 速度边界；
  - 检查 status 输出；
  - 修正构建与 launch 问题。

## 6. 练习 3：运行记录

### 执行命令
```bash
ros2 launch ur3_moveit_servo_lab_cpp task7E_moveit_servo.launch.py \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true
```

### 记录模板
- Servo 链路是否启动：
- 是否收到 `/servo_node/status`：
- 短时移动是否可见：
- 停止后是否能停下：
- 你对当前速度上限与停止策略的评价：

## 7. 完成记录
- 日期：
- 备注：
