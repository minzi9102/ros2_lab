# Task 7E：MoveIt Servo 连续控制入门

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[x] 已完成 fake hardware 下 MoveIt Servo 最小闭环验收`

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
  - `start_motion` 手动触发入口；
  - 7E 实验记录模板；
  - fake hardware 下 2 轮、每轮 8 次手动触发验收。
- 本任务不继续扩展到：
  - URSim / 真机 Servo；
  - 遥操作输入设备；
  - 视觉伺服；
  - 多控制模式切换策略。

## 4. 练习 1：构建 7E C++ 包

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_servo_lab_cpp
source install/setup.bash
```

### 结果记录
- 是否构建通过：通过。
- 构建命令：
  ```bash
  cd /home/minzi/ros2_lab/workspaces/ws_stage3
  source /opt/ros/jazzy/setup.bash
  colcon build --packages-select ur3_moveit_servo_lab_cpp
  source install/setup.bash
  ```
- 结果：`1 package finished`，未出现 `ServoStatus` 或 `TwistStamped` 依赖错误。

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
- `Trigger` 服务手动开始一轮 `TwistStamped` 周期发送；
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

### 参数结论
- `linear_x=0.02`：在 fake hardware + RViz 下可观察到 Servo 命令链路响应，同时保持足够保守。
- `run_duration_sec=2.5`：单次运动窗口足够观察启动、持续发布和停止收尾，不会把一次练习拉得过长。
- `halt_publish_count=5`：每轮结束后能稳定发布零速度 halt，并让 commander 回到 idle。
- 当前未启用角速度命令，避免把 7E 的首轮目标扩大到姿态伺服。

### 学习结论
- Servo 是连续速度控制：它持续消费当前 `TwistStamped` 命令并即时修正运动，而不是先生成一整条离线轨迹再交给控制器执行。
- `forward_position_controller` 适合本任务的最小闭环，因为 7E 要观察 Servo 到前向位置控制器的直接命令链路，而不是复用 `scaled_joint_trajectory_controller` 的轨迹执行语义。
- halt 零速度消息用于明确结束一次短时运动，让“停止发布非零速度”和“发送停止意图”在日志与控制链路里都可观察。

## 6. 练习 3：运行记录

### 执行命令
```bash
ros2 run ur3_moveit_servo_lab_cpp run_task7e_full_test.py \
  --workspace-root . \
  --max-attempts 1 \
  --timeout-sec 60 \
  --trigger-count 8 \
  --use-mock-hardware true \
  --launch-rviz true
```

### 手动开始一轮速度发布
```bash
ros2 service call /ur3_servo_twist_commander/start_motion std_srvs/srv/Trigger "{}"
```

### 观察点
- launch 完成后，`commander` 会先进入待机，不会自动发非零速度。
- 每次调用一次 `start_motion`，会按当前参数跑完一轮，再自动停下并回到待机。
- 你可以重复调用同一个服务，对比不同速度参数下的响应。

### 记录模板
- Servo 链路是否启动：是，两轮验收均出现 `Servo initialized successfully`。
- 是否收到 `/servo_node/status`：是，两轮验收均收到 `code=0 message='No warnings'`。
- `commander` 是否进入 idle ready：是，每次运动结束后都回到 idle。
- 短时移动是否可见：fake hardware + RViz 下可观察到短时 Servo 命令链路响应。
- 停止后是否能停下：是，每次触发都出现 `Motion duration elapsed` 后发布 halt，并出现 `Servo command sequence finished`。
- 你对当前速度上限与停止策略的评价：当前默认值在 fake hardware 下表现稳定，适合作为 7E 的保守入门参数；后续若进入 URSim 或真机，应重新校准速度、持续时间和 halt 策略。

### 验收记录
- 2026-04-27 10:47:53：`logs/task7E/20260427-104753`
  - `trigger 1/8` 到 `trigger 8/8` 均 accepted。
  - 8 次 `Servo command sequence finished. Commander returned to idle.`。
  - 结果：`status=PASS`。
- 2026-04-27 10:48:58：`logs/task7E/20260427-104858`
  - `trigger 1/8` 到 `trigger 8/8` 均 accepted。
  - 8 次 `Servo command sequence finished. Commander returned to idle.`。
  - 结果：`status=PASS`。
- 可接受提示：
  - `/recognize_objects not available`
  - `No 3D sensor plugin(s) defined`
  - realtime / FIFO scheduling warning

## 7. 完成记录
- 日期：2026-04-27
- 备注：Task 7E 已完成 fake hardware 下 MoveIt Servo 最小闭环；`launch-rviz=true` 下连续两轮、每轮 8 次手动触发均通过。URSim、真机、遥操作和视觉伺服留作后续独立任务。
