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

Servo 属于连续速度控制，是因为它实时根据当前状态和输入，持续生成下一时刻的速度命令；离线路径规划则是在执行前先算出整条轨迹，再交给执行链路跟随。

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

## 8. 调试与修改记录

### 8.1 从骨架到可触发命令
- `8295f12 feat(task7E): 新建MoveIt Servo练习包骨架`
  - 新建 `ur3_moveit_servo_lab_cpp`，建立 launch、CMake、package 和 `servo_twist_commander_node.cpp`。
  - 目标是先跑通最小 Servo 链路，而不是一次性加入遥操作或真机策略。
- `ca0f66f fix(task7E): 补上 Servo 启动命令类型握手`
  - commander 启动后先调用 `/servo_node/switch_command_type`，显式切到 TWIST。
  - 避免 `TwistStamped` 已经发布，但 Servo 仍处在其他命令模式。
- `4ba48bb fix(task7E): 为 Servo 命令增加 ready 状态门闩`
  - commander 不再一启动就发非零速度，而是等待 Servo status 后再允许进入可运行状态。
  - 这一步把“启动流程”和“运动命令”拆开，日志更容易判断失败点。
- `3f3a532 feat(task7E): 手动触发 servo commander`
  - 新增 `/ur3_servo_twist_commander/start_motion` 服务。
  - 每次触发只执行一轮短时运动，结束后发布 halt 并回到 idle，便于重复试验。

### 8.2 启动顺序与可观测性
- `4130556 fix(task7E): 独立启动 Servo 节点便于排障`
  - 不直接依赖顶层 MoveIt launch 的 Servo 开关，而是在 7E wrapper 中独立启动 `servo_node`。
  - 这样可以单独观察 Servo 参数、status 和 commander 的启动时序。
- `233b920 fix(task7E): 暴露 commander frame_id 启动参数`
  - 将 `frame_id` 暴露为 launch 参数，便于比较 `tool0` 等参考系对 Twist 语义的影响。
- `006c618 fix(task7E): 为每次 Servo 测试生成独立日志目录`
  - 每次运行生成 `logs/task7E/<timestamp>`，后续排障能直接对照单次实验日志。
- `5e60970 fix(task7E): 暴露线速度轴向测试参数`
  - 将 `linear_x/y/z` 暴露为 launch 参数，保留默认保守速度，同时支持单轴对比。
- `8aa4855 docs(task7E): 补充 servo 教学注释`
  - 给 launch 和 commander 增加教学注释，说明为什么要分阶段启动、为什么默认用 `tool0`。

### 8.3 解决启动不稳定
- `82d89d4 fix(task7E): 将初始姿态切换为 test_configuration`
  - 增加 7E 专用 URDF wrapper 和初始关节位置文件，减少初始状态对 Servo 启动的干扰。
- `8940a0f feat(task7e): delay servo startup until joint states arrive`
  - 新增 `/joint_states` gate，底层状态流和控制器 ready 后才启动 Servo。
  - 失败时能明确知道问题卡在 controller/joint state 阶段，而不是误判为 commander 问题。
- `db23f80 fix(task7E): stabilize servo current state monitor startup`
  - 强化 joint state gate，降低 MoveIt CurrentStateMonitor 启动窗口导致的偶发失败。
- `aff6c04 fix(task7E): gate commander on servo status`
  - 新增 `/servo_node/status` gate，只有看到 Servo status 后才启动 commander。
  - 这一步把启动链路收敛成：`/joint_states` ready -> Servo status ready -> commander ready。
- `ec147ed fix(task7E): add servo startup settle gate`
  - joint state gate 通过后增加 settle 时间，避开 MoveIt 和控制器刚启动时的抖动窗口。
- `dba5c50 fix(task7E): follow ur servo planning scene ownership`
  - 避免 7E wrapper 额外争用 planning scene 相关职责，减少与 `ur_moveit_config` 默认链路的冲突。

### 8.4 验收脚本与静止态问题
- `8294234 test(task7E): probe standalone servo startup path`
  - 尝试把问题拆到独立 Servo 启动路径中定位，而不是只看完整 launch 的混合日志。
- `4d8f212 fix(task7E): add servo-ready retry harness`
  - 新增 `run_task7e_full_test.py`，自动启动 launch、等待服务、触发 motion、检查完成标记。
  - 这让“是否稳定”从人工观察变成可重复的 PASS/FAIL 判断。
- `ac1bf2a fix(task7E): 稳定静止态 servo 启动`
  - 将 Python relay 升级为 C++ `joint_state_stamp_relay_node`，持续发布 fresh-stamped joint states。
  - 背景是 mock robot 静止时关节数值不变，MoveIt 等待“新状态”时可能迟迟不醒。
- `d3f6a00 fix(task7E): 清理 servo 测试进程组`
  - harness 结束时清理 launch 进程组，避免残留 ROS 进程污染下一轮测试。
- `7757a5e docs(task7E): 收口 servo fake hardware 验收`
  - 记录两轮 `trigger-count=8` 的最终验收结果，并明确 URSim/真机 Servo 留作后续任务。

## 9. 经验总结
- Servo 调试不要一开始就盯着 Twist 数值。更可靠的顺序是先确认底层状态和控制器，再确认 Servo status，再启动 commander，最后才触发非零速度。
- 对 MoveIt Servo 来说，“能收到 `/joint_states`”不等于 CurrentStateMonitor 一定会及时满足等待条件。mock hardware 静止时，fresh stamp relay 可以把静止状态也变成稳定的可观测输入。
- commander 默认不自动运动是更安全的实验形态。让它先进入 idle ready，再通过 `Trigger` 服务手动开始一轮短时运动，能避免 launch 一成功就发非零速度。
- 每轮运动结束后发布若干次零速度 halt，比单纯停止发布命令更清楚：日志里能看到主动停机意图，控制链路也有明确的收尾消息。
- 独立日志目录和 harness 很重要。7E 后期的判断不是靠“这次看起来没报错”，而是靠两个目录中重复出现的 `Servo initialized successfully`、`Received ServoStatus`、`Accepted start_motion` 和 `Servo command sequence finished`。
- fake hardware 下可接受 `/recognize_objects not available`、`No 3D sensor plugin(s) defined` 和 realtime/FIFO warning；这些不是 7E 最小 Servo 闭环的失败信号。
