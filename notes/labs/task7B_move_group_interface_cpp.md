# Task 7B：C++ MoveGroupInterface 最小规划节点

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[x] 7B 主路径已完成：joint goal / pose goal / plan_only / plan_and_execute 已验证，成功路径可自动退出，ros2 launch 主路径下 kinematics warning 已收敛；ros2 run warning 作为已知边界保留`

## 1. 目标
- 在 `workspaces/ws_stage3/src/ur3_moveit_move_group_lab_cpp` 中补完一个最小 MoveGroupInterface 节点。
- 支持：
  - `joint goal`
  - `pose goal`
  - `plan_only`
  - `plan_and_execute`

## 2. 应用包位置
- 包路径：`/home/minzi/ros2_lab/workspaces/ws_stage3/src/ur3_moveit_move_group_lab_cpp`
- 关键文件：
  - `src/move_group_planner_node.cpp`
  - `launch/task7B_move_group_interface.launch.py`

## 3. 当前准备情况
- 已完成：
  - C++ 包骨架；
  - 参数声明；
  - `MoveGroupInterface` 初始化外壳；
  - `joint target` / `pose target` 参数校验；
  - `setJointValueTarget(joint_target_)`；
  - `setPoseTarget(pose, end_effector_link_)`；
  - `plan()` 成功 / 失败分支；
  - `execute_plan_` 为 `true` 时的执行逻辑骨架；
  - `pose goal + plan_only` 联调；
  - 不可达 pose 的失败现象记录；
  - `execute_plan:=true` 执行链路验证；
  - one-shot 成功路径自动退出验证；
  - `task7B_move_group_interface.launch.py` 已注入 `robot_description_kinematics`；
  - 7B 实验记录模板。
- 已知边界：
  - 若希望 `ros2 run` 路径也消除本地 warning，仍需补参数文件或 wrapper launch；

## 4. 练习 1：构建 7B C++ 包

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_move_group_lab_cpp
source install/setup.bash
```

### 结果记录
- 是否构建通过：是
- 若失败，错误集中在哪个 MoveIt 头文件或依赖：本轮已构建通过；此前主要踩过 `MoveItErrorCode` 命名空间和 `getJointValueTarget()` / `getPoseTarget()` API 使用方式。

## 5. 练习 2：补完关键规划逻辑

### 背景
这段代码要解决的问题，是把“阶段 3 已跑通的 MoveIt bringup”升级成“能从程序里发出 joint goal / pose goal 的最小规划请求”。

### 位置
- 文件：`workspaces/ws_stage3/src/ur3_moveit_move_group_lab_cpp/src/move_group_planner_node.cpp`
- 函数：
  - `apply_joint_target()`
  - `apply_pose_target()`
  - `run_plan_flow()`

### 已准备内容
- 已有参数：
  - `planning_group`
  - `pose_reference_frame`
  - `end_effector_link`
  - `target_mode`
  - `execute_plan`
- 已完成：
  - `MoveGroupInterface` 创建与通用配置；
  - joint / pose 参数校验；
  - pose 目标消息构造；
  - 日志与 shutdown 外壳。

### 你需要完成的内容
- 在 `joint` 模式下设置 joint target。
- 在 `pose` 模式下设置 pose target。
- 创建 `MoveGroupInterface::Plan`，调用 `plan()`。
- 只在规划成功时，根据 `execute_plan_` 决定是否执行。

### 权衡点
- `joint_target_` 的顺序是否与你理解的 `ur_manipulator` 关节顺序一致。
- pose target 的位置和姿态是否真的可达。
- `plan()` 失败时，是直接退出还是打印更多上下文。
- `execute_plan_` 为 `false` 时，日志如何说明“规划成功但未执行”。

### 完成标准
- 节点能在 `joint` 模式下完成一次规划。
- 节点能在 `pose` 模式下完成一次规划。
- 不可达 pose 不会触发执行。

### 恢复方式
- 你补完后告诉我“请 review task7B 实现”。
- 我会继续负责：
  - review 你的逻辑；
  - 修正编译 / 类型 / include 问题；
  - 补充必要测试与联调命令。

## 6. 练习 3：验证规划结果

### 执行命令
```bash
# 前置：先启动 7A bringup
ros2 launch ur3_moveit_bringup_lab task7A_moveit_quickstart.launch.py \
  use_mock_hardware:=true \
  launch_rviz:=false

# round 1: pose + plan_only（默认 pose）
ros2 launch ur3_moveit_move_group_lab_cpp task7B_move_group_interface.launch.py \
  target_mode:=pose \
  execute_plan:=false

# round 2: 不可达 pose + plan_only
ros2 run ur3_moveit_move_group_lab_cpp move_group_planner_node --ros-args \
  -p target_mode:=pose \
  -p execute_plan:=false \
  -p pose_position:="[1.2, 0.0, 0.8]"

# round 3: pose + plan_and_execute（默认 pose）
ros2 run ur3_moveit_move_group_lab_cpp move_group_planner_node --ros-args \
  -p target_mode:=pose \
  -p execute_plan:=true

# 已完成的 joint + plan_only 最小闭环
ros2 launch ur3_moveit_move_group_lab_cpp task7B_move_group_interface.launch.py \
  target_mode:=joint \
  execute_plan:=false
```

### 记录模板
- `joint goal` 是否规划成功：是；在先启动 `7A` bringup 后，`Planning request accepted -> complete -> Planning succeeded.` 日志已出现
- `pose goal` 是否规划成功：是；默认 pose（`position=[0.25, -0.20, 0.35]`，`orientation_xyzw=[0.7071, 0.0, 0.0, 0.7071]`）在当前 fake hardware + `7A` bringup 前提下可成功 `plan()`
- `plan_only` 与 `plan_and_execute` 的日志区别：`plan_only` 停在 `Planning succeeded.` 后立即打印 `Shutting down node: Plan flow completed successfully`；`plan_and_execute` 会继续出现 `Execute request accepted -> Execute request success! -> Execution succeeded.`，随后同样主动退出
- 不可达 pose 时的失败现象：`setPoseTarget()` 仍然成功，但 `plan()` 阶段出现 `Planning request aborted` / `MoveGroupInterface::plan() failed or timeout reached`，节点随后打印 `Planning failed.`
- `ros2 launch` 主路径下 `No kinematics plugins defined` warning 是否消失：是；已看到本地日志从 warning 变为 `moveit.kinematics.kdl_kinematics_plugin` 正常加载
- `ros2 run` 直跑时 `No kinematics plugins defined` warning 是否仍会出现：是；因为会绕过 `task7B_move_group_interface.launch.py` 的参数注入

### 当前调试结论
- 不能单独直接启动 `7B` 节点做联调；若未先启动 `7A` bringup，`MoveGroupInterface` 会因缺少 `robot_description` 无法构造 robot model。
- `robot_description` 由 `ur_robot_driver` 的 `ur_rsp.launch.py` 生成并传给 `robot_state_publisher`；运行时既可作为 `/robot_state_publisher` 的参数读取，也会以 `std_msgs/msg/String` 的 `/robot_description` 话题发布。
- `7A` 拉起的 `/move_group` 本身已经拥有 `robot_description_kinematics.ur_manipulator.*` 参数，因此官方 MoveIt 主链路并没有缺 kinematics 配置。
- `7B` 自定义节点创建 `MoveGroupInterface` 时，也会在本地节点里创建 `RobotModelLoader`；它能通过 `/robot_description` 和 `/robot_description_semantic` 的同名话题兜底拿到 URDF / SRDF，但 `robot_description_kinematics` 没有同样的话题兜底。
- 之前 `task7B_move_group_interface.launch.py` 只注入了实验参数，没有把 `robot_description_kinematics` 传给 `ur3_move_group_planner`，所以 warning 出现在 7B 客户端本地，而不是 `/move_group` 服务器侧。
- 当前已按最小方案在 `task7B_move_group_interface.launch.py` 注入 `ur_moveit_config/config/kinematics.yaml`；因此 `ros2 launch ur3_moveit_move_group_lab_cpp ...` 主路径下，本地 `MoveGroupInterface` 应不再缺 kinematics 参数。
- 已实测确认：若直接使用 `ros2 run ur3_moveit_move_group_lab_cpp move_group_planner_node ...`，会绕过 launch 注入，因此仍会看到同样的本地 warning。
- 在 `ros2 launch` 主路径下，kdl kinematics plugin 加载成功后，节点退出时会出现一次 `class_loader.ClassLoader: SEVERE WARNING!!!`；当前进程仍然 clean exit，这更像是插件对象释放时机问题，而不是 7B 规划主链路失败。
- 在 `7A` bringup 已启动的前提下，当前代码已经完成：
  - 一次 `joint target + plan_only`
  - 一次默认 `pose target + plan_only`
  - 一次默认 `pose target + plan_and_execute`
- 默认 `pose target + plan_only` 和 `pose target + plan_and_execute` 成功后，节点都会打印 `Shutting down node: Plan flow completed successfully` 并干净结束进程。
- 对明显不可达的 pose，失败点出现在 `plan()`，而不是 `setPoseTarget()`。

## 7. 当前进度记录
- 日期：2026.4.15
- 备注：已完成三轮最小验证并补齐记录：默认 `pose goal` 可规划、不可达 pose 会在 `plan()` 阶段失败、`execute_plan:=true` 可成功走完整执行链路，且成功路径现已自动退出。本轮已定位 kinematics warning 根因，并实测确认 `ros2 launch` 主路径下注入 `robot_description_kinematics` 后 warning 消失；`ros2 run` 仍会绕过该注入，若也要收敛 warning，后续需补参数文件或 wrapper launch。
