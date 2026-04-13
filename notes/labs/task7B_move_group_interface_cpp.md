# Task 7B：C++ MoveGroupInterface 最小规划节点

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[#] 已创建 C++ 包骨架，关键规划调用保留为 TODO(human)`

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
  - 7B 实验记录模板。
- 待你完成：
  - `setJointValueTarget(joint_target_)`
  - `setPoseTarget(pose, end_effector_link_)`
  - `plan()` 的成功 / 失败分支
  - `execute_plan_` 为 `true` 时的执行逻辑

## 4. 练习 1：构建 7B C++ 包

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_move_group_lab_cpp
source install/setup.bash
```

### 结果记录
- 是否构建通过：
- 若失败，错误集中在哪个 MoveIt 头文件或依赖：

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
ros2 launch ur3_moveit_move_group_lab_cpp task7B_move_group_interface.launch.py \
  target_mode:=joint \
  execute_plan:=false
```

### 记录模板
- `joint goal` 是否规划成功：
- `pose goal` 是否规划成功：
- `plan_only` 与 `plan_and_execute` 的日志区别：
- 不可达 pose 时的失败现象：

## 7. 完成记录
- 日期：
- 备注：
