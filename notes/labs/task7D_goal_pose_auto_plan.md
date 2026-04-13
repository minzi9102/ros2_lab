# Task 7D：点击目标位姿 -> 自动规划执行 Demo

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[#] 已创建 /goal_pose 自动规划骨架，映射策略为保守默认值`

## 1. 目标
- 在 `workspaces/ws_stage3/src/ur3_moveit_goal_pose_lab_cpp` 中实现一个“点击 -> 自动规划”的最小 demo。
- 输入来自 RViz 的 `/goal_pose`。
- 默认映射策略：
  - 保留 `x / y / yaw`
  - 固定 `z`
  - 固定末端朝下
  - 超出工作区直接拒绝

## 2. 应用包位置
- 包路径：`/home/minzi/ros2_lab/workspaces/ws_stage3/src/ur3_moveit_goal_pose_lab_cpp`
- 关键文件：
  - `src/goal_pose_executor_node.cpp`
  - `launch/task7D_goal_pose_auto_plan.launch.py`

## 3. 当前准备情况
- 已完成：
  - `/goal_pose` 订阅骨架；
  - 工作区范围检查；
  - `x / y / yaw -> 目标抓取位姿` 的默认映射；
  - 自动 `plan()` 与可选 `execute()`；
  - 7D 实验记录模板。
- 待你完成：
  - 判断当前工作区边界是否合理；
  - 判断 `target_height` 是否合适；
  - 判断“末端朝下 + 保留 yaw”的姿态语义是否真的适合你的抓取理解；
  - 在 fake hardware / URSim 下完成点击演示。

## 4. 练习 1：构建 7D C++ 包

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_goal_pose_lab_cpp
source install/setup.bash
```

### 结果记录
- 是否构建通过：
- 若失败，错误集中在哪个 tf2 / MoveIt 头文件：

## 5. 练习 2：检查默认映射策略

### 背景
这里最值得你思考的，不是“怎么订阅一个 PoseStamped”，而是“二维点击输入为什么不能直接等于末端最终位姿”。你需要亲自判断：桌面任务的语义应该如何映射成机器人末端姿态。

### 位置
- 文件：`workspaces/ws_stage3/src/ur3_moveit_goal_pose_lab_cpp/src/goal_pose_executor_node.cpp`
- 重点参数：
  - `target_height`
  - `workspace_min_x`
  - `workspace_max_x`
  - `workspace_min_y`
  - `workspace_max_y`
  - `downward_roll_rad`
  - `downward_pitch_rad`

### 已准备内容
- `/goal_pose` 订阅；
- 工作区范围判断；
- yaw 提取；
- 目标姿态构造；
- 自动规划与可选执行。

### 你需要完成的内容
- 审查并修改工作区边界。
- 审查并修改安全高度。
- 判断是否保留输入 yaw。
- 判断“朝下抓取”的固定姿态是否足够合理。

### 权衡点
- 工作区太小会误拒绝可用点位。
- 工作区太大又会把明显不可达点送进规划器。
- 固定高度太低会更容易碰桌，太高又会偏离抓取语义。
- 保留 yaw 很自然，但未必总是与你希望的末端朝向一致。

### 完成标准
- 你能解释当前映射策略背后的理由。
- 你能说清哪些目标应该在规划前就拒绝。
- 你能完成至少一次点击后自动规划。

### 恢复方式
- 你调整完参数或逻辑后告诉我“请 review task7D 实现”。
- 我会继续帮你：
  - 检查边界条件；
  - review 可达性与姿态逻辑；
  - 修复构建 / 运行问题。

## 6. 练习 3：运行记录

### 执行命令
```bash
ros2 launch ur3_moveit_goal_pose_lab_cpp task7D_goal_pose_auto_plan.launch.py \
  execute_plan:=true
```

### 记录模板
- RViz 点击后是否收到 `/goal_pose`：
- 工作区内目标是否成功规划：
- 工作区外目标是否被直接拒绝：
- 你对默认映射策略的评价：
- fake hardware / URSim 下的差异：

## 7. 完成记录
- 日期：
- 备注：
