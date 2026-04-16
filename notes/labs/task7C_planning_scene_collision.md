# Task 7C：Planning Scene、Collision Object 与 Cartesian Path

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[#] 已创建 C++ 包骨架，桌面与 Cartesian API 已接通，关键参数已经调整，了解了planning_scene的作用以及pose planning 与 Cartesian Path的区别`

## 1. 目标
- 在 `workspaces/ws_stage3/src/ur3_moveit_scene_lab_cpp` 中理解：
  - `Planning Scene`
  - `CollisionObject`
  - `computeCartesianPath`
- 让 UR3 经过 `3` 个抓取点位，并比较 pose planning 与 Cartesian Path。

## 2. 应用包位置
- 包路径：`/home/minzi/ros2_lab/workspaces/ws_stage3/src/ur3_moveit_scene_lab_cpp`
- 关键文件：
  - `src/scene_cartesian_demo_node.cpp`
  - `launch/task7C_scene_cartesian.launch.py`

## 3. 当前准备情况
- 已完成：
  - C++ 包骨架；
  - `PlanningSceneInterface` 胶水；
  - 桌面 `CollisionObject` 增删；
  - pose planning 入口；
  - `computeCartesianPath` 入口；
  - 7C 实验记录模板。
- 待你完成：
  - 判断桌面尺寸与位置；
  - 重新设计 `3` 个抓取点位；
  - 决定 `min_cartesian_fraction` 的阈值；
  - 在 fake hardware / URSim 下完成解释闭环。

## 4. 练习 1：构建 7C C++ 包

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_scene_lab_cpp
source install/setup.bash
```

### 结果记录
- 是否构建通过：通过
- 若失败，错误集中在哪个 MoveIt / moveit_msgs 头文件：无

## 5. 练习 2：判断桌面和抓取点参数

### 背景
这段代码不是单纯“把消息填满”，而是要把你的场景语义放进 Planning Scene 里。真正值得你亲手决定的，是桌面在哪、点位在哪、以及多低的 Cartesian `fraction` 应该判失败。

### 位置
- 文件：`workspaces/ws_stage3/src/ur3_moveit_scene_lab_cpp/src/scene_cartesian_demo_node.cpp`
- 重点参数：
  - `table_dimensions`
  - `table_position`
  - `waypoint_a_position`
  - `waypoint_b_position`
  - `waypoint_c_position`
  - `common_orientation_xyzw`
  - `min_cartesian_fraction`

### 已准备内容
- `CollisionObject` 的 ADD / REMOVE 外壳；
- 默认桌面消息构造；
- pose planning 调用；
- `computeCartesianPath` 调用与日志。

### 你需要完成的内容
- 重新确认桌面的尺寸和位置：已调整
- 设计 `3` 个更符合你理解的抓取点位。
- 决定多低的 `fraction` 应该被视为“不足以接受”。

### 权衡点
- 桌面是否太高，导致本来可达的目标被误判碰撞。
- 抓取点位是否离桌面太近，导致姿态与碰撞同时变难解释。
- `fraction` 偏低时，是场景不合理、步长太大，还是路径本身不适合做 Cartesian。

### 完成标准
- 你能解释桌面参数是怎么选的。
- 你能解释为什么选择这 `3` 个点。
- 你能解释当前 `fraction` 阈值背后的工程理由。

### 参数填写区
- 当前选择的桌面尺寸 `table_dimensions`：0.35, 0.45, 0.1
- 当前选择的桌面位置 `table_position`：0.50, 0.0, -0.03
- 当前选择的 `waypoint_a_position`：0.29, 0.11, 0.41
- 当前选择的 `waypoint_b_position`：0.4, 0.11, 0.3
- 当前选择的 `waypoint_c_position`：0.4, 0.11, 0.22
- 当前选择的 `common_orientation_xyzw`：-0.70691, 0.70714, -0.014692, 0.0042299
- 当前选择的 `min_cartesian_fraction`：0.95
- 选择这些参数的原因：该方案可以在不碰撞到桌面，不超过机械臂活动范围的情况下成功完成全流程的轨迹规划和执行运动

### 恢复方式
- 你补完参数后告诉我“请 review task7C 参数与结果”。
- 我会继续帮你：
  - review 场景设置；
  - 检查碰撞与可达性；
  - 调整类型、构建与运行细节。

## 6. 练习 3：记录运行结果

### 执行命令
```bash
ros2 launch ur3_moveit_scene_lab_cpp task7C_scene_cartesian.launch.py \
  demo_mode:=both \
  execute_plan:=true
```

### 成功轮次记录
- 运行日期：2026.4.16
- 运行模式：both
- 是否使用 `execute_plan=true`：是
- 本轮使用的桌面参数：
  - `table_dimensions`：0.35, 0.45, 0.1
  - `table_position`：0.50, 0.0, -0.03
- 本轮使用的路径点参数：
  - `waypoint_a_position`：0.29, 0.11, 0.41
  - `waypoint_b_position`：0.4, 0.11, 0.3
  - `waypoint_c_position`：0.4, 0.11, 0.22
  - `common_orientation_xyzw`：-0.70691, 0.70714, -0.014692, 0.0042299
- 本轮使用的阈值参数：
  - `min_cartesian_fraction`：0.95
  - `eef_step`：0.010
  - `jump_threshold`：0.000
- 桌面碰撞体是否成功加入：是
- pose planning 是否成功：是
- Cartesian `fraction`：1.000
- 你对 `fraction` 的解释：从当前真实起点出发，整条请求的 Cartesian 路径实际生成出来的比例
- Cartesian 是否达到你设定的接受阈值：能
- fake hardware / URSim 下的差异：未比较
- 本轮最终结论：成功

### 失败轮次总结
- 是否观察到失败轮次：是
- 观察到的失败类型：
  - pose planning 失败
  - Cartesian `fraction` 低于阈值
- 低于桌面的 pose target 是否被拒绝或规划失败：是
- 失败时的主要原因：桌面碰撞体的尺寸或位置设置过于激进时，目标位姿或路径点会与桌面发生碰撞；某些尝试也会逼近机械臂活动范围边界。
- 失败现象与调整方向：
  - 桌面过高、过大或过于靠近路径时，plan 可能失败。
  - waypoint 过近桌面时，Cartesian `fraction` 会下降。
  - 调整桌面的形状和位置后，pose 与 Cartesian 都可以恢复到可接受结果。
- 失败轮次的反例参数是否已单独归档：否，当前只保留原因总结，后续如需复现实验可再补具体反例参数。
- 下一轮准备调整的内容：在 URSim 中测试

## 7. 完成记录
- 日期：2026.4.16
- 最终采用的桌面参数：
  - `table_dimensions`：0.35, 0.45, 0.1
  - `table_position`：0.50, 0.0, -0.03
- 最终采用的路径点参数：
  - `waypoint_a_position`：0.29, 0.11, 0.41
  - `waypoint_b_position`：0.4, 0.11, 0.3
  - `waypoint_c_position`：0.4, 0.11, 0.22
  - `common_orientation_xyzw`：-0.70691, 0.70714, -0.014692, 0.0042299
- 最终采用的 `fraction` 阈值：0.95
- 最终观察到的关键现象：成功按指定计划执行
- 我对 7C 的一句话总结：学习如何在planning scene中添加碰撞体，并能按pose和cartesian两种模式规划轨迹
- 备注：无
