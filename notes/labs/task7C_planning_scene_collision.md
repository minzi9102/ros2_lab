# Task 7C：Planning Scene、Collision Object 与 Cartesian Path

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[x] 已在 fake hardware 与 URSim 下完成桌面碰撞体、pose planning、Cartesian Path 与执行链路验证`

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
  - 7C 实验记录模板；
  - fake hardware 下参数收敛；
  - URSim 下一次真实执行冒烟。
- 待你完成：
  - 无，本任务已完成主路径验收。

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

### URSim 冒烟记录
- 运行日期：2026.4.16
- 运行目标：在 URSim 下验证 7C 的执行链路，而不仅是 fake hardware 下的规划结果。
- URSim 启动方式：通过 `docker --context default run ... universalrobots/ursim_e-series:latest` 启动。
- 关键前置条件：
  - 需要切到 `docker context default`，因为 `desktop-linux` 与 `default` 是两套独立 daemon，URSim 镜像只存在于 `default`。
  - 需要在 URSim 中安装并运行 `External Control URCap`，否则 RViz 只能 `Plan`，不能 `Execute`。
  - `External Control` 里远程主机地址需要填当前容器可见的宿主机网关，而不是想当然沿用旧桥接网段。
- 关键排障过程：
  - 初始现象：`plan` 成功但 `execute` 失败，`scaled_joint_trajectory_controller` 为 `inactive`，`/speed_scaling_state_broadcaster/speed_scaling=0.0`。
  - 根因 1：URSim 镜像位于 `default` context，不在 `desktop-linux` 中；切换回 `default` 后恢复可运行镜像。
  - 根因 2：URSim 初始没有 `External Control`，需挂载并安装 `externalcontrol-1.0.5.urcap`。
  - 根因 3：`External Control` 最初连接到 `192.168.56.1:50002`，但当前容器实际在 `172.17.0.0/16` bridge 上，正确宿主机地址应为 `172.17.0.1:50002`。
  - 恢复信号：`speed_scaling` 恢复为 `100.0`，`scaled_joint_trajectory_controller` 变为 `active`。
- 冒烟结果：
  - URSim 下 `External Control` 成功运行；
  - 7C 在 URSim 下可正常 `execute`；
  - 机械臂按当前参数完成了一次真实轨迹执行。
- fake hardware / URSim 下的差异：
  - fake hardware 下只要规划参数合理，通常不需要额外关注外部控制程序。
  - URSim 下除了规划成功，还必须保证 `External Control`、reverse interface、控制器激活状态和容器网络地址全部正确。
- 本轮最终结论：
  - 7C 已完成 fake hardware 与 URSim 两条主路径，满足“程序添加桌面碰撞体 + 返回并解释 Cartesian fraction + 至少一次 URSim 安全冒烟执行”的验收目标。

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
- 最终观察到的关键现象：fake hardware 与 URSim 下都能成功按指定计划执行；失败轮次主要由桌面碰撞体设置过于激进或外部控制链路未就绪引起。
- 我对 7C 的一句话总结：学习如何在 Planning Scene 中添加碰撞体，并在 fake hardware 与 URSim 两种执行环境下分别验证 pose 与 Cartesian 轨迹规划、执行和排障路径。
- 备注：URSim 冒烟额外暴露了 Docker context、URCap 安装、容器网络地址与 `External Control` 配置之间的联动关系。
