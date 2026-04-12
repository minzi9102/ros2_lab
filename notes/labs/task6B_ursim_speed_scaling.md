# Task 6B：URSim speed scaling 实验记录

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[#] 已创建 6B 应用包骨架，待补完 TODO(human) 并进入 URSim 联调`

## 1. 目标
- 在 `workspaces/ws_stage2/src/ur3_ursim_speed_scaling_lab_py` 中完成一个独立的 6B 实验包。
- 验证 `/speed_scaling_state_broadcaster/speed_scaling` 在 URSim 下会随速度滑块变化。
- 用同一条保守轨迹比较 `100%` 与降速条件下的执行节奏差异。

## 2. 应用包位置
- 包路径：`/home/minzi/ros2_lab/workspaces/ws_stage2/src/ur3_ursim_speed_scaling_lab_py`
- 关键节点：
  - `speed_scaling_monitor`
  - `scaled_trajectory_runner`

## 3. 当前准备情况
- 已完成：
  - 新建 6B 独立 Python 应用包骨架；
  - 提供 speed scaling 观测节点；
  - 提供轨迹发送节点骨架、launch 文件与参数 YAML。
- 待你完成：
  - `scaled_trajectory_runner.py` 中 `plan_demo_points()` 的保守轨迹设计；
  - URSim 界面中的速度滑块操作与实验解释。

## 4. 练习 1：构建 6B 新包

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_ursim_speed_scaling_lab_py
source install/setup.bash
```

### 结果记录
- 是否构建通过：
- 如失败，报错关键信息：

## 5. 练习 2：只验证 speed scaling 观测链路

### 执行命令
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=<ursim_container_ip> \
  launch_rviz:=true

ros2 launch ur3_ursim_speed_scaling_lab_py task6B_ursim_speed_scaling.launch.py
```

### 观察记录
- `100%` 速度滑块时的日志：
- `50%` 速度滑块时的日志：
- 更低速度滑块时的日志：
- 我观察到的变化：

## 6. 练习 3：补写保守轨迹并做执行对比

### 位置
- 文件：`workspaces/ws_stage2/src/ur3_ursim_speed_scaling_lab_py/ur3_ursim_speed_scaling_lab_py/scaled_trajectory_runner.py`
- 函数：`plan_demo_points()`

### TODO(human) 完成后记录
- 我选择移动哪些关节：
- 每个关节变化幅度大约多少：
- 为什么认为这条轨迹足够保守：
- `100%` speed scaling 下的执行结果：
- `50%` speed scaling 下的执行结果：
- 两次耗时差异：

## 7. 解释闭环
- 为什么这次可以算“真实 speed scaling 实证”：
- 为什么 result 成功不等于“速度滑块没生效”：
- 如果换成 `joint_trajectory_controller`，我预计会有什么差异：
- 这次实验对未来手术机器人控制链路设计的启发：

## 8. 完成记录
- 日期：
- 备注：
