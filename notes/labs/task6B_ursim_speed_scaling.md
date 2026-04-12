# Task 6B：URSim speed scaling 实验记录

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[x] 已完成 URSim speed scaling 实证与 100% / 50% 轨迹对比`

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
  - 提供轨迹发送节点骨架、launch 文件与参数 YAML；
  - 已在 `scaled_trajectory_runner.py` 中补完保守轨迹；
  - 已完成 URSim 下的 speed scaling 观测与轨迹对比实验。
- 待你完成：
  - 将本轮观察转写成自己的解释；
  - 若需要，可继续做更低速度档位的扩展复验。

## 4. 练习 1：构建 6B 新包

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_ursim_speed_scaling_lab_py
source install/setup.bash
```

### 结果记录
- 是否构建通过：是
- 如失败，报错关键信息：无

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
  - `Update: speed_scaling=100.0% fraction=1.000`
- `50%` 速度滑块时的日志：
  - `Update: speed_scaling=50.0% fraction=0.500`
- 更低速度滑块时的日志：
  - 本轮未继续降低到 `50%` 以下。
- 我观察到的变化：
  - 速度滑块从 `100%` 调到 `50%` 后，`/speed_scaling_state_broadcaster/speed_scaling` 稳定变为 `50.0`。
  - 中途一度出现 `0.0%`，对应的是外部控制程序停止，而不是“50% 没生效”。
  - 通过 `/io_and_status_controller/resend_robot_program` 恢复后，speed scaling 从 `0.0 -> 50.0` 再次稳定。

## 6. 练习 3：补写保守轨迹并做执行对比

### 位置
- 文件：`workspaces/ws_stage2/src/ur3_ursim_speed_scaling_lab_py/ur3_ursim_speed_scaling_lab_py/scaled_trajectory_runner.py`
- 函数：`plan_demo_points()`

### TODO(human) 完成后记录
- 我选择移动哪些关节：
  - `shoulder_pan_joint`
  - `shoulder_lift_joint`
- 每个关节变化幅度大约多少：
  - `shoulder_pan_joint += 0.5 rad`
  - `shoulder_lift_joint -= 0.5 rad`
- 为什么认为这条轨迹足够保守：
  - 只动了 2 个关节；
  - 幅度比 5C 的 `1.0 rad` 更小；
  - 轨迹为“起点 -> 中间点 -> 起点”，便于解释发送前后状态。
- `100%` speed scaling 下的执行结果：
  - `speed_scaling_before_send=100.0%`
  - `status=4 error_code=0`
  - `elapsed_sec=5.00`
- `50%` speed scaling 下的执行结果：
  - `speed_scaling_before_send=50.0%`
  - `status=4 error_code=0`
  - `elapsed_sec=10.00`
- 两次耗时差异：
  - `50%` 比 `100%` 多约 `5s`；
  - 总耗时约为 `2x`，符合“speed scaling 拉伸时间轴”的预期。

## 7. 解释闭环
- 为什么这次可以算“真实 speed scaling 实证”：
  - 在 mock hardware 下，speed scaling 始终固定为 `100%`；
  - 这次在 URSim 下，速度滑块改变后，ROS 话题真实变为 `50.0`；
  - 同一条轨迹在 `50%` 与 `100%` 下表现出明显不同的执行耗时。
- 为什么 result 成功不等于“速度滑块没生效”：
  - `scaled_joint_trajectory_controller` 会在低速条件下拉伸时间轴；
  - 路径仍能被完整执行，所以 Action 仍可能返回成功；
  - 真正的差异体现在执行节奏和 feedback 持续时间上，而不是只看 success / failure。
- 如果换成 `joint_trajectory_controller`，我预计会有什么差异：
  - 它不会消费 speed scaling；
  - 在真机或 URSim 外部控制语义下，不如 `scaled_joint_trajectory_controller` 适合作为默认入口。
- 这次实验对未来手术机器人控制链路设计的启发：
  - 状态观测必须和控制结果一起验证，不能只看命令是否发出；
  - “程序停止导致 speed scaling=0” 与 “速度滑块变为 50%” 是两种不同状态，必须分开解释；
  - 做真实控制系统时，验证时间轴变化与状态回传同样重要。

## 8. 完成记录
- 日期：2026-04-12
- 备注：
  - 已完成 6B 核心验收：
    - `speed_scaling=100.0%` 时轨迹耗时约 `5.00s`
    - `speed_scaling=50.0%` 时轨迹耗时约 `10.00s`
    - 两次均返回 `status=4 error_code=0`
  - 中途定位到一次 `reverse interface dropped -> scaled_joint_trajectory_controller inactive -> speed_scaling=0.0` 的现象，并通过重发 robot program 恢复。
