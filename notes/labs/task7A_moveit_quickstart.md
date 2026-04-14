# Task 7A：MoveIt 2 Bringup 与 RViz Quickstart

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[x] fake hardware 下已完成 MoveIt bringup、RViz Quickstart、joint goal / pose goal 规划与执行，以及概念解释闭环`

## 1. 目标
- 在 `workspaces/ws_stage3/src/ur3_moveit_bringup_lab` 中跑通阶段 3 的 MoveIt bringup。
- 复用官方 `ur_moveit_config`，而不是在本任务里自建 MoveIt 配置。
- 在 RViz 中分别完成：
  - 一次 `joint goal`
  - 一次 `pose goal`

## 2. 应用包位置
- 包路径：`/home/minzi/ros2_lab/workspaces/ws_stage3/src/ur3_moveit_bringup_lab`
- 关键文件：
  - `launch/task7A_moveit_quickstart.launch.py`
  - `config/task7A_quickstart.yaml`

## 3. 当前准备情况
- 已完成：
  - `ws_stage3` 工作区骨架；
  - `ur3_moveit_bringup_lab` 包骨架；
  - bringup wrapper launch；
  - 7A 实验记录模板；
  - fake hardware 下的 bringup 联调；
  - RViz Motion Planning 面板中的一次 `joint goal` 与一次 `pose goal`；
  - 对 `ur_manipulator`、`base_link`、`tool0`、`Planning Scene` 的概念解释闭环。

## 4. 练习 1：构建 7A bringup 包

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_bringup_lab
source install/setup.bash
```

### 结果记录
- 是否构建通过：是
- 若失败，错误集中在哪个 launch / package 依赖：无

## 5. 练习 2：启动 MoveIt Quickstart

### 执行命令
```bash
ros2 launch ur3_moveit_bringup_lab task7A_moveit_quickstart.launch.py \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true \
  launch_rviz:=true
```

### 观察记录
- `/robot_description` 是否已发布：是
- RViz 是否出现 Motion Planning 面板：是
- `Planning Group` 下拉框里是否能看到 `ur_manipulator`：是
- 默认末端 link 是否显示为 `tool0`：是

## 6. 练习 3：RViz 中做 joint goal / pose goal

### 记录模板
- `joint goal` 是否规划成功：是
- `joint goal` 是否执行成功：是
- `pose goal` 是否规划成功：是
- `pose goal` 是否执行成功：是
- 你观察到 `Plan` 与 `Execute` 的差别：`Plan` 是根据当前机器人状态和 `Planning Scene` 先求出一条可行轨迹；`Execute` 是把这条轨迹发送给底层 controller 去实际跟踪执行，所以规划成功后执行仍然是一个独立步骤。

## 7. 解释闭环
- `ur_manipulator` 在这个任务里表示 MoveIt 的规划组。它在 `SRDF` 中定义了从 `base_link` 到 `tool0` 的运动学链，告诉 MoveIt 这次规划涉及哪些关节和连杆，以及该对哪条机械臂链做正逆运动学和碰撞检查。它不是 controller 名字；controller 的选择是 MoveIt 执行配置决定的，这里默认对接的是 `scaled_joint_trajectory_controller`。
- `base_link` 在 `pose goal` 里扮演参考坐标系的角色。你在 RViz 里拖动末端手柄时，目标位姿本质上是在描述“`tool0` 相对于 `base_link` 要到什么位置和朝向”。
- `tool0` 适合当末端目标 link，因为在这套 UR MoveIt 配置里，它被定义为机械臂规划链的 tip link，是一个稳定的工具参考坐标系。后续如果再挂夹爪或工具，也通常是在 `tool0` 的基础上再加固定变换。
- `Planning Scene` 影响的是规划，而不是 controller，因为它主要提供碰撞环境、附着物体和场景状态，供 MoveIt 在路径搜索和状态有效性检查时使用。controller 负责执行已经生成好的轨迹，而不是在执行层做碰撞检测或重新规划。
- `Plan` 和 `Execute` 不是一回事。`Plan` 是根据当前机器人状态和 `Planning Scene` 先求出一条可行轨迹；`Execute` 是把这条轨迹发送给底层 controller 去实际跟踪执行，所以规划成功后执行仍然是一个独立步骤。

## 8. Launch 调试记录
- 现象：第一次执行 `ros2 launch ur3_moveit_bringup_lab task7A_moveit_quickstart.launch.py ... launch_rviz:=true` 后，`move_group` 正常启动，但没有看到 RViz 窗口。
- 初步判断：日志里虽然有 `Cannot infer URDF/SRDF` 一类 warning，但 `wait_for_robot_description` 能收到 `/robot_description`，`move_group` 也输出了 `You can start planning now!`，说明 bringup 主链路已经起来，问题更像是 RViz 节点没有被拉起，而不是 MoveIt 本身起不来。
- 根因定位：`task7A_moveit_quickstart.launch.py` 同时 include 了 `ur_control.launch.py` 和 `ur_moveit.launch.py`，两个子 launch 都使用了同名参数 `launch_rviz`。为了关闭 driver 自带的 RViz，上层 launch 给 `ur_control.launch.py` 传了 `launch_rviz:=false`；但 ROS 2 launch 在 `IncludeLaunchDescription` 里会把这个参数写进共享的 launch configuration context，导致后面的 `ur_moveit.launch.py` 读取到的也是被覆盖后的 `false`，从而把 MoveIt 的 RViz 也一起关掉了。
- 修复方法：在上层 launch 里先通过 `SetLaunchConfiguration("task7A_moveit_launch_rviz", LaunchConfiguration("launch_rviz"))` 缓存用户传入的原始 RViz 选项，再让 `ur_moveit.launch.py` 读取这个缓存值；同时继续给 `ur_control.launch.py` 传 `launch_rviz:=false`，只关闭 driver 自带的 RViz。
- 修复验证：重新构建并启动后，日志中出现 `rviz2 process started`，界面正常显示 Motion Planning 面板，且在 RViz 中可以完成 `joint goal` 和 `pose goal` 的规划与执行。

## 9. 完成记录
- 日期：2026.4.14
- 备注：本任务的验收范围已满足，可正式收尾并进入 `7B`。当前仍只在 fake hardware 下完成；这不是缺项，因为 `7A` 的范围本来就限定在 fake hardware Quickstart，不要求引入 URSim。
