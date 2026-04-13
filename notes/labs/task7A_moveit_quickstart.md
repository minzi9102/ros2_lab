# Task 7A：MoveIt 2 Bringup 与 RViz Quickstart

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[#] 已创建包骨架与实验模板，待你完成 RViz Quickstart 联调`

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
  - 7A 实验记录模板。
- 待你完成：
  - 在 fake hardware 下亲手启动 bringup；
  - 在 RViz Motion Planning 面板中完成一次 `joint goal` 与一次 `pose goal`；
  - 用自己的语言解释 `ur_manipulator`、`base_link`、`tool0`、`Planning Scene`。

## 4. 练习 1：构建 7A bringup 包

### 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_bringup_lab
source install/setup.bash
```

### 结果记录
- 是否构建通过：
- 若失败，错误集中在哪个 launch / package 依赖：

## 5. 练习 2：启动 MoveIt Quickstart

### 执行命令
```bash
ros2 launch ur3_moveit_bringup_lab task7A_moveit_quickstart.launch.py \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true \
  launch_rviz:=true
```

### 观察记录
- `/robot_description` 是否已发布：
- RViz 是否出现 Motion Planning 面板：
- `Planning Group` 下拉框里是否能看到 `ur_manipulator`：
- 默认末端 link 是否显示为 `tool0`：

## 6. 练习 3：RViz 中做 joint goal / pose goal

### 记录模板
- `joint goal` 是否规划成功：
- `joint goal` 是否执行成功：
- `pose goal` 是否规划成功：
- `pose goal` 是否执行成功：
- 你观察到 `Plan` 与 `Execute` 的差别：

## 7. 解释闭环
- `ur_manipulator` 在这个任务里表示什么：
- `base_link` 为什么是规划参考系：
- `tool0` 为什么能作为 pose goal 的目标 link：
- `Planning Scene` 为什么会影响规划，而不是直接影响 controller：

## 8. 完成记录
- 日期：
- 备注：
