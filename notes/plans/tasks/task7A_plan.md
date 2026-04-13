# 任务 7A 规划文档：MoveIt 2 Bringup 与 RViz Quickstart

## 1. 任务目标
- 在 `ws_stage3` 新建一个独立包 `ur3_moveit_bringup_lab`。
- 复用官方 `ur_moveit_config` 跑通：
  - `ur_robot_driver/ur_control.launch.py`
  - `ur_moveit_config/ur_moveit.launch.py`
- 在 RViz 中完成一次 `joint goal` 和一次 `pose goal` 的规划与执行。

## 2. 当前基线（来自仓库现状）
- `6B` 已完成：仓库已经能在 URSim 下做真实 speed scaling 验证。
- 当前仓库还没有阶段 3 的 MoveIt 2 工作区与 launch 胶水包。
- 本机已安装 `ur_moveit_config`，因此本任务不需要从零生成 MoveIt 配置。

## 3. 任务范围（单功能约束）
- 包含：
  - 新建 `ur3_moveit_bringup_lab` 包；
  - 提供一个阶段 3 的 bringup launch；
  - 在 fake hardware 下验证 MoveIt 2 + RViz Quickstart。
- 不包含：
  - 自写 MoveGroupInterface C++ 节点；
  - Collision Object / Planning Scene 编程接口；
  - URSim 验证；
  - MoveIt Servo。

## 4. 包设计
- 包名：`ur3_moveit_bringup_lab`
- 位置：`workspaces/ws_stage3/src/ur3_moveit_bringup_lab`
- 目录骨架：
  ```text
  ur3_moveit_bringup_lab/
  ├── CMakeLists.txt
  ├── package.xml
  ├── config/
  │   └── task7A_quickstart.yaml
  └── launch/
      └── task7A_moveit_quickstart.launch.py
  ```

## 5. 核心学习问题
1. `Planning Scene` 在 RViz Quickstart 里扮演什么角色？
2. 为什么 `planning group` 是 `ur_manipulator`，而不是 controller 名称？
3. `tool0` 作为 pose goal 目标 link 时，和 “关节目标” 的思维方式有什么不同？
4. 为什么 “Plan 成功” 不等于 “已经执行”？

## 6. learn mode 分工
- 智能体负责：
  - 包骨架、launch 胶水、参数文件与构建验证。
- 人类负责：
  - 在 RViz 里亲手完成交互；
  - 用自己的话解释：
    - `ur_manipulator`
    - `base_link`
    - `tool0`
    - `Planning Scene`

## 7. 实施步骤
1. 新建 `ws_stage3` 与 `ur3_moveit_bringup_lab`。
2. 先在 fake hardware 下启动 driver。
3. 再启动 MoveIt bringup，确认 `move_group` 与 RViz 正常出现。
4. 在 RViz 中分别完成：
   - `joint goal`
   - `pose goal`
5. 记录：
   - 规划是否成功；
   - 执行是否成功；
   - joint goal 与 pose goal 的交互感受差异。

## 8. 关键命令（执行阶段使用）
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_bringup_lab
source install/setup.bash

ros2 launch ur3_moveit_bringup_lab task7A_moveit_quickstart.launch.py \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true \
  launch_rviz:=true
```

## 9. 交付物
- 代码：`workspaces/ws_stage3/src/ur3_moveit_bringup_lab`
- 文档：`notes/labs/task7A_moveit_quickstart.md`

## 10. 验收标准
- fake hardware 下能启动 driver + move_group + RViz。
- RViz 中能完成一次 `joint goal` 规划与执行。
- RViz 中能完成一次 `pose goal` 规划与执行。
- 你能解释 “Plan” 和 “Execute” 不是一回事。

## 11. 风险与回退
- 风险 1：`move_group` 起不来。
  - 回退：先确认 `/robot_description` 已由 driver 正常发布，再启动 MoveIt。
- 风险 2：RViz 中没有可用的规划组。
  - 回退：检查 SRDF 是否加载成功，确认 `ur_manipulator` 出现在 Motion Planning 面板。
- 风险 3：把 URSim 一起引入，排障面变大。
  - 回退：本任务只在 fake hardware 下完成。
