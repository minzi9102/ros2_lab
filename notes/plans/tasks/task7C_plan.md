# 任务 7C 规划文档：Planning Scene、Collision Object 与 Cartesian Path

## 1. 任务目标
- 在 `ws_stage3` 新建一个独立 C++ 包 `ur3_moveit_scene_lab_cpp`。
- 在程序中添加桌面 `CollisionObject`。
- 让 UR3 从 home 位经过 `3` 个抓取点位，并比较：
  - pose planning
  - `computeCartesianPath`

## 2. 当前基线（来自仓库现状）
- `7B` 将提供最小 MoveGroupInterface 规划节点。
- 当前仓库还没有任何通过程序修改 Planning Scene 的练习包。

## 3. 任务范围（单功能约束）
- 包含：
  - Planning Scene 接口最小使用；
  - 桌面碰撞体增删；
  - 3 点 Cartesian Path 最小验证；
  - fake hardware 必做，URSim 可做 1 次冒烟验证。
- 不包含：
  - `goal_pose` 点击 demo；
  - MoveIt Servo；
  - 多障碍物、多工件场景管理；
  - Pick and Place。

## 4. 包设计
- 包名：`ur3_moveit_scene_lab_cpp`
- 位置：`workspaces/ws_stage3/src/ur3_moveit_scene_lab_cpp`
- 目录骨架：
  ```text
  ur3_moveit_scene_lab_cpp/
  ├── CMakeLists.txt
  ├── package.xml
  ├── launch/
  │   └── task7C_scene_cartesian.launch.py
  └── src/
      └── scene_cartesian_demo_node.cpp
  ```

## 5. learn mode 分工
- 智能体负责：
  - Planning Scene 更新胶水；
  - `CollisionObject` 消息骨架；
  - Cartesian 结果日志与参数框架。
- 人类负责：
  - 桌面碰撞体尺寸与位置；
  - `3` 个抓取点位设计；
  - `fraction` 多低算失败的判断。

## 6. 核心学习问题
1. `Planning Scene` 里的桌面为什么会影响规划结果，而不是影响 controller？
2. 为什么 `computeCartesianPath` 会返回 `fraction`，而不是简单的成功 / 失败？
3. pose planning 与 Cartesian Path 各自更适合什么场景？

## 7. 实施步骤
1. 新建 C++ 包并先构建通过。
2. 用程序添加一个桌面碰撞体。
3. 先做一次 pose planning 验证桌面约束是否生效。
4. 再增加 `computeCartesianPath`，让 UR3 经过 `3` 个点位。
5. 记录：
   - 是否成功添加桌面；
   - 低于桌面的目标是否被拒绝；
   - `fraction` 值与结果解释。

## 8. 关键命令（执行阶段使用）
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_scene_lab_cpp
source install/setup.bash

ros2 launch ur3_moveit_scene_lab_cpp task7C_scene_cartesian.launch.py
```

## 9. 交付物
- 代码：`workspaces/ws_stage3/src/ur3_moveit_scene_lab_cpp`
- 文档：`notes/labs/task7C_planning_scene_collision.md`

## 10. 验收标准
- 能通过程序添加桌面碰撞体。
- 低于桌面的目标位姿不会被当成正常可执行轨迹。
- `3` 个抓取点位的 Cartesian Path 能返回 `fraction` 并被正确解释。
- URSim 下至少完成一次安全轨迹的冒烟执行。
