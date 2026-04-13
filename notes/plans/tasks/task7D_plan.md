# 任务 7D 规划文档：点击目标位姿 -> 自动规划执行 Demo

## 1. 任务目标
- 在 `ws_stage3` 新建一个独立 C++ 包 `ur3_moveit_goal_pose_lab_cpp`。
- 订阅 `/goal_pose`，把 RViz 的 `2D Goal Pose` 输入转换为桌面抓取位姿。
- 自动完成规划与执行，形成一个“点击 -> 机械臂动起来”的最小 demo。

## 2. 当前基线（来自仓库现状）
- `7C` 将已具备桌面碰撞体和 MoveGroup 规划骨架。
- 当前仓库还没有基于 RViz 输入的自动规划执行节点。

## 3. 任务范围（单功能约束）
- 包含：
  - `/goal_pose` 订阅；
  - 2D 点击到 3D 抓取姿态的最小映射；
  - 越界输入拒绝；
  - fake hardware 必做，URSim 做 1 次演示。
- 不包含：
  - 自写 interactive marker；
  - 多目标队列；
  - 视觉感知；
  - MoveIt Servo。

## 4. 包设计
- 包名：`ur3_moveit_goal_pose_lab_cpp`
- 位置：`workspaces/ws_stage3/src/ur3_moveit_goal_pose_lab_cpp`
- 目录骨架：
  ```text
  ur3_moveit_goal_pose_lab_cpp/
  ├── CMakeLists.txt
  ├── package.xml
  ├── launch/
  │   └── task7D_goal_pose_auto_plan.launch.py
  └── src/
      └── goal_pose_executor_node.cpp
  ```

## 5. 默认策略
- 保留点击得到的 `x / y / yaw`
- 固定 `z` 为桌面上方的安全高度
- 固定末端为“朝下抓取”姿态
- 超出桌面工作区的输入直接拒绝

## 6. learn mode 分工
- 智能体负责：
  - 订阅器、参数、日志、范围校验框架。
- 人类负责：
  - “点击点如何映射为抓取姿态”的核心判断；
  - 工作区范围边界；
  - 哪些输入应该立即拒绝。

## 7. 核心学习问题
1. 为什么这里选 `/goal_pose` 而不是自己写 interactive marker？
2. 为什么二维点击输入必须经过语义映射，不能直接当作末端最终位姿？
3. 如果目标超出桌面范围，为什么应该在规划前就拒绝？

## 8. 实施步骤
1. 新建包并确保构建通过。
2. 添加 `/goal_pose` 订阅与参数框架。
3. 完成目标映射函数。
4. 对合法目标做自动规划与执行。
5. 对越界或姿态非法目标只输出错误，不执行。

## 9. 关键命令（执行阶段使用）
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage3
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_moveit_goal_pose_lab_cpp
source install/setup.bash

ros2 launch ur3_moveit_goal_pose_lab_cpp task7D_goal_pose_auto_plan.launch.py
```

## 10. 交付物
- 代码：`workspaces/ws_stage3/src/ur3_moveit_goal_pose_lab_cpp`
- 文档：`notes/labs/task7D_goal_pose_auto_plan.md`

## 11. 验收标准
- 节点能订阅 `/goal_pose`。
- 桌面范围内点击目标时，节点能自动规划并执行。
- 越界目标不会触发执行。
- URSim 下完成一次点击演示。
