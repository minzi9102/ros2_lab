# UR3e 手眼标定运动工作区

该工作区只负责 `base_link <- tool0` 的两阶段规划与执行，不包含相机采样或手眼矩阵求解。

## 构建与启动

所有 ROS 命令都按以下顺序加载覆盖层：

```bash
source /opt/ros/jazzy/setup.bash
source /home/minzi/ros2_lab/workspaces/ws_stage3/install/setup.bash
source /home/minzi/ros2_lab/workspaces/ws_stage4/install/setup.bash
source /home/minzi/ros2_lab/workspaces/ws_handeye_calibration/install/setup.bash
```

一键启动真机 bringup、MoveIt、RViz 和运动 API：

```bash
ros2 launch ur3e_handeye_motion handeye_motion.launch.py
```

无头模式使用 `launch_rviz:=false`。启动过程会把速度滑块设为 10%；门禁没有就绪时，规划和执行请求都会保持机械臂不动。

已有 driver、Dashboard 和 MoveIt 时，只启动接口节点：

```bash
ros2 launch ur3e_handeye_motion motion_server.launch.py
```

## 两阶段调用

规划目标位姿：

```bash
ros2 service call /handeye_motion/plan_pose ur3e_handeye_motion/srv/PlanPose "{
  target: {
    header: {frame_id: base_link},
    pose: {
      position: {x: 0.355990762, y: 0.198345853, z: 0.058619190},
      orientation: {x: -0.219401936, y: 0.757383177, z: 0.159643951, w: 0.593925351}
    }
  }
}"
```

确认响应中的轨迹、关节位移和 RViz 预览后，只提交该响应返回的 `plan_id`：

```bash
ros2 action send_goal --feedback \
  /handeye_motion/execute_plan \
  ur3e_handeye_motion/action/ExecutePlan \
  "{plan_id: 1}"
```

每个 Plan 只能执行一次；新规划会使旧 `plan_id` 失效。执行前会重新运行 8C 门禁并核对轨迹起点，执行后按位置 1 mm、姿态 1° 验收。失败、取消或超差均不会自动重试或补偿。
