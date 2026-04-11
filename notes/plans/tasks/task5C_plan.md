# 任务 5C 规划文档：最小控制程序（Python + C++）

## 1. 任务目标
- 用 Python 写一个最小节点，向 UR3 发送关节轨迹并订阅机器人状态。
- 用 C++ 复现同等功能，对比两种语言在 ROS 2 节点开发上的差异。
- 读取当前关节角，验证指令执行结果，形成完整的"发指令 → 观测状态"闭环。

## 2. 当前基线（来自仓库现状）
- 5A 完成后：fake hardware 链路可用，`scaled_joint_trajectory_controller` 可接收轨迹目标。
- 5B 完成后：理解控制器体系，知道应向哪个 Action server 发送目标。
- 4F 完成后：已有 Python FollowJointTrajectory Action Client 经验，本任务在此基础上迁移到官方 UR 驱动环境。

## 3. 任务范围（单功能约束）
- 包含：
  - Python 最小控制节点（发轨迹 + 订阅 `/joint_states`）；
  - C++ 最小控制节点（功能等价）；
  - 两个节点均在 fake hardware 环境下验证通过；
  - Python vs C++ 实现差异对比记录。
- 不包含：
  - MoveIt 规划接口；
  - 多段轨迹、速度/加速度约束优化；
  - 真机或 URSim 接入；
  - GUI 或可视化工具开发。

## 4. 新建应用包设计（本任务硬性要求）

### Python 包
- 包名：`ur3_minimal_control_py`
- 位置：`workspaces/ws_tutorials/src/ur3_minimal_control_py`
- 目录骨架：
  ```
  ur3_minimal_control_py/
  ├── package.xml
  ├── setup.py
  ├── setup.cfg
  ├── resource/
  │   └── ur3_minimal_control_py
  └── ur3_minimal_control_py/
      ├── __init__.py
      ├── joint_trajectory_sender.py   # 发送关节轨迹的 Action Client
      └── joint_state_monitor.py       # 订阅并打印当前关节角
  ```

### C++ 包
- 包名：`ur3_minimal_control_cpp`
- 位置：`workspaces/ws_tutorials/src/ur3_minimal_control_cpp`
- 目录骨架：
  ```
  ur3_minimal_control_cpp/
  ├── package.xml
  ├── CMakeLists.txt
  └── src/
      ├── joint_trajectory_sender.cpp  # 发送关节轨迹的 Action Client
      └── joint_state_monitor.cpp      # 订阅并打印当前关节角
  ```

## 5. 核心实现要点

### 5.1 轨迹发送节点（Python 参考结构）
```python
# 关键要素：
# 1. Action server 名称：/scaled_joint_trajectory_controller/follow_joint_trajectory
# 2. joint_names 必须与 URDF 中声明的一致（UR3 共 6 个关节）
# 3. 至少包含 2 个轨迹点（起点 + 终点），time_from_start 必须递增
# 4. 等待 result 后打印 error_code
```

### 5.2 关节状态订阅节点（C++ 参考结构）
```cpp
// 关键要素：
// 1. 订阅 /joint_states（sensor_msgs/msg/JointState）
// 2. 打印 name、position、velocity 三个字段
// 3. 使用 rclcpp::spin_some 或 spin 处理回调
```

### 5.3 UR3 关节名称（必须与驱动一致）
```
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
```

## 6. learn mode 分工
- 智能体负责（低学习价值，可直接完成）：
  - 包骨架、CMakeLists.txt、package.xml、setup.py 模板；
  - 关节名称列表、Action server 名称确认。
- 人类负责（关键学习点，执行时需你亲自决策）：
  - 设计测试轨迹：选择目标关节角度和执行时间（需在关节限位内）；
  - 对比 Python 与 C++ 实现：哪里更繁琐？哪里更清晰？
  - 观察 Action feedback：执行过程中 `/joint_states` 如何变化？
  - 验证：发送目标后，`/joint_states` 的 position 是否收敛到目标值？

## 7. 实施步骤（3 天节奏）

### Day 1：Python 节点实现与验证
1. 新建 `ur3_minimal_control_py` 包骨架。
2. 实现 `joint_trajectory_sender.py`：
   - 连接 `/scaled_joint_trajectory_controller/follow_joint_trajectory`；
   - 发送单段轨迹（`shoulder_pan_joint` 从 0 移动到 0.5 rad，其余关节保持 0，执行时间 3s）；
   - 打印 goal accepted / result。
3. 实现 `joint_state_monitor.py`：
   - 订阅 `/joint_states`，每秒打印一次 6 个关节的当前角度。
4. 在 fake hardware 环境下验证两个节点。

### Day 2：C++ 节点实现与验证
1. 新建 `ur3_minimal_control_cpp` 包骨架，配置 CMakeLists.txt。
2. 实现 `joint_trajectory_sender.cpp`（功能与 Python 版等价）。
3. 实现 `joint_state_monitor.cpp`（功能与 Python 版等价）。
4. 构建并在 fake hardware 环境下验证。

### Day 3：对比总结与文档沉淀
1. 对比记录：
   - Python vs C++ 代码行数差异；
   - 编译/运行流程差异；
   - Action Client 写法差异（`rclpy.action` vs `rclcpp_action`）。
2. 填写 `notes/labs/task5C_minimal_control_node.md`。
3. 更新 `task5C_plan.md` 完成记录。

## 8. 关键命令（执行阶段使用）
```bash
# 构建两个包
cd /home/minzi/ros2_lab/workspaces/ws_tutorials
colcon build --packages-select ur3_minimal_control_py ur3_minimal_control_cpp
source install/setup.bash

# 启动 fake hardware（前置条件）
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 robot_ip:=yyy.yyy.yyy.yyy \
  use_fake_hardware:=true

# 运行 Python 轨迹发送节点
ros2 run ur3_minimal_control_py joint_trajectory_sender

# 运行 Python 状态监控节点
ros2 run ur3_minimal_control_py joint_state_monitor

# 运行 C++ 轨迹发送节点
ros2 run ur3_minimal_control_cpp joint_trajectory_sender

# 运行 C++ 状态监控节点
ros2 run ur3_minimal_control_cpp joint_state_monitor

# 验证 Action server 是否可用
ros2 action list
ros2 action info /scaled_joint_trajectory_controller/follow_joint_trajectory
```

## 9. 交付物
- 代码：`ur3_minimal_control_py` 包（含 2 个节点）。
- 代码：`ur3_minimal_control_cpp` 包（含 2 个节点）。
- 文档：`notes/labs/task5C_minimal_control_node.md`
  - 包含：实现思路、关键代码片段、Python vs C++ 对比、执行日志截图。

## 10. 验收标准
- Python 节点能在 fake hardware 下成功发送轨迹，Action result 返回成功。
- C++ 节点能复现相同功能，构建无报错。
- 两个状态监控节点均能实时打印 6 个关节的当前角度。
- 能解释 Python 与 C++ 实现 Action Client 的主要差异。

## 11. 风险与回退
- 风险 1：Action server 名称与实际不符（驱动版本差异）。
  - 回退：先用 `ros2 action list` 确认实际 Action server 名称，再修改节点代码。
- 风险 2：joint_names 顺序与 `/joint_states` 不一致导致轨迹执行异常。
  - 回退：先 `ros2 topic echo /joint_states --once` 确认关节名称顺序，严格对齐。
- 风险 3：C++ Action Client 编译依赖缺失。
  - 回退：检查 `package.xml` 中 `control_msgs`、`rclcpp_action` 依赖是否声明；参考 4F C++ 包的 CMakeLists.txt。

## 12. 完成记录
- 状态：`[ ] 未开始` `[#] 进行中` `[x] 已完成`
- 日期：
- 备注：
