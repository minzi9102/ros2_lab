# 任务 5C 规划文档：最小控制闭环（实践版）

## 1. 任务目标
- 在阶段 2 工作区新建一个独立应用包 `ur3_minimal_control_lab_py`，集中存放 5C 任务代码。
- 先完成一条最小闭环：`观测 /joint_states -> 发送 FollowJointTrajectory -> 回看执行结果`。
- 为后续手术机器人系统开发打基础，重点练习"状态观测、动作下发、执行验证"三件事的接口边界。

## 2. 当前基线（来自仓库现状）
- 5A 已完成：`ur_robot_driver` 在 mock hardware 模式下可稳定启动，`scaled_joint_trajectory_controller` 可见。
- 5B 已完成：已理解 `scaled_joint_trajectory_controller`、speed scaling、驱动/控制器/MoveIt 分工。
- 阶段 1 已有可复用参考：
  - `ur3_follow_joint_trajectory_client_py`：最小 Python Action Client 写法；
  - `ur3_state_monitor_cpp`：状态观测节点的打印与参数化模式。
- `workspaces/ws_stage2/src` 当前为空，正适合作为 5C 的新应用包落点。

## 3. 任务范围（单功能约束）
- 包含：
  - 新建一个 Python 应用包；
  - 包内提供 2 个节点：关节状态观察节点、轨迹发送节点；
  - 提供 1 个 launch 文件，方便在 fake hardware 环境下启动观察链路；
  - 提供实验记录模板与执行步骤。
- 不包含：
  - C++ 复刻版本；
  - MoveIt 规划集成；
  - 多段复杂轨迹、速度/加速度优化；
  - 真机、URSim、实时内核与安全 IO 主题。

> 说明：旧版 5C 把 Python 与 C++ 放在同一任务中，这与仓库的"单功能约束"冲突。实践版先收敛到一个 Python 应用包，等闭环跑通后再单开后续扩展任务更稳妥。

## 4. 新建应用包设计（本任务硬性要求）
- 包名：`ur3_minimal_control_lab_py`
- 位置：`workspaces/ws_stage2/src/ur3_minimal_control_lab_py`
- 目录骨架：
  ```text
  ur3_minimal_control_lab_py/
  ├── package.xml
  ├── setup.py
  ├── setup.cfg
  ├── resource/
  │   └── ur3_minimal_control_lab_py
  ├── launch/
  │   └── task5C_minimal_control.launch.py
  └── ur3_minimal_control_lab_py/
      ├── __init__.py
      ├── joint_state_observer.py
      └── joint_trajectory_sender.py
  ```

## 5. learn mode 分工
- 智能体负责（低学习价值，可直接完成）：
  - 包脚手架、依赖声明、安装规则；
  - 观察节点、launch 文件、Action Client 外层胶水代码；
  - 轨迹点校验、命令清单、实验文档模板。
- 人类负责（关键学习点，执行时需你亲自决策）：
  - 设计 demo 轨迹点：目标关节角、每段持续时间、是否回到起点；
  - 判断当前位姿到目标位姿的位移是否足够温和；
  - 观察执行过程中的 `/joint_states` 变化并解释结果；
  - 决定 5C 完成后是否继续做 C++ 迁移任务。

## 6. 核心学习问题
1. 为什么 5C 发送目标时应优先连接 `/scaled_joint_trajectory_controller/follow_joint_trajectory`，而不是旧阶段里使用的普通 JTC？
2. 发送 Action goal 之前，为什么先读一次 `/joint_states` 会更安全？
3. Action result 成功后，如何用 `/joint_states` 证明"控制器真的走到了目标附近"？

## 7. 实施步骤（推荐 3 次练习）

### 练习 1：只做状态观测
1. 在 `ws_stage2` 构建 `ur3_minimal_control_lab_py`。
2. 启动官方 UR mock hardware。
3. 只运行 `joint_state_observer`，确认能按固定顺序打印 6 个关节角。
4. 记录机械臂初始位姿，不急着发轨迹。

### 练习 2：补写最小轨迹并验证 Action 链路
1. 在 `joint_trajectory_sender.py` 的 `TODO(human)` 位置补写 2 到 3 个轨迹点。
2. 先选择温和动作，建议只让 1 到 2 个关节发生小幅变化。
3. 运行发送节点，观察：
   - goal 是否 accepted；
   - feedback 中 desired / actual 是否变化；
   - result 的 `error_code` 是否为成功。

### 练习 3：完成闭环解释
1. 发送轨迹前后各记录一次 `/joint_states`。
2. 比较目标值与实际收敛值的差距。
3. 在实验文档中回答：
   - 这次轨迹是如何被控制器消费的？
   - 为什么 fake hardware 下也能验证控制闭环？
   - 如果换成 `joint_trajectory_controller`，本实验会有什么不同？

## 8. 关键命令（执行阶段使用）
```bash
# 构建 5C 应用包
cd /home/minzi/ros2_lab/workspaces/ws_stage2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_minimal_control_lab_py
source install/setup.bash

# 启动 UR 官方 mock hardware
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur3 \
  robot_ip:=192.168.56.101 \
  use_mock_hardware:=true \
  launch_rviz:=true

# 启动 5C 观察链路（默认只启动 observer）
ros2 launch ur3_minimal_control_lab_py task5C_minimal_control.launch.py

# 单独运行发送节点（补完 TODO(human) 之后）
ros2 run ur3_minimal_control_lab_py joint_trajectory_sender

# 执行前确认 Action server
ros2 action list
ros2 action info /scaled_joint_trajectory_controller/follow_joint_trajectory

# 验证关节状态
ros2 topic echo /joint_states --once
```

## 9. 交付物
- 代码：`workspaces/ws_stage2/src/ur3_minimal_control_lab_py`
- 文档：`notes/labs/task5C_minimal_control_node.md`
- 结果：能清楚演示一次"先观察、再发送、再验证"的最小控制闭环。

## 10. 验收标准
- `joint_state_observer` 能持续输出 6 个 UR3 关节的当前角度。
- `joint_trajectory_sender` 在补完 `TODO(human)` 后能成功连接 Action server 并发送目标。
- 你能用自己的语言解释：
  - 为什么本任务优先使用 `scaled_joint_trajectory_controller`；
  - Action 成功与"状态真的到位"之间的区别；
  - 本次实验对未来手术机器人控制开发的启发。

## 11. 风险与回退
- 风险 1：`/joint_states` 顺序与代码中的 `joint_names` 不一致。
  - 回退：先 `ros2 topic echo /joint_states --once` 核对名称，再修正节点参数。
- 风险 2：初次设计的轨迹点过大，难以判断反馈是否合理。
  - 回退：先只动 `shoulder_pan_joint` 或 `elbow_joint` 的小幅度角度。
- 风险 3：把"补完轨迹点"和"多段复杂轨迹实验"一次做完，导致排障面扩大。
  - 回退：严格先做单段、小幅、可回退的最小动作。

## 12. 后续扩展（不属于本次 5C）
- 若 5C Python 闭环顺利完成，可新开独立任务：
  - C++ 版 Action Client 迁移；
  - 轨迹参数化与 YAML 外置；
  - URSim 接入与 speed scaling 真实链路验证。

## 13. 完成记录
- 状态：`[ ] 未开始` `[#] 进行中` `[x] 已完成`
- 日期：`2026-04-12`
- 备注：已按当前仓库状态重规划，并要求 5C 代码统一放入 `ws_stage2` 新应用包。
