# ws_ur3e_controller 真机 home 执行调试记录

记录日期：2026-05-06  
工作区：`/home/minzi/ros2_lab/workspaces/ws_ur3e_controller`  
目标：基于阶段四真机 bringup 新经验，复测 `ws_ur3e_controller`，并完成 UR3e 真机 `home` 命名目标的低速单次执行。

## 1. 最终结论

本次调试最终完成了从只读门闩到真实单点执行的闭环：

- Task 8B 真机 bringup 成功启动硬件链路、Dashboard client 和 External Control。
- Task 8C 动作前门闩通过：robot mode、safety mode、External Control、Remote Control、controller、`/joint_states`、speed scaling 均满足要求。
- `ws_ur3e_controller` 的 `real.home` 目标经现场人工确认后启用。
- `home` 目标先通过 real / plan-only 规划检查。
- `home` 目标随后真实执行成功，并通过 final-target gate。
- 执行后重新运行 Task 8C，系统状态仍为 PASS。

最终 service 响应：

```text
accepted=True
planned=True
executed=True
status='executed'
message="target 'home' executed and final-target gate passed"
```

## 2. 关键经验

本次最重要的经验是分清三层职责：

1. **8B bringup**：负责真机 driver、`ros2_control`、Dashboard client、External Control 生命周期和基础状态流。
2. **8C gate**：负责只读检查动作前门闩，确认机器人和 controller 状态是否允许进入运动任务。
3. **ws_ur3e_controller + MoveIt**：负责命名目标 catalog、joint delta gate、MoveGroup 规划、真机执行门闩、轨迹执行和 final-target gate。

因此，`8B/8C PASS` 不等于 `ws_ur3e_controller` 可以立即规划。`ws_ur3e_controller` 的 `MoveGroupInterface` 还需要 MoveIt 语义层在线，也就是需要启动：

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e \
  launch_rviz:=false \
  launch_servo:=false
```

如果漏启 MoveIt，首次 service 请求会在创建 `MoveGroupInterface` 时失败，典型日志是：

```text
Could not find parameter robot_description_semantic
Unable to parse SRDF
Unable to construct robot model
```

这类错误应优先判断为 MoveIt 语义层缺失，而不是回头怀疑 Dashboard、External Control 或 `/joint_states`。

## 3. 调试时间线

### 3.1 直接运行 8C 失败

最初直接运行：

```bash
ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
  require_trajectory_controller_active:=true
```

结果出现多个服务和 topic 不可用：

```text
Service unavailable: /dashboard_client/get_robot_mode
Service unavailable: /dashboard_client/get_safety_mode
Service unavailable: /dashboard_client/program_running
Service unavailable: /dashboard_client/is_in_remote_control
Service unavailable: /controller_manager/list_controllers
State / /joint_states: no samples
```

判断：8C 是只读检查器，不负责启动真机 driver。必须先启动 8B bringup，并保持 8B 终端运行。

### 3.2 启动 8B 后暴露 External Control manager 兼容性问题

启动 8B：

```bash
ros2 launch ur3_real_bringup_lab task8B_readonly_bringup.launch.py \
  ur_type:=ur3e \
  robot_ip:=192.168.56.101 \
  reverse_ip:=192.168.56.2 \
  launch_rviz:=false \
  activate_joint_controller:=false
```

8B 硬件 gate 已通过：

```text
Hardware ready gate passed: joint_state_broadcaster=active and complete JointState received
Task 8B hardware ready gate passed; launching dashboard client.
```

但随后 `manage_external_control.py` 崩溃：

```text
AttributeError: module 'rclpy' has no attribute 'FutureReturnCode'
```

处理：修复 `workspaces/ws_stage4/src/ur3_real_bringup_lab/scripts/manage_external_control.py`，将不存在的 `rclpy.FutureReturnCode` 判断改为 `spin_until_future_complete(...)` 后检查 `future.done()`。

修复提交：

```text
ec2c133 fix(task8B): 兼容 rclpy future 等待结果
```

验证：

```text
colcon build --packages-select ur3_real_bringup_lab 通过
python3 -m py_compile manage_external_control.py 通过
```

完整 `ur3_real_bringup_lab` 测试仍因包内既有 flake8 风格问题失败，和本次兼容性修复无关。

### 3.3 8C 只读门闩通过到 WARN

8B 正常运行后，先运行只读版 8C：

```bash
ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
  require_external_control:=false \
  require_trajectory_controller_active:=false
```

结果为 WARN，符合预期：

```text
robot mode: RUNNING
safety mode: NORMAL
External Control: Program running: true
remote_control=True
joint_state_broadcaster: active
trajectory controller: inactive
/joint_states: 503.1 Hz
speed scaling: 100.0
Task 8C gate result: WARN
```

判断：`activate_joint_controller:=false` 下 trajectory controller 故意保持 inactive，8C 只读阶段允许 WARN。

### 3.4 激活 trajectory controller 后 8C 动作前门闩 PASS

手动激活：

```bash
ros2 control switch_controllers \
  --activate scaled_joint_trajectory_controller
```

再次运行动作前 8C：

```bash
ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
  require_trajectory_controller_active:=true
```

结果：

```text
trajectory controller: active
/joint_states: 499.5 Hz
speed scaling: 100.0
Task 8C gate result: PASS
```

### 3.5 ws_ur3e_controller real 目标默认禁用门闩通过

在 `runtime_mode:=real`、`execute:=false` 下请求 `home`：

```bash
ros2 service call /ur3e_named_motion_controller/execute_named_target \
  ur3e_controller_msgs/srv/ExecuteNamedTarget \
  "{target_name: home, execute: false, human_confirmation: ''}"
```

初始结果：

```text
accepted=False
planned=False
executed=False
status='rejected_disabled_target'
message="target is present but disabled; reviewed_by='TODO(human): 现场复核后填写'"
```

即使带确认 token 请求执行，也仍被禁用目标门闩挡住：

```text
accepted=False
planned=False
executed=False
status='rejected_disabled_target'
```

判断：确认 token 不能绕过 catalog 中未审核目标的禁用状态，这是正确安全行为。

### 3.6 现场确认后只启用 real.home

经现场人工确认，启用 `real.home`，保持 `ready` 禁用。

涉及文件：

- `src/ur3e_named_motion_controller_cpp/config/ur3e_named_targets.yaml`
- `src/ur3e_named_motion_controller_cpp/test/test_target_catalog.py`

提交：

```text
2c01d9c feat(ur3e-controller): 启用真机 home 命名目标
```

验证：

```text
colcon build --packages-select ur3e_controller_msgs ur3e_named_motion_controller_cpp
colcon test --packages-select ur3e_named_motion_controller_cpp
colcon test-result --verbose
18 tests, 0 failures, 1 skipped
```

### 3.7 漏启 MoveIt 语义层导致 controller 节点退出

只启动：

```bash
ros2 launch ur3e_named_motion_controller_cpp named_motion_controller.launch.py \
  runtime_mode:=real \
  execute:=false
```

然后请求 plan-only，节点在 delta gate 通过后创建 `MoveGroupInterface`，但因缺少 `robot_description_semantic` 退出：

```text
delta gate passed; max_delta=0.005231 rad at wrist_3_joint
Could not find parameter robot_description_semantic
Unable to parse SRDF
Unable to construct robot model
```

处理：新增经验记录，明确 8B/8C 不负责启动 MoveIt 语义层。

经验提交：

```text
7b0a5fc docs(experience): 记录控制器真机规划依赖 MoveIt 语义层
```

### 3.8 启动 MoveIt 后 real/home plan-only PASS

启动 MoveIt：

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e \
  launch_rviz:=false \
  launch_servo:=false
```

重启 named controller：

```bash
ros2 launch ur3e_named_motion_controller_cpp named_motion_controller.launch.py \
  runtime_mode:=real \
  execute:=false
```

再次请求 plan-only：

```bash
ros2 service call /ur3e_named_motion_controller/execute_named_target \
  ur3e_controller_msgs/srv/ExecuteNamedTarget \
  "{target_name: home, execute: false, human_confirmation: ''}"
```

结果：

```text
accepted=True
planned=True
executed=False
status='planned'
message='plan-only request passed catalog, joint-state, delta, and MoveGroup planning gates'
```

### 3.9 真实执行前后 8C 均 PASS，home 执行成功

执行前再次运行 8C：

```bash
ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
  require_trajectory_controller_active:=true
```

结果：

```text
Task 8C gate result: PASS
```

重启 named controller 为执行模式：

```bash
ros2 launch ur3e_named_motion_controller_cpp named_motion_controller.launch.py \
  runtime_mode:=real \
  execute:=true
```

现场确认空间、示教器和急停后，只执行一次：

```bash
ros2 service call /ur3e_named_motion_controller/execute_named_target \
  ur3e_controller_msgs/srv/ExecuteNamedTarget \
  "{target_name: home, execute: true, human_confirmation: 'I_CONFIRM_REAL_ROBOT_MOTION'}"
```

结果：

```text
accepted=True
planned=True
executed=True
status='executed'
message="target 'home' executed and final-target gate passed"
```

执行后再次运行 8C：

```text
Task 8C gate result: PASS
```

## 4. 推荐复测顺序

后续复测 `ws_ur3e_controller` 真机 home 时，建议固定使用以下顺序：

1. 启动 8B bringup：

   ```bash
   cd /home/minzi/ros2_lab/workspaces/ws_stage4
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash

   ros2 launch ur3_real_bringup_lab task8B_readonly_bringup.launch.py \
     ur_type:=ur3e \
     robot_ip:=192.168.56.101 \
     reverse_ip:=192.168.56.2 \
     launch_rviz:=false \
     activate_joint_controller:=false
   ```

2. 激活 trajectory controller：

   ```bash
   ros2 control switch_controllers \
     --activate scaled_joint_trajectory_controller
   ```

3. 跑 8C 动作前门闩：

   ```bash
   ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
     require_trajectory_controller_active:=true
   ```

4. 启动 MoveIt 语义层：

   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 launch ur_moveit_config ur_moveit.launch.py \
     ur_type:=ur3e \
     launch_rviz:=false \
     launch_servo:=false
   ```

5. 启动 named controller plan-only：

   ```bash
   cd /home/minzi/ros2_lab/workspaces/ws_ur3e_controller
   source /opt/ros/jazzy/setup.bash
   source install/setup.bash

   ros2 launch ur3e_named_motion_controller_cpp named_motion_controller.launch.py \
     runtime_mode:=real \
     execute:=false
   ```

6. 请求 `home` plan-only：

   ```bash
   ros2 service call /ur3e_named_motion_controller/execute_named_target \
     ur3e_controller_msgs/srv/ExecuteNamedTarget \
     "{target_name: home, execute: false, human_confirmation: ''}"
   ```

7. 若要真实执行，先重启 named controller 为 `execute:=true`，再重新跑一次 8C PASS，最后只发一次执行请求：

   ```bash
   ros2 service call /ur3e_named_motion_controller/execute_named_target \
     ur3e_controller_msgs/srv/ExecuteNamedTarget \
     "{target_name: home, execute: true, human_confirmation: 'I_CONFIRM_REAL_ROBOT_MOTION'}"
   ```

8. 执行后再跑一次 8C，确认系统仍为 PASS。

## 5. 风险与后续改进

- 本次只验证 `home`，`ready` 仍应保持禁用，直到单独现场复核。
- 不建议在一次会话内连续多次真实执行命令；每次执行后应重新确认 8C。
- 目前真实复测需要多个终端手动编排，容易漏启 MoveIt 语义层。
- 后续可新增 `real_named_motion_bringup.launch.py`，把 8B ready、MoveIt move_group、named controller 的启动顺序固化。
- 如果再次出现 `robot_description_semantic` 或 SRDF 相关错误，优先检查 `ur_moveit_config ur_moveit.launch.py` 是否在线。

## 6. 本次关联提交

```text
ec2c133 fix(task8B): 兼容 rclpy future 等待结果
2c01d9c feat(ur3e-controller): 启用真机 home 命名目标
7b0a5fc docs(experience): 记录控制器真机规划依赖 MoveIt 语义层
```
