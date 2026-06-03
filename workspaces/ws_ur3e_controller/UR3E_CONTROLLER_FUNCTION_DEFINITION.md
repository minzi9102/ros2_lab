# UR3e 自制命名点控制器功能定义

本文档定义 `/home/minzi/ros2_lab/workspaces/ws_ur3e_controller` 中当前 UR3e 自制控制器的功能边界、接口契约、执行门闩和验收标准。它面向后续继续实现、仿真验收和真机低速验证使用。

## 1. 控制器定位

当前控制器是一个基于 MoveIt MoveGroup 的上层命名关节目标控制器。

它的职责是：

- 常驻运行一个 ROS 2 节点。
- 通过 service 接收一次命名目标请求。
- 只允许执行 YAML catalog 中已经列出的命名关节目标。
- 在规划前检查当前 joint state、目标合法性和每关节位移差。
- 在仿真模式下支持 plan-only 和单次 execute。
- 在真机模式下额外要求人工确认 token、Dashboard 状态、External Control、controller、joint state 和 speed scaling 门闩通过。

它的首版边界是：

- 不重写 `ros2_control`。
- 不直接驱动底层硬件接口。
- 不在 v1 发送 MoveIt Servo 非零速度命令。
- 不提供连续 Servo、笛卡尔点击、waypoint 队列、自动重试、无人值守执行或 Action cancel。
- 不自动调用 `unlock_protective_stop`、`restart_safety`、`brake_release`、`power_on`、`load`、`play` 等真机恢复流程。

## 2. 当前包结构

工作空间包含两个核心包：

- `ur3e_controller_msgs`
  - 定义控制器 service 接口。
  - 当前接口文件：`srv/ExecuteNamedTarget.srv`。

- `ur3e_named_motion_controller_cpp`
  - 实现命名点控制器节点。
  - 节点可执行文件：`named_motion_controller_node`。
  - 节点名：`ur3e_named_motion_controller`。
  - 默认目标 catalog：`config/ur3e_named_targets.yaml`。
  - 主要 launch：
    - `launch/named_motion_controller.launch.py`
    - `launch/sim_named_motion_bringup.launch.py`

## 3. Service 接口

服务名：

```text
/ur3e_named_motion_controller/execute_named_target
```

接口类型：

```text
ur3e_controller_msgs/srv/ExecuteNamedTarget
```

请求字段：

```text
string target_name
bool execute
string human_confirmation
```

响应字段：

```text
bool accepted
bool planned
bool executed
string status
string message
```

字段语义：

- `target_name`：请求的命名目标，必须存在于当前 `runtime_mode` 对应的 catalog 中，并且 `enabled=true`。
- `execute`：`false` 表示只规划不执行；`true` 表示规划通过后尝试执行。
- `human_confirmation`：真机执行确认 token。仿真模式和 plan-only 请求不依赖该字段。
- `accepted`：请求是否通过前置合法性检查。规划失败时请求仍可能是 `accepted=true`。
- `planned`：MoveGroup 是否成功规划。
- `executed`：MoveGroup 是否已调用并完成 execute。若 final-target gate 失败，该字段仍可能为 `true`，表示轨迹执行调用已经发生。
- `status`：机器可读的状态码。
- `message`：面向操作者的详细原因。

## 4. 运行模式

当前支持两个运行模式：

- `sim`
  - 面向 fake hardware、URSim 或仿真验收。
  - 默认允许较大的 `max_joint_delta_rad`。
  - `require_joint_state_stamp=false`，适配部分仿真 joint state 没有真实时间戳的情况。
  - catalog 中 `home`、`ready` 默认启用。

- `real`
  - 面向真实 UR3e 低速单目标执行。
  - 默认每关节最大位移差更小。
  - 默认要求 non-zero joint state stamp。
  - 默认要求 Remote Control。
  - 默认不接受 `REDUCED` safety mode，除非人工审核后在 catalog 中显式设置 `allow_reduced_safety_mode=true`。
  - catalog 中真机目标默认 `enabled=false`，必须由现场人员复核后才能启用。

## 5. Launch 参数

`named_motion_controller.launch.py` 暴露的关键参数：

- `runtime_mode`
  - 默认：`sim`
  - 可选：`sim` 或 `real`
  - 用于选择 catalog 内对应配置段。

- `execute`
  - 默认：`false`
  - 传入节点参数 `allow_execution`。
  - 当它为 `false` 时，即使 service 请求 `execute=true` 也会被拒绝。

- `target_catalog`
  - 默认：`ur3e_named_motion_controller_cpp/config/ur3e_named_targets.yaml`
  - 指向命名目标 YAML catalog。

- `joint_state_topic`
  - 默认：`/joint_states`
  - 用于当前状态、delta gate 和 final-target gate。

节点内部还定义以下参数，当前 launch 未全部外显：

- `joint_state_timeout_sec`：等待首个 joint state 的超时，默认 `3.0`。
- `max_joint_state_age_sec`：joint state 和 speed scaling 最大接收年龄，默认 `1.0`。
- `speed_scaling_topic`：默认 `/speed_scaling_state_broadcaster/speed_scaling`。
- `human_confirmation_token`：默认 `I_CONFIRM_REAL_ROBOT_MOTION`。
- `robot_mode_service`：默认 `/dashboard_client/get_robot_mode`。
- `safety_mode_service`：默认 `/dashboard_client/get_safety_mode`。
- `program_running_service`：默认 `/dashboard_client/program_running`。
- `remote_control_service`：默认 `/dashboard_client/is_in_remote_control`。
- `list_controllers_service`：默认 `/controller_manager/list_controllers`。
- `service_timeout_sec`：Dashboard 和 controller manager 服务等待超时，默认 `3.0`。

## 6. Target Catalog Schema

目标配置文件为 YAML，当前 schema version 为 `1`。

顶层结构：

```yaml
schema_version: 1
runtime_modes:
  sim:
    planning_group: ur_manipulator
    joint_names: [...]
    max_joint_delta_rad: 3.20
    final_position_tolerance_rad: 0.03
    final_state_timeout_sec: 5.0
    planning_time_sec: 3.0
    planning_attempts: 3
    max_velocity_scaling: 0.10
    max_acceleration_scaling: 0.10
    require_joint_state_stamp: false
    required_active_controllers: [...]
    targets:
      home:
        enabled: true
        reviewed_by: simulation baseline
        positions_rad: [...]
  real:
    ...
```

每个 runtime mode 的关键字段：

- `planning_group`：MoveIt planning group，目前为 `ur_manipulator`。
- `joint_names`：目标关节名顺序，当前必须覆盖 UR3e 六个关节。
- `max_joint_delta_rad`：从当前状态到目标状态的每关节最大允许差值。
- `final_position_tolerance_rad`：执行后 final-target gate 的每关节误差容忍度。
- `final_state_timeout_sec`：执行后等待最终 joint state 收敛的超时。
- `planning_time_sec`：MoveGroup 规划时间。
- `planning_attempts`：MoveGroup 规划尝试次数。
- `max_velocity_scaling`：MoveGroup 速度缩放。
- `max_acceleration_scaling`：MoveGroup 加速度缩放。
- `min_speed_scaling`：真机 speed scaling 最小允许值。
- `require_joint_state_stamp`：是否要求 joint state header stamp 非零。
- `require_remote_control`：真机执行时是否要求 Remote Control。
- `allow_reduced_safety_mode`：是否接受 `REDUCED` safety mode。
- `required_active_controllers`：执行前必须 active 的 controller 列表。
- `targets`：命名目标表。

每个目标的字段：

- `enabled`：是否允许被 service 请求。
- `reviewed_by`：审核来源或 TODO 说明。
- `positions_rad`：按 `joint_names` 顺序排列的关节角，单位 rad。

## 7. 请求处理流程

收到 `ExecuteNamedTarget` 请求后，控制器按以下顺序处理：

1. 检查是否已有请求正在处理。若 busy，返回 `rejected_busy`。
2. 检查 catalog 是否加载成功。若失败，返回 `rejected_catalog`。
3. 检查 `target_name` 是否存在。若不存在，返回 `rejected_unknown_target`。
4. 检查目标是否 `enabled=true`。若禁用，返回 `rejected_disabled_target`。
5. 若请求 `execute=true`，检查节点参数 `allow_execution` 是否为 true。若不允许执行，返回 `rejected_execution_disabled`。
6. 等待并读取当前 joint state。
7. 检查 joint state 新鲜度、真机模式时间戳要求和关节名完整性。失败时返回 `rejected_joint_state`。
8. 检查当前关节值到目标关节值的每关节 delta。超限时返回 `rejected_delta`。
9. 创建或复用 `MoveGroupInterface`。
10. 设置当前状态为 planning start state。
11. 设置 planning time、attempts、velocity scaling、acceleration scaling。
12. 设置命名目标对应的 joint value map。若 MoveGroup 拒绝目标，返回 `rejected_move_group_target`。
13. 调用 MoveGroup `plan()`。
14. 若规划失败，返回 `planning_failed`，此时 `accepted=true`、`planned=false`。
15. 若 `execute=false`，返回 `planned`，不执行轨迹。
16. 若 `execute=true` 且 `runtime_mode=real`，执行真机门闩。
17. 调用 MoveGroup `execute(plan)`。
18. 若 execute 失败，返回 `execution_failed`。
19. 执行后等待 final-target gate。
20. 若最终误差未在超时内进入容忍度，返回 `final_gate_failed`。
21. 全部通过后返回 `executed`。

## 8. 真机执行门闩

`runtime_mode=real` 且请求 `execute=true` 时，控制器在规划成功后、执行前额外检查：

- `human_confirmation` 必须等于 `human_confirmation_token`。
- Dashboard robot mode 必须为 `RUNNING`。
- Dashboard safety mode 必须为 `NORMAL`，或在 catalog 中明确允许 `REDUCED`。
- External Control program 必须正在运行。
- 若 `require_remote_control=true`，Dashboard 必须报告 Remote Control。
- `required_active_controllers` 中列出的 controller 必须存在且 state 为 `active`。
- 必须收到 speed scaling sample。
- speed scaling sample 必须足够新。
- speed scaling 当前值必须大于等于 `min_speed_scaling`。

真机门闩失败时，控制器返回：

```text
accepted=true
planned=true
executed=false
status=rejected_real_gate
```

这表示目标和规划本身通过，但真机执行条件不满足。

## 9. 安全策略

当前控制器采用保守的单目标执行策略：

- 默认 launch `execute=false`，优先 plan-only。
- 真机 catalog 目标默认禁用。
- 真机执行必须显式传入确认 token。
- 真机执行前检查 Dashboard、controller、External Control、joint state 和 speed scaling。
- 不从代码中恢复安全停机或重新启动机器人。
- 不自动处理 protective stop、emergency stop、brake release 或程序重载。
- 不对失败请求做自动重试。
- 不提供长时间连续速度控制。

人工职责：

- 审核 `real` catalog 中每个目标的 `positions_rad`。
- 决定每个真机目标何时可以 `enabled=true`。
- 根据现场风险确认 `max_joint_delta_rad`、速度缩放和加速度缩放。
- 决定是否接受 `REDUCED` safety mode，并在文档或 commit 中记录理由。
- 每次真实执行前确认机器人工作空间清空、示教器和急停可达、速度低且任务单一。

## 10. 典型使用

构建：

```bash
cd /home/minzi/ros2_lab/workspaces/ws_ur3e_controller
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

启动仿真 bringup，默认只规划：

```bash
ros2 launch ur3e_named_motion_controller_cpp sim_named_motion_bringup.launch.py \
  runtime_mode:=sim \
  execute:=false \
  launch_rviz:=false
```

调用 plan-only：

```bash
ros2 service call /ur3e_named_motion_controller/execute_named_target \
  ur3e_controller_msgs/srv/ExecuteNamedTarget \
  "{target_name: home, execute: false, human_confirmation: ''}"
```

仿真中允许单次执行：

```bash
ros2 launch ur3e_named_motion_controller_cpp sim_named_motion_bringup.launch.py \
  runtime_mode:=sim \
  execute:=true \
  launch_rviz:=false
```

仿真执行命名目标：

```bash
ros2 service call /ur3e_named_motion_controller/execute_named_target \
  ur3e_controller_msgs/srv/ExecuteNamedTarget \
  "{target_name: ready, execute: true, human_confirmation: ''}"
```

真机执行必须在人工审核后单独进行。最低要求是：

- `runtime_mode:=real`
- launch 参数 `execute:=true`
- 目标在 real catalog 中 `enabled=true`
- service 请求中 `human_confirmation` 等于当前确认 token
- 所有真机门闩通过

## 11. 当前测试覆盖

当前已有 catalog 单元测试：

- 检查 `schema_version == 1`。
- 检查 `sim` 和 `real` runtime mode 均存在。
- 检查每个目标的 `positions_rad` 数量与六关节数量一致。
- 检查每个目标包含 `reviewed_by` 和布尔型 `enabled`。
- 检查 real targets 在人工审核前保持禁用，并保留 `TODO(human)`。

建议继续补充的节点级测试：

- 未知 target 拒绝。
- disabled target 拒绝。
- `execute=true` 但 launch `execute=false` 时拒绝。
- `execute=false` 只规划不执行。
- busy 状态拒绝第二个请求。
- joint state 缺失时拒绝。
- joint state 关节名不完整时拒绝。
- joint state 过旧时拒绝。
- delta 超限时拒绝。
- 真机模式缺少 confirmation token 时拒绝。
- Dashboard、External Control、controller、speed scaling 任一门闩失败时拒绝。

## 12. 验收标准

仿真验收：

- `colcon build` 通过。
- `colcon test --packages-select ur3e_named_motion_controller_cpp` 通过。
- fake hardware 下 `home`、`ready` 的 plan-only 请求返回 `status=planned`。
- fake hardware 下单次 execute 返回 `status=executed`。
- 执行后 final-target gate 通过。

真机前置验收：

- real catalog 的目标仍为禁用时，请求必须返回 `rejected_disabled_target`。
- 未提供正确 `human_confirmation` 时，真实执行必须返回 `rejected_real_gate`。
- Dashboard robot mode 非 `RUNNING` 时必须拒绝。
- safety mode 非 `NORMAL` 且未允许 `REDUCED` 时必须拒绝。
- External Control 未运行时必须拒绝。
- 必需 controller 缺失或非 active 时必须拒绝。
- speed scaling 缺失、过旧或低于阈值时必须拒绝。

真机执行验收：

- 只允许低速、单目标、人工确认后的命名点。
- 现场人员已复核目标关节值、机器人姿态、周围空间和急停可达性。
- 每次执行前先做 plan-only。
- 每次真实执行只发送一个命名目标请求。

## 13. 后续 TODO

- TODO(human)：审核 `real` catalog 中 `home`、`ready` 的真实关节值。
- TODO(human)：确认真机 `max_joint_delta_rad` 是否继续使用 `0.10` rad。
- TODO(human)：确认真机速度缩放、加速度缩放是否足够保守。
- TODO(human)：决定是否接受 `REDUCED` safety mode。
- TODO(dev)：补充节点级 service 测试，覆盖拒绝路径和 plan-only 路径。
- TODO(dev)：为真机门闩增加 mock service 测试。
- TODO(dev)：考虑将更多内部参数外显到 launch 文件，便于实验记录。
- TODO(dev)：在 README 中加入快速启动入口，并从 README 链接本文档。
