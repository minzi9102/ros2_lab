# Task 8C：Dashboard、controller 与 External Control 状态验证

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[x] 已完成`

## 1. 目标
- 建立进入动作任务前的状态门闩。
- 查询 Dashboard、controller、External Control 和 `/joint_states` 状态。
- 输出 `pass / warn / block` 状态矩阵。

## 2. 应用包与文件位置
- 骨架包：`workspaces/ws_stage4/src/ur3_real_bringup_lab`
- launch：`launch/task8C_state_check.launch.py`
- 脚本：`scripts/check_real_robot_state.py`
- 任务计划：`notes/plans/tasks/task8C_plan.md`

## 3. 当前准备情况
- 已准备：
  - 状态检查脚本骨架；
  - 状态矩阵模板。
- 待你完成：
  - 根据现场服务名查询 Dashboard；
  - 填写状态矩阵；
  - 判断是否允许进入 8D。

## 4. 执行命令
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_real_bringup_lab
source install/setup.bash

ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py

# 动作前门闩演示：只检查，不自动激活 controller
ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py \
  require_trajectory_controller_active:=true

ros2 service list | grep dashboard
ros2 control list_controllers
ros2 action list | grep follow_joint_trajectory
```

Dashboard 查询命令按现场服务名填写：

```bash
ros2 service call <robot_mode_service> <service_type> "{}"
ros2 service call <safety_mode_service> <service_type> "{}"
ros2 service call <program_state_service> <service_type> "{}"
```

## 5. Dashboard 状态记录
- robot mode 服务名：`/dashboard_client/get_robot_mode`
- robot mode 查询结果：`Robotmode: RUNNING`
- safety mode 服务名：`/dashboard_client/get_safety_mode`
- safety mode 查询结果：`Safetymode: NORMAL`
- program state / running 服务名：`/dashboard_client/program_running`
- program state / running 查询结果：`Program running: true`
- Remote Control 相关状态：`remote_control=false`
- 你对当前 Dashboard 状态的解释：`机器人和 External Control 处于可观察、可解释状态；但当前不是 Remote Control，说明程序由示教器侧启动，不能假设 ROS 端可远程 load/play 程序。`

## 6. Controller 与状态流记录
- `joint_state_broadcaster` 状态：`active`
- `scaled_joint_trajectory_controller` 状态：`inactive`
- 其他相关 controller：`io_and_status_controller、speed_scaling_state_broadcaster、force_torque_sensor_broadcaster、tcp_pose_broadcaster、ur_configuration_controller 为 active；joint_trajectory_controller、forward_*、force_mode、freedrive、tool_contact、passthrough 为 inactive`
- `/joint_states` 是否新鲜：`是；只读检查约 464.9 Hz，动作前门闩演示约 503.3 Hz`
- Action server 是否存在：`存在 /scaled_joint_trajectory_controller/follow_joint_trajectory 和 /trajectory_until_node/execute；本轮未调用`
- External Control 是否运行：`是，program_running=true，speed_scaling=100.0`

## 6.1 真机 calibration 记录
- 提取状态：`已提取`
- 主用文件：`workspaces/ws_stage4/src/ur3_real_bringup_lab/config/ur3e_real_calibration.yaml`
- 仓库归档副本：`docs/calibration/ur3e_real_calibration.yaml`
- calibration hash：`calib_9781467669625414396`
- 文件用途：`替代 ur_description 默认 kinematics 参数，使 robot_description、TF、MoveIt 与 TCP 位姿计算使用当前这台 UR3e 的真实标定参数。`
- 当前处理状态：`已记录并归档；尚未接入 task8B/task8C bringup wrapper。后续进入 8D 前，需要通过 description launch/wrapper 将该文件作为 kinematics_params_file 传入。`

## 7. 状态门闩矩阵

| 类别 | 检查项 | 当前结果 | pass/warn/block | 说明 |
|---|---|---|---|---|
| Dashboard | robot mode | `RUNNING` | `pass` | `机器人处于运行模式` |
| Dashboard | safety mode | `NORMAL` | `pass` | `未处于 protective stop / safeguard stop / fault` |
| Program | External Control | `program_running=true` | `pass` | `External Control 正在运行` |
| Dashboard | Remote Control | `remote_control=false` | `warn` | `示教器启动可接受；远程 Dashboard load/play 不可假设` |
| Controller | joint state broadcaster | `active` | `pass` | `状态流 broadcaster active` |
| Controller | trajectory controller | `inactive` | `warn / 动作前 block` | `8C 只读阶段故意保持 inactive；当 require_trajectory_controller_active:=true 时脚本正确给出 block` |
| State | `/joint_states` | `约 465-503 Hz` | `pass` | `关节名匹配 UR3e 六轴` |
| State | speed scaling | `100.0` | `pass` | `External Control 链路处于可解释状态` |

### 自动检查结果
- 只读状态门闩命令：`ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py`
- 只读状态门闩结论：`WARN`
- 只读 warning：`remote_control=false；scaled_joint_trajectory_controller inactive`
- 动作前门闩命令：`ros2 launch ur3_real_bringup_lab task8C_state_check.launch.py require_trajectory_controller_active:=true`
- 动作前门闩结论：`BLOCK`
- 动作前阻断项：`scaled_joint_trajectory_controller inactive`
- 脚本安全边界：`只读服务/topic/controller 查询；不解锁 protective stop，不 restart safety，不 power/brake，不 switch controller，不发送 motion command`

## 8. 你需要完成的判断
- 本轮状态门闩是否通过：`只读 8C 通过但带 warning；动作前门闩不通过`
- 如果不通过，阻断项：`进入动作任务前，scaled_joint_trajectory_controller 仍为 inactive；此外 calibration 文件已提取但尚未接入 bringup，FIFO/实时调度 warning、overrun warning 也需要作为 8D 风险前置记录`
- 如果通过，进入 8D 前还需人工确认什么：`是否允许在 8D 中显式激活轨迹控制器；是否接受当前 remote_control=false 的现场操作模式；是否先把真机 calibration 文件接入 description launch/wrapper；是否继续在非实时内核/powersave 下仅做极低风险动作测试`

## 9. 完成标准
- Dashboard、controller、External Control 状态都已记录。
- 状态门闩有明确 pass/warn/block 结论。
- 你能解释 controller active 为什么不是允许运动的充分条件。

## 10. 完成记录
- 日期：`2026-04-28`
- 最终结论：`8C 完成。Dashboard、External Control、状态流和 controller manager 均可只读查询；真机 calibration 已提取并归档；当前状态允许继续做 8D 前置设计与人工评审，但不允许直接进入运动执行。`
- 下一步：`进入 8D 前，先把 calibration 接入 bringup wrapper，并决定实时调度、Remote Control 与 trajectory controller 激活策略；若继续 8D，必须保持 guarded home-ready motion 的最小动作范围。`
