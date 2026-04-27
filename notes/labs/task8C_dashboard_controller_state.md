# Task 8C：Dashboard、controller 与 External Control 状态验证

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[ ] 待填写`

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
- robot mode 服务名：`【请填写】`
- robot mode 查询结果：`【请填写】`
- safety mode 服务名：`【请填写】`
- safety mode 查询结果：`【请填写】`
- program state / running 服务名：`【请填写】`
- program state / running 查询结果：`【请填写】`
- Remote Control 相关状态：`【请填写】`
- 你对当前 Dashboard 状态的解释：`【请填写】`

## 6. Controller 与状态流记录
- `joint_state_broadcaster` 状态：`【请填写】`
- `scaled_joint_trajectory_controller` 状态：`【请填写】`
- 其他相关 controller：`【请填写】`
- `/joint_states` 是否新鲜：`【请填写】`
- Action server 是否存在：`【请填写】`
- External Control 是否运行：`【请填写】`

## 7. 状态门闩矩阵

| 类别 | 检查项 | 当前结果 | pass/warn/block | 说明 |
|---|---|---|---|---|
| Dashboard | robot mode | `【请填写】` | `【请填写】` | `【请填写】` |
| Dashboard | safety mode | `【请填写】` | `【请填写】` | `【请填写】` |
| Program | External Control | `【请填写】` | `【请填写】` | `【请填写】` |
| Controller | joint state broadcaster | `【请填写】` | `【请填写】` | `【请填写】` |
| Controller | trajectory controller | `【请填写】` | `【请填写】` | `【请填写】` |
| State | `/joint_states` | `【请填写】` | `【请填写】` | `【请填写】` |

## 8. 你需要完成的判断
- 本轮状态门闩是否通过：`【请填写：通过 / 不通过】`
- 如果不通过，阻断项：`【请填写】`
- 如果通过，进入 8D 前还需人工确认什么：`【请填写】`

## 9. 完成标准
- Dashboard、controller、External Control 状态都已记录。
- 状态门闩有明确 pass/warn/block 结论。
- 你能解释 controller active 为什么不是允许运动的充分条件。

## 10. 完成记录
- 日期：`【请填写】`
- 最终结论：`【请填写】`
- 下一步：`【请填写】`
