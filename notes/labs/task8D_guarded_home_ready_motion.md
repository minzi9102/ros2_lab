# Task 8D：低速小范围 home / ready 关节动作

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[ ] 待填写`

## 1. 目标
- 在 8A-8C 通过后，执行第一组低速、小范围、人工确认过的真机关节空间动作。
- 只做 home / ready 点，不做复杂规划、笛卡尔路径或 Servo。
- 每次动作都记录目标、当前状态、delta、状态门闩、人工确认和执行结果。

## 2. 应用包与文件位置
- 骨架包：`workspaces/ws_stage4/src/ur3_real_guarded_motion_lab_cpp`
- config：`config/task8D_guarded_targets.yaml`
- launch：`launch/task8D_guarded_home_ready.launch.py`
- 节点：`src/guarded_joint_motion_node.cpp`
- 任务计划：`notes/plans/tasks/task8D_plan.md`

## 3. 当前准备情况
- 已准备：
  - 受限动作 C++ 节点骨架；
  - 默认 dry-run launch；
  - home / ready 点配置模板；
  - 本记录模板。
- 待你完成：
  - 审核并填写 home / ready 关节目标；
  - 判断速度、加速度、delta 上限；
  - 现场确认是否允许执行。

## 4. 构建与 dry-run
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ur3_real_guarded_motion_lab_cpp
source install/setup.bash

ros2 launch ur3_real_guarded_motion_lab_cpp task8D_guarded_home_ready.launch.py \
  execute:=false \
  target_name:=ready
```

### dry-run 记录
- 是否构建通过：`【请填写】`
- dry-run 是否运行：`【请填写】`
- 是否确认未发送 goal：`【请填写】`
- 输出的关键日志：`【请填写】`

## 5. 点位审核填写区
- home 目标来源：`【请填写】`
- home 目标关节值（rad）：`【请填写】`
- ready 目标来源：`【请填写】`
- ready 目标关节值（rad）：`【请填写】`
- 点位审核人：`【请填写】`
- 最大单关节 delta：`【请填写】`
- 速度限制：`【请填写】`
- 加速度限制：`【请填写】`
- 最短执行时间：`【请填写】`
- 你选择这些值的理由：`【请填写】`

## 6. 单次执行前记录

| 项目 | 内容 |
|---|---|
| 时间 | `【请填写】` |
| 操作者 | `【请填写】` |
| 旁站确认 | `【请填写】` |
| 目标名 | `【请填写：home / ready】` |
| 当前 joint state | `【请填写】` |
| 目标 joint state | `【请填写】` |
| 每关节 delta | `【请填写】` |
| 8C 状态门闩结果 | `【请填写】` |
| controller 状态 | `【请填写】` |
| robot / safety / program 状态 | `【请填写】` |
| 人工确认 | `【请填写：yes / no】` |

## 7. 真实执行记录
> 只有 8C 通过、点位审核完成、现场人工确认后，才允许填写并执行本节。

执行命令（执行前再次确认）：

```bash
ros2 launch ur3_real_guarded_motion_lab_cpp task8D_guarded_home_ready.launch.py \
  execute:=true \
  require_confirmation:=true \
  human_confirmation:=I_CONFIRM_REAL_ROBOT_MOTION \
  target_name:=ready
```

- 是否真实执行：`【请填写：是 / 否】`
- goal 是否发送：`【请填写】`
- goal 是否 accepted：`【请填写】`
- result：`【请填写】`
- 最终 joint state：`【请填写】`
- 与目标误差：`【请填写】`
- 执行中是否出现异常：`【请填写】`
- 你的观察：`【请填写】`

## 8. 你需要完成的判断
- 这次动作是否足够保守：`【请填写】`
- 是否允许进行第二个目标：`【请填写：允许 / 不允许】`
- 是否需要进入 8E 复盘异常：`【请填写】`

## 9. 完成标准
- dry-run 可运行且不发送 goal。
- 点位、delta、速度、加速度经过人工审核。
- 如真实执行，每次动作都有完整日志。

## 10. 完成记录
- 日期：`【请填写】`
- 最终结论：`【请填写】`
- 下一步：`【请填写】`
