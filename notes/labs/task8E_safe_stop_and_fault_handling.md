# Task 8E：异常场景与安全停机逻辑

## 状态
- `[ ] 未开始`
- `[#] 进行中`
- `[x] 已完成`
- 当前状态：`[ ] 待填写`

## 1. 目标
- 整理并验证异常情况下的拒绝执行、cancel、停止 program 与人工恢复边界。
- 建立异常处理矩阵。
- 明确恢复类操作不进入默认自动化流程。

## 2. 应用包与文件位置
- 骨架包：`workspaces/ws_stage4/src/ur3_real_guarded_motion_lab_cpp`
- launch：`launch/task8E_fault_review.launch.py`
- 任务计划：`notes/plans/tasks/task8E_plan.md`

## 3. 当前准备情况
- 已准备：
  - dry-run 异常复盘 launch；
  - 异常矩阵模板；
  - 本记录模板。
- 待你完成：
  - 判断哪些异常可以现场验证；
  - 记录拒绝执行与人工恢复路径；
  - 明确哪些操作禁止自动化。

## 4. 执行前约束
- 是否完成 8D 最小动作：`【请填写】`
- 是否有现场人工确认：`【请填写】`
- 本轮是否只验证低风险异常：`【请填写】`
- 是否禁止自动 unlock / restart safety：`【请填写：是 / 否】`

## 5. 低风险拒绝路径验证
```bash
cd /home/minzi/ros2_lab/workspaces/ws_stage4
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch ur3_real_guarded_motion_lab_cpp task8E_fault_review.launch.py
```

- 是否运行 dry-run fault review：`【请填写】`
- 是否确认未发送 goal：`【请填写】`
- 触发的拒绝原因：`【请填写】`
- 日志摘要：`【请填写】`

## 6. 异常处理矩阵

| 异常 | 检测阶段 | 本轮是否验证 | 默认动作 | 是否人工确认 | 是否允许自动恢复 | 记录 |
|---|---|---|---|---|---|---|
| 目标越界 | 执行前 | `【请填写】` | 拒绝执行 | 否 | 否 | `【请填写】` |
| delta 过大 | 执行前 | `【请填写】` | 拒绝执行 | 否 | 否 | `【请填写】` |
| `/joint_states` 过期 | 执行前 | `【请填写】` | 拒绝执行 | 否 | 否 | `【请填写】` |
| controller inactive | 执行前 | `【请填写】` | 拒绝执行 | 是 | 否 | `【请填写】` |
| External Control 未运行 | 执行前 | `【请填写】` | 拒绝执行 | 是 | 否 | `【请填写】` |
| Action 执行异常 | 执行中 | `【请填写】` | cancel / 停止观察 | 是 | 否 | `【请填写】` |
| protective stop | 任意阶段 | `【请填写】` | 停止脚本，人工处理 | 是 | 否 | `【请填写】` |
| driver 断连 | 任意阶段 | `【请填写】` | 停止脚本，记录日志 | 是 | 否 | `【请填写】` |

## 7. 人工恢复 runbook 填写区
- protective stop 后由谁判断恢复：`【请填写】`
- safeguard stop 后由谁判断恢复：`【请填写】`
- External Control program 停止后如何处理：`【请填写】`
- 哪些 Dashboard 服务只允许人工调用：`【请填写】`
- 哪些情况必须终止当天实验：`【请填写】`

## 8. 你需要完成的判断
- 本轮异常验证是否足够：`【请填写】`
- 哪些异常不适合实测，只能写 runbook：`【请填写】`
- 8F 收口前还缺哪些证据：`【请填写】`

## 9. 完成标准
- 至少一个执行前拒绝路径有证据。
- 保护停止等高风险恢复不会被默认自动化。
- 每类异常都有处理动作和人工确认边界。

## 10. 完成记录
- 日期：`【请填写】`
- 最终结论：`【请填写】`
- 下一步：`【请填写】`
