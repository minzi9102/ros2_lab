# UR3 ROS 2 控制链路学习笔记（任务 3）

## 使用方式
1. 先按顺序填完每个 `TODO(human)`。
2. 每段尽量用“我自己的话”描述，不要只贴命令。
3. 完成后把“验收清单”逐条打勾，再交给 Codex review。

---

## 0. 基本信息
- 日期：`TODO(human): 填写日期`
- 机器人型号：`TODO(human): ur3 或 ur3e`
- 软件栈：Ubuntu 24.04 + ROS 2 Jazzy + MoveIt 2 + UR ROS 2 Driver
- 本次验证环境：`TODO(human): URSim / 真机`

---

## 1. 总览链路（每层一句职责）

链路：`robot_description -> ros2_control -> ur_robot_driver -> controllers -> MoveIt 2 -> 自定义控制节点`

| 组件 | 它的职责（1-2 句） | 它接收什么 | 它输出什么 |
|---|---|---|---|
| robot_description | `TODO(human)` | `TODO(human)` | `TODO(human)` |
| ros2_control | `TODO(human)` | `TODO(human)` | `TODO(human)` |
| ur_robot_driver | `TODO(human)` | `TODO(human)` | `TODO(human)` |
| controllers | `TODO(human)` | `TODO(human)` | `TODO(human)` |
| MoveIt 2 | `TODO(human)` | `TODO(human)` | `TODO(human)` |
| 自定义控制节点 | `TODO(human)` | `TODO(human)` | `TODO(human)` |

`TODO(human): 用 3-5 句话总结“谁负责规划、谁负责执行、谁负责反馈状态”。`

---

## 2. 命令流（从规划到机械臂执行）

请用“谁 -> 谁 -> 用什么接口”的格式填写。

1. `TODO(human): 例如 RViz/MoveIt 发起规划请求给 move_group`
2. `TODO(human)`
3. `TODO(human)`
4. `TODO(human)`
5. `TODO(human)`
6. `TODO(human): 最终到 UR3 执行`

`TODO(human): 在这里补一句“follow_joint_trajectory 在命令流中的作用”。`

---

## 3. 状态流（从机器人回到上层）

1. `TODO(human): 机器人状态如何被 driver 采集`
2. `TODO(human): joint_state_broadcaster 做了什么`
3. `TODO(human): /joint_states 被哪些模块消费`
4. `TODO(human): MoveIt 如何用状态更新规划场景`
5. `TODO(human): RViz 如何体现状态变化`

`TODO(human): 对比命令流与状态流，写出两者的方向和耦合点。`

---

## 4. 控制器选择与原因

- 当前激活控制器：`scaled_joint_trajectory_controller`
- 备选控制器：`joint_trajectory_controller` / `passthrough_trajectory_controller`

`TODO(human): 解释为什么当前实验默认使用 scaled_joint_trajectory_controller。`

`TODO(human): 写出如果换成 joint_trajectory_controller，行为上可能有什么不同。`

---

## 5. 故障复盘（本次至少 2 个）

### 问题 A
- 现象：`TODO(human)`
- 日志关键信息：`TODO(human)`
- 根因判断：`TODO(human)`
- 修复动作：`TODO(human)`
- 如何预防复发：`TODO(human)`

### 问题 B
- 现象：`TODO(human)`
- 日志关键信息：`TODO(human)`
- 根因判断：`TODO(human)`
- 修复动作：`TODO(human)`
- 如何预防复发：`TODO(human)`

---

## 6. 面向手术机器人开发的 3 个可靠性关注点

1. `TODO(human): 关注点 1（例如延迟/抖动/实时性）`
原因：`TODO(human)`
当前栈里可落地点：`TODO(human)`

2. `TODO(human): 关注点 2（例如安全边界/故障安全）`
原因：`TODO(human)`
当前栈里可落地点：`TODO(human)`

3. `TODO(human): 关注点 3（例如状态一致性/可观测性）`
原因：`TODO(human)`
当前栈里可落地点：`TODO(human)`

---

## 7. 60-90 秒口述稿（验收用）

`TODO(human): 用一段自然语言口述完整链路，要求包含：`
- 从 MoveIt 到控制器再到 UR3 的命令路径
- 从 UR3 回到 /joint_states 再回到 MoveIt 的状态路径
- 你认为最容易出错的一环和原因

---

## 8. 验收清单

- [ ] 我能在不看命令的情况下说出完整链路。
- [ ] 我能解释 `scaled_joint_trajectory_controller` 的作用。
- [ ] 我能根据日志区分“驱动问题”与“工具链命令问题”（如管道命令）。
- [ ] 我能说出至少 3 个面向手术机器人的可靠性关注点。
- [ ] 我的命令流和状态流都写到了 5 步以上。

---

## 9. 下一步（任务 4 入口）

`TODO(human): 我希望先做哪个方向的最小控制节点？`
- 选项 A：只订阅状态并打印关键指标
- 选项 B：发送简单关节轨迹（最小可执行）
- 选项 C：做一个“状态监测 + 安全停止”雏形
