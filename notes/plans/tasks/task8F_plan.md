# 任务 8F 规划文档：执行日志、操作规程与阶段验收

## 1. 任务目标
- 收口阶段 4：整理真机运行日志规范、操作规程、验收记录和边界声明。
- 证明当前能力是“安全实验入口”，而不是完整控制系统。
- 用最小动作完成阶段验收，不扩大到 Servo、遥操作、复杂 MoveIt 规划或自动化任务。

## 2. 当前基线（来自仓库现状）
- 8A-8E 将分别提供 preflight、只读 bringup、状态门闩、受限动作、异常处理矩阵。
- 当前 README 已记录阶段 3 完成态，但尚未收录阶段 4 完成结论。
- 当前仓库已有 `experience/` 经验沉淀方式，可在阶段收口时复用。

## 3. 任务范围（单功能约束）
- 包含：
  - 真机日志目录规范；
  - 单次执行日志字段；
  - 阶段 4 操作规程；
  - 阶段验收记录；
  - 风险与后续任务边界。
- 不包含：
  - 新增运动能力；
  - 扩大到 MoveIt Servo 真机；
  - 自动化连续任务；
  - 生产级控制系统声明。

## 4. 文档与目录设计
- 实验记录：`notes/labs/task8F_real_robot_acceptance.md`
- 建议日志目录：
  ```text
  logs/task8/
  ├── YYYYMMDD-HHMMSS-preflight/
  ├── YYYYMMDD-HHMMSS-readonly-bringup/
  ├── YYYYMMDD-HHMMSS-guarded-motion/
  └── YYYYMMDD-HHMMSS-fault-review/
  ```
- 建议经验沉淀：
  - `experience/sessions/<timestamp>--task8-real-robot.json`
  - 更新 `experience/index.json`

## 5. learn mode 分工
- 智能体负责：
  - 日志模板；
  - 操作规程整理；
  - 阶段验收文档；
  - README / experience 更新建议；
  - 风险与后续任务拆分。
- 人类负责：
  - 确认阶段验收是否真实通过；
  - 判断真机操作规程是否符合现场要求；
  - 标注哪些能力仍不允许复用到无人值守场景；
  - 决定是否进入阶段 5 或继续补阶段 4 安全项。

## 6. 核心学习问题
1. 为什么真机阶段的“日志”是安全能力的一部分，而不是事后附录？
2. 为什么阶段验收要强调边界，不只写成功结果？
3. 哪些代码只是实验入口，哪些模式未来可能提炼成控制系统组件？
4. 阶段 4 后，继续做 Servo / MoveIt 真机规划前还缺什么？

## 7. 实施步骤

### 练习 1：整理日志规范
1. 定义日志目录命名。
2. 定义每次动作必须记录的字段。
3. 将 8A-8E 的记录统一到同一套模板。

### 练习 2：整理操作规程
1. 启动前：现场、网络、示教器、External Control、ROS PC。
2. 执行前：状态门闩、目标检查、人工确认。
3. 执行中：观察项、禁止操作、停止条件。
4. 异常后：停机、记录、人工恢复、复盘。
5. 收尾：停止 driver、保存日志、恢复现场状态。

### 练习 3：完成阶段验收记录
1. 记录真机连接证据。
2. 记录只读状态流证据。
3. 记录一次或少量几次低速 home / ready 动作证据。
4. 记录异常拒绝路径证据。
5. 写清楚当前不支持的能力。

### 练习 4：沉淀经验与后续边界
1. 将阶段 4 的关键经验写入 `experience/`。
2. 如阶段完成，更新 README 的阶段状态。
3. 为后续任务开边界清单，例如真机 MoveIt planning、真机 Servo、操作界面、系统级安全审计。

## 8. 单次动作日志字段

| 字段 | 说明 |
|---|---|
| timestamp | 动作开始时间 |
| operator | 操作者 |
| observer | 旁站或安全确认者 |
| robot_id | 机器人型号 / IP / 控制柜信息 |
| target_name | home / ready / reject-test |
| current_joint_state | 执行前状态 |
| target_joint_state | 目标状态 |
| joint_delta | 每关节 delta |
| controller_state | controller list 摘要 |
| dashboard_state | robot / safety / program 状态 |
| precheck_result | pass / reject |
| execution_result | success / failed / canceled / not-sent |
| final_joint_state | 执行后状态 |
| notes | 异常、人工判断、后续动作 |

## 9. 阶段 4 验收清单

| 验收项 | 证据 | 状态 |
|---|---|---|
| 安全连接 UR3 真机 | 8A/8B 记录 | 待完成 |
| 只读状态流正常 | `/joint_states`、controller、Dashboard 记录 | 待完成 |
| 执行低速 home / ready 点 | 8D 动作日志 | 待完成 |
| 异常时拒绝执行或停机 | 8E 矩阵与日志 | 待完成 |
| 操作规程可复用 | 8F runbook | 待完成 |
| 边界声明清楚 | 阶段总结 | 待完成 |

## 10. 交付物
- 文档：`notes/labs/task8F_real_robot_acceptance.md`
- 可选文档：`notes/runbooks/real_robot_operation.md`
- 可选经验：`experience/sessions/<timestamp>--task8-real-robot.json`
- 可选更新：README 阶段 4 状态。

## 11. 验收标准
- 每次真机动作都有可追踪日志。
- 阶段 4 操作规程覆盖启动前、执行前、执行中、异常后和收尾。
- 阶段验收只使用 8A-8E 已验证能力，不新增运动范围。
- 文档明确说明当前代码仍是学习实验入口，不是生产控制系统。
- 后续任务边界清晰，不把真机 Servo、复杂规划或自动化任务混进阶段 4 收口。

## 12. 风险与回退
- 风险 1：阶段收口时想顺手加入新能力。
  - 回退：新能力单开后续任务，不进入 8F。
- 风险 2：验收记录只写成功，不写边界。
  - 回退：补充限制、失败路径和未验证项。
- 风险 3：日志字段过少，无法复盘。
  - 回退：补录 8A-8E 关键证据，缺失则标为未验收。
