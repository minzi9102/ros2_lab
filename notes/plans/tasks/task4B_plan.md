# 任务 4B 规划文档：QoS 对比实验（实践版）

## 1. 任务目标
- 通过可复现实验量化 QoS 对通信行为的影响（连接、接收率、时延、稳定性）。
- 建立“场景 -> QoS 选择”依据，避免无差别使用默认 QoS。

## 2. 任务范围
- 包含：`reliable` / `best_effort`、`keep_last` / `keep_all`、`depth` 的最小对比。
- 包含：本机单机实验、固定话题负载下的对比记录。
- 不包含：DDS 厂商调优、跨网段复杂压测、实时内核优化。

## 3. 前置条件
- 已完成任务 4A，并能独立执行 `colcon build` 与 `source install/setup.bash`。
- 可运行现有 JointState 链路，熟悉 `ros2 topic info/echo/hz` 基础命令。
- 当前任务只做 4B，不与 4C/4D 并行。

## 4. 实施步骤（按 3 天节奏）
1. Day 1（实验设计与基线）
- 固定实验变量：同机、同话题、固定发布频率与载荷。
- 设计最小矩阵（至少 4 组）：
  - A：pub `reliable depth=10` / sub `reliable depth=10`
  - B：pub `reliable depth=10` / sub `best_effort depth=1`
  - C：pub `best_effort depth=1` / sub `reliable depth=10`
  - D：pub `best_effort depth=1` / sub `best_effort depth=1`
- 约定评价指标：连接是否建立、接收率、估算丢包率、时延 p95。

2. Day 2（脚手架与关键实现）
- 创建包：`workspaces/ws_tutorials/src/ur3_qos_lab_py`。
- 完成参数化发布/订阅脚手架：`reliability/history/depth/rate/payload`。
- 人类完成关键实现：订阅侧统计逻辑（丢包估算、时延统计、窗口策略）。

3. Day 3（执行与总结）
- 每组实验至少运行 60s，记录命令、日志、结果表格。
- 汇总 QoS 选择建议：
  - 传感数据（高频、可丢少量）
  - 控制指令/关键状态（低容错）
- 将结论写入实验文档，并同步风险与下一步。

## 5. 人机分工（learn mode）
- 智能体负责：计划落盘、包脚手架、参数接口、构建与集成检查。
- 人类负责：统计逻辑实现与 QoS 取舍判断（本任务核心学习点）。
- 约束：在人类完成关键实现前，智能体不直接给出完整统计答案。

## 6. 交付物
- 代码：`ur3_qos_lab_py` 最小可运行脚手架（发布者、订阅者、launch）。
- 文档：`notes/labs/task4B_qos_experiment.md`（含矩阵、命令、数据、结论）。
- 证据：至少 4 组实验原始日志片段与结果表。

## 7. 验收标准
- 能稳定复现实验矩阵 A-D，且每组均有记录。
- 能说明至少两类场景的 QoS 选择依据及代价。
- 实验文档包含：复现步骤、关键命令、观测结果、结论与风险。

## 8. 风险与回退
- 风险：实验变量过多导致结论失真。
- 回退：先固定负载与话题，只比较单一参数（如 reliability）。
- 风险：统计窗口过短导致抖动误判。
- 回退：统一窗口长度并重复实验取中位结论。

## 9. 完成记录
- 状态：`[ ] 未开始` `[#] 进行中` `[x] 已完成`
- 日期：
- 备注：
