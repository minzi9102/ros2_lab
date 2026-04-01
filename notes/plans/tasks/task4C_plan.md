# 任务 4C 规划文档：tf2 主动查询（实践版）

## 1. 任务目标
- 在现有 UR3 简化模型链路上，完成 `base_link -> tool0` 的主动 TF 查询最小闭环。
- 用可复现实验掌握 tf2 三类高频问题：`LookupException`、`ConnectivityException`、`ExtrapolationException`（含超时语义）。
- 形成“查询时间语义 + 异常处理策略”文字结论，为后续 4E/4F 控制器联调做准备。

## 2. 当前基线（来自仓库现状）
- 已具备可运行链路：`ur3_joint_state_publisher_py/launch/ur3_simplified_rviz.launch.py`。
- 现有 launch 已将 TF 隔离到 `/tf_demo` 与 `/tf_static_demo`，并发布 `/joint_states_demo`。
- 当前 URDF 已包含目标 frame：`base_link`（根）与 `tool0`（末端）。
- 4A、4B 已完成，本任务只推进 4C，不并行处理 4D+。

## 3. 任务范围
- 包含：
  - tf2 `Buffer + TransformListener` 最小节点；
  - 定时查询与位姿输出（平移 + 四元数）；
  - 超时、外推、frame 不存在等异常分支；
  - 与 RViz/`tf2_echo` 对照验证。
- 不包含：
  - 复杂标定链与多机器人命名空间冲突；
  - 运动学求解、控制器执行；
  - 真实机械臂硬件时钟同步策略。

## 4. 前置条件
- 能独立运行以下命令并看到 UR3 模型：
```bash
cd /home/minzi/ros2_lab/workspaces/ws_tutorials
source install/setup.bash
ros2 launch ur3_joint_state_publisher_py ur3_simplified_rviz.launch.py
```
- 能通过 `ros2 topic list` 看到 `/tf_demo`、`/tf_static_demo`、`/joint_states_demo`。
- 当前任务仅做 4C；如发现跨任务改动需求，先拆单再推进。

## 5. 实施步骤（按 3 天节奏）
1. Day 1（链路核对 + 查询接口设计）
- 固定查询目标：`target_frame=tool0`，`source_frame=base_link`。
- 明确参数接口：`target_frame/source_frame/query_rate_hz/timeout_sec/use_sim_time/tf_topic/tf_static_topic`。
- 验证基线链路：`tf2_echo` 与 RViz 显示一致。

2. Day 2（脚手架 + 关键实现）
- 新建最小包（建议）：`workspaces/ws_tutorials/src/ur3_tf_lookup_py`。
- 完成节点脚手架：参数声明、定时器、日志格式、launch 接线。
- 人类完成关键实现（本任务学习重点）：
  - 查询时间语义选择：`Time()`（latest）或指定时间；
  - 异常分支策略：重试节奏、告警等级、何时判定失败；
  - 输出字段取舍：是否同时打印欧拉角与四元数。

3. Day 3（实验矩阵 + 文档沉淀）
- 至少跑 3 组场景（每组建议 60s）：
  - A 正常场景：`base_link -> tool0`；
  - B 不存在 frame：如 `target_frame=toolX`；
  - C 时间相关异常：缩短 timeout 或引入查询时间偏移触发外推。
- 记录每组命令、关键日志、现象与修复动作。
- 将结论写入：`notes/labs/task4C_tf2_lookup.md`。

## 6. 人机分工（learn mode）
- 智能体负责：
  - 计划落盘、包骨架、参数与 launch 通路、构建和集成检查；
  - 提供实验矩阵模板与验收口径。
- 人类负责（核心学习点）：
  - tf2 查询核心逻辑与异常处理决策；
  - 时间语义选择依据（latest/指定时刻）；
  - 结果复盘与取舍说明。
- 约束：
  - 在人类完成核心实现前，智能体不直接给出完整关键实现答案。

## 7. 实验记录模板（4C 最小矩阵）

| 组别 | 输入参数 | 预期现象 | 必记日志 |
|---|---|---|---|
| A 正常 | `base_link -> tool0`，`timeout=0.2s` | 持续输出稳定 transform | 位置/姿态、查询耗时 |
| B frame不存在 | `target_frame=toolX` | 报 frame 不存在异常 | 异常类型、重试节奏 |
| C 时间异常 | 缩短 timeout 或制造时间偏移 | 出现超时/外推异常 | 异常次数、恢复策略 |

## 8. 交付物
- 代码：
  - tf2 查询最小节点（建议包：`ur3_tf_lookup_py`）；
  - 对应 launch（与 `/tf_demo`、`/tf_static_demo` 对齐）。
- 文档：
  - 实验记录：`notes/labs/task4C_tf2_lookup.md`（命令、日志、表格、结论）；
  - 若参数策略有变更，补充到任务备注。
- 证据：
  - 至少 3 组实验原始日志片段；
  - 至少 1 组 RViz 与命令行对照截图/文字记录。

## 9. 验收标准
- 成功场景下，查询输出连续稳定且与 RViz 观测一致。
- 失败场景下，日志能区分至少两类异常，且可定位修复方向。
- 能口述：
  - 为什么 `base_link -> tool0` 查询对控制器调试有价值；
  - “latest 查询”与“指定时刻查询”的代价差异；
  - 超时与重试策略对系统稳定性的影响。

## 10. 风险与回退
- 风险 1：TF 话题 remap 不一致导致查不到数据。
  - 回退：先用默认 `/tf` 验证节点，再恢复 `/tf_demo` 隔离。
- 风险 2：frame 命名与 URDF 不一致（如大小写、下划线）。
  - 回退：先 `view_frames` 导出 TF 树，按真实 frame 名称修正参数。
- 风险 3：时间戳语义理解不足，误判外推问题。
  - 回退：先固定 latest 查询跑通，再引入指定时间查询。

## 11. 完成记录
- 状态：`[ ] 未开始` `[#] 进行中` `[x] 已完成`
- 日期：
- 备注：
