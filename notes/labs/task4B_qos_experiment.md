# Task4B QoS 对比实验记录（4B）

## 1. 实验目标
- 通过 A/B/C/D 四组最小矩阵，理解 ROS 2 QoS 对通信行为的影响。
- 能基于观测结果解释：为什么 C 组不兼容、为什么 A/B/D 在当前负载下差异不大。
- 形成“场景 -> QoS 选择”文字结论，作为后续机械臂控制链路依据。

## 2. 本次任务的关键学习点（先写文字，再贴数据）
- QoS 是通信契约，不只是“参数”：发布端与订阅端必须在关键策略上兼容。
- `reliable` 与 `best_effort` 的关系：订阅端可靠性要求不能高于发布端提供能力。
- `keep_last + depth` 影响缓存与拥塞时的可观测行为。
- 统计口径要先约定：`loss_rate_percent`、`latency_p50/p95/max`、窗口大小。

`TODO(human): 用 4-8 句话写下你自己的理解（不要抄命令输出，重点写“为什么”）`

## 3. 实验环境

| 项目 | 内容 |
|---|---|
| 日期 | `TODO(human)` |
| 机器/系统 | `TODO(human)` |
| ROS 2 版本 | `TODO(human)` |
| 工作区 | `/home/minzi/ros2_lab/workspaces/ws_tutorials` |
| 实验包 | `ur3_qos_lab_py` |
| 订阅统计实现 | `qos_subscriber_node.py` |
| 发布速率 | `20 Hz`（本轮） |
| 日志周期 | `2 s`（本轮） |

## 4. 实验设计

### 4.1 固定变量
- 单机环境，同一 Topic：`/ur3/qos_lab`
- 消息类型：`std_msgs/msg/String`
- 负载与采样窗口：`TODO(human)`
- 观察时长：每组至少 `60s`（可补充本次实际时长）

### 4.2 变化变量
- `pub_reliability`
- `sub_reliability`
- `pub_depth / sub_depth`

### 4.3 指标定义（统一口径）
- 连接是否建立：是否出现 QoS 不兼容告警。
- 接收计数：`rx_count`。
- 估算丢包率：`loss_rate_percent`。
- 时延统计：`latency_p50_ms / latency_p95_ms / latency_max_ms`。

## 5. 实验矩阵与预期

| 组别 | Publisher QoS | Subscriber QoS | 预期 |
|---|---|---|---|
| A | reliable, keep_last, depth=10 | reliable, keep_last, depth=10 | 正常通信 |
| B | reliable, keep_last, depth=10 | best_effort, keep_last, depth=1 | 正常通信 |
| C | best_effort, keep_last, depth=1 | reliable, keep_last, depth=10 | 可靠性不兼容，无法接收 |
| D | best_effort, keep_last, depth=1 | best_effort, keep_last, depth=1 | 正常通信 |

## 6. 执行过程（命令留档）

### 6.1 通用命令模板
```bash
cd /home/minzi/ros2_lab/workspaces/ws_tutorials
source install/setup.bash

ros2 launch ur3_qos_lab_py qos_lab_pair.launch.py \
  publish_rate_hz:=20.0 \
  report_period_sec:=2.0 \
  pub_reliability:=<reliable|best_effort> \
  pub_history:=keep_last \
  pub_depth:=<10|1> \
  sub_reliability:=<reliable|best_effort> \
  sub_history:=keep_last \
  sub_depth:=<10|1>
```

### 6.2 本次实际执行命令
- A 组：`TODO(human)`
- B 组：`TODO(human)`
- C 组：`TODO(human)`
- D 组：`TODO(human)`

## 7. 原始观测结果（先填表，再分析）

### 7.1 汇总表

| 组别 | 是否建立通信 | 末段 rx_count | 末段 loss_rate_percent | latency_p50_ms | latency_p95_ms | latency_max_ms | 关键日志/告警 |
|---|---|---:|---:|---:|---:|---:|---|
| A | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` |
| B | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` |
| C | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` |
| D | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` | `TODO(human)` |

### 7.2 每组摘录（保留关键 2-4 行日志）
- A 组：
  - `TODO(human)`
- B 组：
  - `TODO(human)`
- C 组：
  - `TODO(human)`
- D 组：
  - `TODO(human)`

## 8. 对比分析

### 8.1 A / B / D 对比
- 相同点：`TODO(human)`
- 差异点：`TODO(human)`
- 为什么在当前负载下差异不显著：`TODO(human)`

### 8.2 C 组失配分析
- 不兼容触发点：`TODO(human)`
- 现象：`TODO(human)`
- 根因（QoS 契约角度）：`TODO(human)`

## 9. 结论（可直接用于复述）
- 结论 1：`TODO(human)`
- 结论 2：`TODO(human)`
- 结论 3：`TODO(human)`

### 9.1 场景化建议（面向机械臂/手术机器人方向）
- 传感流（高频、可容忍少量丢失）：`TODO(human)`
- 控制指令/关键状态（低容错）：`TODO(human)`

## 10. 踩坑与排查
- 坑 1：`TODO(human)`
  - 现象：`TODO(human)`
  - 根因：`TODO(human)`
  - 修复：`TODO(human)`
- 坑 2：`TODO(human)`
  - 现象：`TODO(human)`
  - 根因：`TODO(human)`
  - 修复：`TODO(human)`

## 11. 个人复盘（学习增量）
- 我之前不知道：`TODO(human)`
- 我现在能解释：`TODO(human)`
- 我还不确定：`TODO(human)`
- 我下一步想验证：`TODO(human)`

## 12. 下一步实验计划
- [ ] 提高 `publish_rate_hz`（如 200/500）复跑 A/B/D
- [ ] 提高 `payload_size_bytes`（如 4096/16384）复跑 A/B/D
- [ ] 补一组跨进程/负载干扰实验
- [ ] 将结论同步到阶段复盘文档

`TODO(human): 选 1-2 项作为 4B 的扩展实验，并写下预期现象`
