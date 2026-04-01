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
- A 组：`timeout 65s ros2 launch ur3_qos_lab_py qos_lab_pair.launch.py   pub_reliability:=reliable pub_history:=keep_last pub_depth:=10   sub_reliability:=reliable sub_history:=keep_last sub_depth:=10   publish_rate_hz:=20.0 report_period_sec:=1.0 2>&1 | tee /tmp/task4b_A.log`
- B 组：`timeout 65s ros2 launch ur3_qos_lab_py qos_lab_pair.launch.py   pub_reliability:=reliable pub_history:=keep_last pub_depth:=10   sub_reliability:=best_effort sub_history:=keep_last sub_depth:=10   publish_rate_hz:=20.0 report_period_sec:=1.0 2>&1 | tee /tmp/task4b_B.log`
- C 组：`timeout 65s ros2 launch ur3_qos_lab_py qos_lab_pair.launch.py   pub_reliability:=best_effort pub_history:=keep_last pub_depth:=10   sub_reliability:=reliable sub_history:=keep_last sub_depth:=10   publish_rate_hz:=20.0 report_period_sec:=1.0 2>&1 | tee /tmp/task4b_C.log`
- D 组：`timeout 65s ros2 launch ur3_qos_lab_py qos_lab_pair.launch.py   pub_reliability:=best_effort pub_history:=keep_last pub_depth:=10   sub_reliability:=best_effort sub_history:=keep_last sub_depth:=10   publish_rate_hz:=20.0 report_period_sec:=1.0 2>&1 | tee /tmp/task4b_D.log`

## 7. 原始观测结果（先填表，再分析）

### 7.1 汇总表（本轮：1000Hz + 64KB + 60s，A/B/D 同口径）

| 组别 | 是否建立通信 | sent_count(末段) | rx_count(末段) | rx/sent | 末段 loss_rate_percent | latency_p50_ms(末段) | latency_p95_ms(末段) | latency_max_ms(末段) | 关键日志/告警 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|
| A | `是` | `63701` | `62726` | `98.47%` | `1.39%` | `0.359` | `10.055` | `11.172` | `无` |
| B | `是` | `63672` | `62823` | `98.67%` | `1.25%` | `0.240` | `2.914` | `10.249` | `无` |
| C | `本轮未执行（仅做 A/B/D 放大差异）` | `-` | `-` | `-` | `-` | `-` | `-` | `-` | `-` |
| D | `是` | `63689` | `61882` | `97.16%` | `1.22%` | `0.229` | `3.319` | `10.291` | `无` |

### 7.2 每组摘录（保留关键 2-4 行日志）
- A 组：
    [qos_subscriber_node-2] [INFO] [1775029204.115407697] [qos_subscriber_node]: rx_count=159 parse_fail=0 latest_latency_ms=0.392 expected_count=158 lost_count=0 loss_rate_percent=0.00% latency_p50_ms=0.318 latency_p95_ms=0.548 latency_max_ms=87.554
    [qos_publisher_node-1] [INFO] [1775029204.117834379] [qos_publisher_node]: sent_count=160 seq=160
    [qos_subscriber_node-2] [INFO] [1775029205.115079746] [qos_subscriber_node]: rx_count=179 parse_fail=0 latest_latency_ms=0.304 expected_count=178 lost_count=0 loss_rate_percent=0.00% latency_p50_ms=0.317 latency_p95_ms=0.548 latency_max_ms=87.554
    [qos_publisher_node-1] [INFO] [1775029205.116469098] [qos_publisher_node]: sent_count=180 seq=180
- B 组：
    [qos_subscriber_node-2] [INFO] [1775029278.807955796] [qos_subscriber_node]: rx_count=159 parse_fail=0 latest_latency_ms=0.361 expected_count=158 lost_count=0 loss_rate_percent=0.00% latency_p50_ms=0.308 latency_p95_ms=0.759 latency_max_ms=3.105
    [qos_publisher_node-1] [INFO] [1775029278.825215101] [qos_publisher_node]: sent_count=160 seq=160
    [qos_subscriber_node-2] [INFO] [1775029279.807862882] [qos_subscriber_node]: rx_count=179 parse_fail=0 latest_latency_ms=0.291 expected_count=178 lost_count=0 loss_rate_percent=0.00% latency_p50_ms=0.304 latency_p95_ms=0.759 latency_max_ms=3.105
    [qos_publisher_node-1] [INFO] [1775029279.824919111] [qos_publisher_node]: sent_count=180 seq=180
- C 组：
    [qos_publisher_node-1] [INFO] [1775029342.232532695] [qos_publisher_node]: sent_count=160 seq=160
    [qos_subscriber_node-2] [INFO] [1775029342.249154353] [qos_subscriber_node]: rx_count=0 parse_fail=0 latest_latency_ms=N/A expected_count=0 lost_count=0 loss_rate_percent=0.00% latency_p50_ms=0.000 latency_p95_ms=0.000 latency_max_ms=0.000
    [qos_publisher_node-1] [INFO] [1775029343.232318763] [qos_publisher_node]: sent_count=180 seq=180
    [qos_subscriber_node-2] [INFO] [1775029343.249406136] [qos_subscriber_node]: rx_count=0 parse_fail=0 latest_latency_ms=N/A expected_count=0 lost_count=0 loss_rate_percent=0.00% latency_p50_ms=0.000 latency_p95_ms=0.000 latency_max_ms=0.000
- D 组：
    [qos_publisher_node-1] [INFO] [1775029395.764999267] [qos_publisher_node]: sent_count=160 seq=160
    [qos_subscriber_node-2] [INFO] [1775029395.781056281] [qos_subscriber_node]: rx_count=160 parse_fail=0 latest_latency_ms=0.300 expected_count=159 lost_count=0 loss_rate_percent=0.00% latency_p50_ms=0.305 latency_p95_ms=0.454 latency_max_ms=4.530
    [qos_publisher_node-1] [INFO] [1775029396.764936413] [qos_publisher_node]: sent_count=180 seq=180
    [qos_subscriber_node-2] [INFO] [1775029396.781077114] [qos_subscriber_node]: rx_count=180 parse_fail=0 latest_latency_ms=0.279 expected_count=179 lost_count=0 loss_rate_percent=0.00% latency_p50_ms=0.302 latency_p95_ms=0.449 latency_max_ms=4.530

## 8. 对比分析

### 8.1 A / B / D 对比（1000Hz + 64KB + 60s）
- 相同点：`A/B/D 三组都能建立通信，没有出现 QoS 不兼容告警。`
- 差异点：
  - `接收比（rx/sent）`：`B(98.67%)` 最好，`A(98.47%)` 次之，`D(97.16%)` 最低。
  - `末段估算丢包率`：`A=1.39%`，`B=1.25%`，`D=1.22%`，三组都出现了可观测丢包。
  - `末段 p95`：`A=10.055ms` 明显高于 `B=2.914ms` 与 `D=3.319ms`，说明 A 组尾延迟波动更大。
- 为什么在高负载下仍然没有“完全拉开”：`三组在单机环境下都还处于可运行区间，差异主要体现在接收比和尾延迟，而不是是否能通信。`

### 8.2 C 组失配分析
- 不兼容触发点：`发布端策略为best_effort,订阅端策略为reliable`
- 现象：`节点提示qos策略不兼容，通信将不被开启`
- 根因（QoS 契约角度）：`qos为向下兼容，订阅端qos策略不能高于发布端`

## 9. 结论（可直接用于复述）
- 结论 1：`qos契约是用于控制信息发送质量调控的，对于高频发送信息的状态类节点，发布端可以设置为best_effort,即尽量发布，不管订阅端有没有接收到；对于模式切换，急停开关等控制类节点，订阅端可以设置为reliable,即必须接收到，如果订阅端没接收到，可要求发布端重新发布消息`
- 结论 2：`qos为向下兼容，订阅端qos策略不能高于发布端`
- 结论 3：`topic,service,action都适用qos,只是topic配置自由，其他两个基本上已经配置好，不用自己配置`

### 9.1 场景化建议（面向机械臂/手术机器人方向）
- 传感流（高频、可容忍少量丢失）：`pub：best_effort;sub：best_effort`
- 控制指令/关键状态（低容错）：`pub：reliable;sub：reliable`

## 10. 踩坑与排查
- 坑 1：`订阅者发布者qos策略不匹配`
  - 现象：`提示requesting incompatible QoS`
  - 根因：`订阅者qos模式为reliable,发布者qos模式为best_effort,订阅者qos模式高于发布者`
  - 修复：`将订阅者或发布者的qos模式调整为兼容的`
- 坑 2：`暂无`
  - 现象：`暂无`
  - 根因：`暂无`
  - 修复：`暂无`

## 11. 个人复盘（学习增量）
- 我之前不知道：`什么是QoS`
- 我现在能解释：`QoS是ros2中的通信质量管理机制`
- 我还不确定：`高负荷状态下QOS的实际效果`
- 我下一步想验证：`高负荷状态下QOS的实际效果`

## 12. 下一步实验计划
- [x] 提高 `publish_rate_hz`（如 200/500）复跑 A/B/D
- [x] 提高 `payload_size_bytes`（如 4096/16384）复跑 A/B/D
- [ ] 补一组跨进程/负载干扰实验
- [ ] 将结论同步到阶段复盘文档

`TODO(human): 选 1-2 项作为 4B 的扩展实验，并写下预期现象`
